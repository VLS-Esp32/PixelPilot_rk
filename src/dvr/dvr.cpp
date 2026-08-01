#include <pthread.h>
#include <poll.h>
#include <unistd.h>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <system_error>
#include <algorithm>
#include <regex>

#include <rockchip/rk_mpi.h>

#include "spdlog/spdlog.h"

#include "dvr.h"

extern "C" {
#include "../osd.h"
}

namespace fs = std::filesystem;

int dvr_enabled = 0;

static const int SEQUENCE_PADDING = 4;   // zero-padding width for sequence-numbered filenames

// Max RPC_FRAME entries allowed to sit in the queue before new frames are dropped. Keeps the DVR
// current: in VideoOnly we carry only the decoder's prime_fd, which the decoder recycles (its
// MAX_FRAMES pool) soon after we enqueue, so a deep backlog would encode a recycled (wrong) buffer.
// Keeping this small bounds DVR lateness to a few frames, well inside the recycle window.
static const size_t DVR_MAX_PENDING_FRAMES = 3;

static const int MP4_TIMEBASE_90K = 90000;  // minimp4 timescale (ticks per second)
static const int MS_TO_90K        = 90;     // 1 ms = 90 ticks at 90kHz

// Cap on a single frame's MP4 duration (90k ticks). A drop burst (e.g. frames lost while the DVR
// thread blocks on segment rotation) inflates seq_delta; without this cap the resulting frame would
// be held for seconds - a freeze. 0.25s is far above any real inter-frame gap, so normal frames are
// unaffected; it only bounds the pathological case.
static const int MAX_FRAME_DURATION_90K = MP4_TIMEBASE_90K / 4;

//Periodic storage guard check runs during recording (free-space / mount check).
static const int64_t  STORAGE_CHECK_INTERVAL_MS = 3000;

// Consecutive failed frame writes (disk full / I/O error) before we fail-stop the recording.
static const uint32_t MAX_CONSECUTIVE_WRITE_FAILURES = 5;

// Max wait for the DRM writeback capture fence to signal (the VOP finished writing the composited
// frame into the WB buffer). A timeout means something is wrong; we drop that frame rather than
// encode a half-written buffer.
static const int WB_FENCE_TIMEOUT_MS = 200;

static int64_t monotonic_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

Dvr::Dvr(dvr_thread_params params)
    : rec_dir(fs::path(params.filename_template).parent_path().string()),
      storage(rec_dir, params.dvr_min_free_bytes, params.dvr_require_mount) {
    filename_template           = params.filename_template;
    mp4_fragmentation_mode      = params.mp4_fragmentation_mode;
    dvr_filenames_with_sequence = params.dvr_filenames_with_sequence;
    dvr_bitrate                 = params.dvr_bitrate;
    segment_limit_ms            = (int64_t)params.dvr_segment_minutes * 60 * 1000;
    if (params.enable_osd_in_dvr && params.enable_wb) {
        // Record the display's composited output (video+OSD) captured via DRM writeback.
        mode = RecordingMode::VideoWithOsdWriteback;
        wb_nv12           = params.wb_nv12;
        wb_enc_width      = params.wb_width;
        wb_enc_height     = params.wb_height;
        wb_enc_hor_stride = params.wb_hor_stride_bytes;
        wb_enc_ver_stride = params.wb_ver_stride;
    } else {
        if (params.enable_osd_in_dvr) {
            spdlog::warn("[ DVR ] OSD-in-DVR requested but writeback unavailable — recording clean video (no OSD)");
        }
        mode = RecordingMode::VideoOnly;
    }

    video_frm_width  = params.video_p.video_frm_width;
    video_frm_height = params.video_p.video_frm_height;
    enc_fps = params.display_fps > 0 ? (int)params.display_fps : 60;
}

Dvr::~Dvr() {}

void Dvr::writeback_frame(dvr_frame_info info) {
    bool dropped = false;
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (dvrQueue.size() >= DVR_MAX_PENDING_FRAMES) {
            dropped = true;
        } else {
            dvrQueue.push({ .command = dvr_rpc::RPC_FRAME, .frame_info = info });
        }
    }
    if (dropped) {
        if (info.fence_fd >= 0) {
            close(info.fence_fd);
        }
        pp_wb_release(info.wb_index);
        return;
    }
    cv.notify_one();
}

void Dvr::frame(dvr_frame_info info) {
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (dvrQueue.size() >= DVR_MAX_PENDING_FRAMES) {
            return; // backlog full — drop this frame to stay current
        }
        dvrQueue.push({ .command = dvr_rpc::RPC_FRAME, .frame_info = info });
    }
    cv.notify_one();
}

// Release the writeback resources a frame carries (capture fence + pool slot). No-op for the
// decode-tap paths, whose frames have fence_fd/wb_index == -1. Must be called for every writeback
// frame that does NOT reach encode_and_write_wb(), or the display thread's pool starves.
static inline void release_wb_frame(dvr_frame_info &fi) {
    if (fi.fence_fd >= 0) {
        close(fi.fence_fd);
        fi.fence_fd = -1;
    }
    if (fi.wb_index >= 0) {
        pp_wb_release(fi.wb_index);
        fi.wb_index = -1;
    }
}

void Dvr::drop_pending_frames() {
    std::queue<dvr_rpc> kept;
    while (!dvrQueue.empty()) {
        if (dvrQueue.front().command != dvr_rpc::RPC_FRAME) {
            kept.push(dvrQueue.front());
        } else {
            release_wb_frame(dvrQueue.front().frame_info); // free WB slot/fence of discarded frames
        }
        dvrQueue.pop();
    }
    dvrQueue.swap(kept);
}

void Dvr::set_video_params(uint32_t video_frm_w, uint32_t video_frm_h) {
    std::lock_guard<std::mutex> lock(mtx);
    video_frm_width = video_frm_w;
    video_frm_height = video_frm_h;
    drop_pending_frames();
}

void Dvr::restart() {
    {
        std::lock_guard<std::mutex> lock(mtx);
        drop_pending_frames();
        dvrQueue.push({ .command = dvr_rpc::RPC_SET_PARAMS });
    }
    cv.notify_one();
}

void Dvr::start_recording() {
    enqueue_dvr_command({ .command = dvr_rpc::RPC_START });
}

void Dvr::stop_recording() {
    dvr_enabled = 0;
    std::lock_guard<std::mutex> lock(mtx);
    while (!dvrQueue.empty()) dvrQueue.pop();
    dvrQueue.push({ .command = dvr_rpc::RPC_STOP });
    cv.notify_one();
}

void Dvr::toggle_recording() {
    enqueue_dvr_command({ .command = dvr_rpc::RPC_TOGGLE });
}

void Dvr::shutdown() {
    dvr_enabled = 0;
    std::lock_guard<std::mutex> lock(mtx);
    while (!dvrQueue.empty()) {
        dvrQueue.pop();
    }
    dvrQueue.push({ .command = dvr_rpc::RPC_SHUTDOWN });
    cv.notify_one();
}

void Dvr::enqueue_dvr_command(dvr_rpc rpc) {
    {
        std::lock_guard<std::mutex> lock(mtx);
        dvrQueue.push(rpc);
    }
    cv.notify_one();
}

void *Dvr::__THREAD__(void *param) {
    pthread_setname_np(pthread_self(), "__DVR");
    ((Dvr *)param)->loop();
    return nullptr;
}

void Dvr::loop() {
    while (true) {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this] { return !this->dvrQueue.empty(); });
        if (dvrQueue.empty()) {
            break;
        }
        dvr_rpc rpc = dvrQueue.front();
        dvrQueue.pop();
        lock.unlock();
        switch (rpc.command) {
        case dvr_rpc::RPC_SET_PARAMS:
            spdlog::debug("[ DVR ] got rpc SET_PARAMS");
            if (writer.is_open())
                rotate_recording_file();
            break;
        case dvr_rpc::RPC_START:
            spdlog::debug("[ DVR ] got rpc START");
            if (writer.is_open())
                break;
            start();
            break;
        case dvr_rpc::RPC_STOP:
            spdlog::debug("[ DVR ] got rpc STOP");
            if (writer.is_open())
                stop();
            break;
        case dvr_rpc::RPC_TOGGLE:
            spdlog::debug("[ DVR ] got rpc TOGGLE");
            if (!writer.is_open()) {
                start(); // encoder init postponed to RPC_FRAME, see RPC_START
            } else {
                stop();
            }
            break;
        case dvr_rpc::RPC_FRAME: {
            // Periodic storage guard check: fail-stop if the mount vanished or free space
            // dropped below the threshold. Uses an I/O-free estimate (free-at-start minus bytes
            // written) rather than statvfs, which on a busy FAT32 card stalls 100ms+ and would
            // stutter the recording. True disk-full is still caught by the write-failure fail-stop.
            if (_ready_to_write) {
                int64_t now_ms = monotonic_ms();
                if (now_ms - last_storage_check_ms >= STORAGE_CHECK_INTERVAL_MS) {
                    last_storage_check_ms = now_ms;
                    uint64_t written = writer.size();
                    uint64_t est_free = session_free_at_start > written ? session_free_at_start - written : 0;
                    if (!storage.mount_ok()) {
                        spdlog::warn("[ DVR ] storage no longer mounted, stopping recording");
                        stop();
                        release_wb_frame(rpc.frame_info);
                        break;
                    }
                    if (!storage.has_enough_free(est_free)) {
                        spdlog::warn("[ DVR ] low free space (~{}MB left, need {}MB), stopping recording",
                                     est_free / (1024 * 1024), storage.min_free() / (1024 * 1024));
                        stop();
                        release_wb_frame(rpc.frame_info);
                        break;
                    }
                }
            }

            // FAT32 per-file size cap: rotate before reaching the 4GB.
            if (_ready_to_write && max_file_bytes > 0 && writer.size() >= max_file_bytes) {
                spdlog::info("[ DVR ] file size cap reached, starting new file");
                rotate_recording_file();
            }

            if (_ready_to_write && segment_limit_ms > 0 &&
                segment_video_ticks >= segment_limit_ms * MS_TO_90K) {
                spdlog::info("[ DVR ] segment time limit reached, starting new file");
                rotate_recording_file();
            }
            if (!_ready_to_write) {
                if (writer.is_open() && video_frm_width > 0 && video_frm_height > 0) {
                    init();
                }
                if (!_ready_to_write) {
                    spdlog::debug("[ DVR ] RPC_FRAME: encoder not ready (awaiting video params), skipping");
                    release_wb_frame(rpc.frame_info);
                    break;
                }
            }
            encode_and_write(rpc.frame_info);

            {
                uint32_t fails = writer.consecutive_write_failures();
                if (fails >= MAX_CONSECUTIVE_WRITE_FAILURES) {
                    spdlog::error("[ DVR ] {} consecutive write failures (disk full or I/O error), stopping recording",
                                  fails);
                    stop();
                }
            }
            break;
        }
        case dvr_rpc::RPC_SHUTDOWN:
            spdlog::debug("[ DVR ] got rpc SHUTDOWN");
            goto end;
        }
    }
end:
    if (writer.is_open()) {
        stop();
    }
    spdlog::info("DVR thread done.");
}

std::string Dvr::generate_filename() {
    fs::path pathObj(filename_template);
    std::string filename_pattern = pathObj.filename().string();
    std::string paddedNumber = "";

    if (!fs::exists(rec_dir)) {
        spdlog::error("[ DVR ] Directory does not exist: {}", rec_dir);
        return "";
    }

    if (dvr_filenames_with_sequence) {
        // Next sequence number = max existing "<digits>_..." prefix + 1.
        std::regex pattern(R"(^(\d+)_.*)");
        int maxNumber = -1;
        for (const auto &entry : fs::directory_iterator(rec_dir)) {
            if (!entry.is_regular_file())
                continue;
            std::string filename = entry.path().filename().string();
            std::smatch match;
            if (std::regex_match(filename, match, pattern))
                maxNumber = std::max(maxNumber, std::stoi(match[1].str()));
        }
        int nextFileNumber = (maxNumber == -1) ? 0 : maxNumber + 1;

        std::ostringstream stream;
        stream << std::setw(SEQUENCE_PADDING) << std::setfill('0') << nextFileNumber;
        paddedNumber = stream.str() + "_";
    }

    std::time_t now = std::time(nullptr);
    char formattedFilename[256];
    std::strftime(formattedFilename, sizeof(formattedFilename),
                  filename_pattern.c_str(), std::localtime(&now));

    return rec_dir + "/" + paddedNumber + formattedFilename;
}

int Dvr::start() {
    std::string reason;
    if (!storage.is_ready(reason)) {
        spdlog::error("[ DVR ] not starting recording: {}", reason);
        osd_publish_bool_fact("dvr.recording", NULL, 0, false);
        return -1;
    }

    std::string mp4_filename = generate_filename();
    if (mp4_filename.empty()) {
        return -1;
    }
    if (!writer.open(mp4_filename, mp4_fragmentation_mode)) {
        return -1;
    }
    current_filename = mp4_filename;

    max_file_bytes = storage.file_size_cap();
    session_free_at_start = storage.free_bytes(); // one statvfs; mid-recording checks estimate from this
    spdlog::info("[ DVR ] storage: {}MB free, per-file cap {}MB",
                 session_free_at_start / (1024 * 1024),
                 max_file_bytes / (1024 * 1024));

    frames_submitted = 0;
    frames_written   = 0;
    rec_start_pts    = -1;   // re-anchor the video clock on this segment's first frame
    while (!submitted_pts.empty()) {
        submitted_pts.pop();
    }

    osd_publish_bool_fact("dvr.recording", NULL, 0, true);
    dvr_enabled = 1;
    return 0;
}

void Dvr::init() {
    int fps = enc_fps;
    if (video_frm_width == 0 || video_frm_height == 0 || fps <= 0) {
        spdlog::warn("[ DVR ] invalid video params {}x{} @{}",
                     video_frm_width, video_frm_height, fps);
        return;
    }


    // Tear down any previous encoder (re-init on resolution change).
    _ready_to_write = 0;
    encoder.cleanup();

    int enc_w, enc_h, enc_hor, enc_ver, mp4_w, mp4_h;
    MppFrameFormat enc_format;
    if (mode == RecordingMode::VideoWithOsdWriteback) {
        // The display thread captures the composited output (video + OSD) the VOP wrote into the
        // writeback buffers at display resolution. Encoder geometry is fixed to those buffers;
        // NV12 goes to the encoder natively, a packed RGB format is HW-converted.
        enc_w = (int)wb_enc_width;
        enc_h = (int)wb_enc_height;
        enc_hor = (int)wb_enc_hor_stride;
        enc_ver = (int)wb_enc_ver_stride;
        mp4_w = (int)wb_enc_width;
        mp4_h = (int)wb_enc_height;
        enc_format = wb_nv12 ? MPP_FMT_YUV420SP : MPP_FMT_BGRA8888;
        spdlog::info("[ DVR ] setting up dvr encoder {}x{} @{}fps bitrate={} H265 [writeback WYSIWYG, {} in]",
                     wb_enc_width, wb_enc_height, fps, dvr_bitrate, wb_nv12 ? "NV12" : "BGRA");
    }
    else {
        enc_w = (int)video_frm_width;
        enc_h = (int)video_frm_height;
        enc_hor = (int)((video_frm_width  + 15) & ~15u);
        enc_ver = (int)((video_frm_height + 15) & ~15u);
        mp4_w = (int)video_frm_width;
        mp4_h = (int)video_frm_height;
        enc_format = MPP_FMT_YUV420SP;
        spdlog::info("[ DVR ] setting up dvr encoder {}x{} @{}fps bitrate={} H265 [zero-copy]",
                     video_frm_width, video_frm_height, fps, dvr_bitrate);
    }

    if (!encoder.init(enc_w, enc_h, enc_hor, enc_ver, fps, dvr_bitrate, enc_format)) {
        return;
    }

    if (!writer.begin_video(mp4_w, mp4_h)) {
        encoder.cleanup();
        return;
    }

    _ready_to_write = 1;
    spdlog::info("[ DVR ] encoder ready");
}

int Dvr::next_frame_duration() {
    if (submitted_pts.empty()) {
        int d = last_good_duration > 0 ? last_good_duration : (MP4_TIMEBASE_90K / enc_fps);
        segment_video_ticks += d;
        return d;
    }
    int64_t pts = submitted_pts.front().pts;
    submitted_pts.pop();

    if (rec_start_pts < 0) {
        rec_start_pts = pts;   // anchor the segment on its first frame
    }
    int64_t target   = (pts - rec_start_pts) * MS_TO_90K;   // real elapsed since segment start (ticks)
    int64_t duration = target - segment_video_ticks;         // close the drift to real time
    if (duration < 1) {
        duration = 1;   // MP4 sample durations must be positive
    }
    if (duration > MAX_FRAME_DURATION_90K) {
        duration = MAX_FRAME_DURATION_90K;
    }
    last_good_duration = (int)duration;
    segment_video_ticks += duration;
    return (int)duration;
}

MppBuffer Dvr::import_decoder_buffer(const dvr_frame_info &info) {
    // Import the decoded frame's DRM buffer directly as encoder input.
    MppBufferInfo buf_info;
    memset(&buf_info, 0, sizeof(buf_info));
    buf_info.type = MPP_BUFFER_TYPE_DRM;
    buf_info.fd   = info.prime_fd;
    buf_info.size = info.buf_size;

    MppBuffer dec_buf = nullptr;
    if (mpp_buffer_import(&dec_buf, &buf_info) || !dec_buf) {
        spdlog::warn("[ DVR ] mpp_buffer_import failed");
        return nullptr;
    }
    return dec_buf;
}

void Dvr::encode_and_write_wb(dvr_frame_info info) {
    // 1) Wait for the VOP to finish writing the composited frame into this WB buffer.
    if (info.fence_fd >= 0) {
        struct pollfd pfd = { info.fence_fd, POLLIN, 0 };
        int pr = poll(&pfd, 1, WB_FENCE_TIMEOUT_MS);
        close(info.fence_fd);
        if (pr <= 0) {
            spdlog::warn("[ DVR ] writeback fence wait failed/timed out ({}), dropping frame", pr);
            pp_wb_release(info.wb_index);
            return;
        }
    }

    // 2) Drain the previous submission (the encoder reads a frame for one more cycle - IPPP
    //    latency, same as the VideoOnly path). After the drain MPP is done with the previous WB
    //    buffer, so release that pool slot back to the display thread.
    encoder.drain([this](const uint8_t *data, int len) {
        if (writer.write_nal(data, len, next_frame_duration())) {
            frames_written++;
        }
    });
    if (wb_pending_index >= 0) {
        pp_wb_release(wb_pending_index);
        wb_pending_index = -1;
    }

    // 3) Import & submit the freshly-composited buffer; hold its slot until the next drain.
    MppBuffer buf = import_decoder_buffer(info);
    if (!buf) {
        pp_wb_release(info.wb_index);
        return;
    }
    int ret = encoder.submit(buf, (int64_t)info.pts,
                             (int)wb_enc_width, (int)wb_enc_height,
                             (int)wb_enc_hor_stride, (int)wb_enc_ver_stride);
    mpp_buffer_put(buf); // encoder holds its own ref; release our import ref
    if (ret == 0) {
        submitted_pts.push({ (int64_t)info.pts, info.frame_seq });
        frames_submitted++;
        wb_pending_index = info.wb_index;
    } else {
        pp_wb_release(info.wb_index);
    }
}

void Dvr::encode_and_write(dvr_frame_info info) {
    if (!encoder.ready() || !_ready_to_write) {
        spdlog::warn("[ DVR ] encode_and_write skipped — encoder not ready");
        release_wb_frame(info); // no-op for decode-tap modes
        return;
    }

    if (mode == RecordingMode::VideoWithOsdWriteback) {
        encode_and_write_wb(info);
        return;
    }

    // VideoOnly: zero-copy, done inline on this thread (drain previous output, import, submit).
    encoder.drain([this](const uint8_t *data, int len) {
        if (writer.write_nal(data, len, next_frame_duration())) {
            frames_written++;
        }
    });

    // Sync encoder config strides to actual decoded frame strides
    if ((int)info.hor_stride != encoder.get_hor_stride() || (int)info.ver_stride != encoder.get_ver_stride()) {
        encoder.sync_strides((int)info.hor_stride, (int)info.ver_stride);
    }

    MppBuffer buf = import_decoder_buffer(info);
    if (!buf) {
        return;
    }
    int ret = encoder.submit(buf, (int64_t)info.pts,
                             (int)info.width, (int)info.height,
                             (int)info.hor_stride, (int)info.ver_stride);
    mpp_buffer_put(buf); // encoder holds its own ref; release ours
    if (ret == 0) {
        submitted_pts.push({ (int64_t)info.pts, info.frame_seq });
        frames_submitted++;
    }
}

void Dvr::finalize_current_file() {
    if (encoder.ready() && _ready_to_write) {
        // Write output from the last submission, then flush the encoder
        encoder.drain([this](const uint8_t *data, int len) {
            if (writer.write_nal(data, len, next_frame_duration())) {
                frames_written++;
            }
        });
        encoder.flush([this](const uint8_t *data, int len) -> bool {
            if (frames_written >= frames_submitted) {
                return false; // all submitted frames accounted for - stop
            }
            if (writer.write_nal(data, len, next_frame_duration())) {
                frames_written++;
            }
            return true;
        });
    }

    // Writeback mode: the last submitted buffer has now been drained/flushed, so release its slot.
    if (wb_pending_index >= 0) {
        pp_wb_release(wb_pending_index);
        wb_pending_index = -1;
    }

    spdlog::info("[ DVR ] recording finalized: {} frames, {:.1f}s",
                 frames_written, segment_video_ticks / 90000.0);

    encoder.cleanup();

    bool empty = (frames_written == 0);
    bool finalized_ok = writer.close();
    _ready_to_write = 0;

    if (!empty && !finalized_ok && !current_filename.empty()) {
        spdlog::error("[ DVR ] recording incomplete (index/moov write failed): {} — may need repair",
                      current_filename);
    }

    if (empty && !current_filename.empty()) {
        std::error_code ec;
        fs::remove(current_filename, ec);
        if (ec) {
            spdlog::warn("[ DVR ] failed to remove empty recording {}: {}", current_filename, ec.message());
        } else {
            spdlog::info("[ DVR ] removed empty recording {}", current_filename);
        }
    }
    current_filename.clear();
    segment_video_ticks = 0;
}

void Dvr::stop() {
    finalize_current_file();
    osd_publish_bool_fact("dvr.recording", NULL, 0, false);
    dvr_enabled = 0;
}

void Dvr::rotate_recording_file() {
    finalize_current_file();
    if (start() != 0) {
        spdlog::error("[ DVR ] failed to open next recording file, stopping recording");
        osd_publish_bool_fact("dvr.recording", NULL, 0, false);
        dvr_enabled = 0;
    }
}

// C-compatible interface
extern "C" {
    void dvr_start_recording(Dvr *dvr) {
        if (dvr) dvr->start_recording();
    }

    void dvr_stop_recording(Dvr *dvr) {
        if (dvr) dvr->stop_recording();
    }
}
