#include <pthread.h>
#include <cstring>
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

// Max RPC_FRAME entries allowed to sit in the queue before new frames are dropped.
// The decoder recycles its buffer pool (MAX_FRAMES) soon after a frame is enqueued and
// we only carry the prime_fd, so a deep backlog would composite recycled (wrong) buffers.
// Keeping this small bounds DVR lateness to a few frames, well inside the recycle window.
static const size_t DVR_MAX_PENDING_FRAMES = 3;

static const int MP4_TIMEBASE_90K = 90000;  // minimp4 timescale (ticks per second)
static const int MS_TO_90K        = 90;     // 1 ms = 90 ticks at 90kHz

static const uint32_t FPS_MIN_FRAMES = 30;
static const int64_t  FPS_MIN_PERIOD_MS = 500;

// Round a measured fps to the closest standard fps rate (e.g. 30, 60) for a clean GOP
static int round_to_standard_fps(int fps) {
    static const int common[] = {24, 25, 30, 48, 50, 60, 90, 120};
    for (int c : common) {
        if (fps >= c - 2 && fps <= c + 2) {
            return c;
        }
    }
    return fps;
}

Dvr::Dvr(dvr_thread_params params) {
    filename_template           = params.filename_template;
    mp4_fragmentation_mode      = params.mp4_fragmentation_mode;
    dvr_filenames_with_sequence = params.dvr_filenames_with_sequence;
    dvr_bitrate                 = params.dvr_bitrate;
    segment_limit_ms            = (int64_t)params.dvr_segment_minutes * 60 * 1000;
    mode = params.enable_osd_in_dvr ? RecordingMode::VideoWithOsd : RecordingMode::VideoOnly;

    video_frm_width  = params.video_p.video_frm_width;
    video_frm_height = params.video_p.video_frm_height;
    // Recording resolution - display resolution when available, otherwise native video.
    disp_width  = (params.display_width  > 0) ? params.display_width  : params.video_p.video_frm_width;
    disp_height = (params.display_height > 0) ? params.display_height : params.video_p.video_frm_height;
}

Dvr::~Dvr() {}

void Dvr::frame(dvr_frame_info info) {
    if (fps_measure_first_pts < 0) {
        fps_measure_first_pts = (int64_t)info.pts;
        fps_measure_count = 0;
    }

    fps_measure_count++;
    int64_t period_ms = (int64_t)info.pts - fps_measure_first_pts;
    if (fps_measure_count >= FPS_MIN_FRAMES && period_ms >= FPS_MIN_PERIOD_MS) {
        int fps = (int)(((int64_t)(fps_measure_count - 1) * 1000 + period_ms / 2) / period_ms);
        if (fps < 1) {
            fps = 1;
        }
        if (fps > 120) {
            fps = 120;
        }
        detected_fps.store(round_to_standard_fps(fps));
        // Reset the window so the estimate keeps tracking the stream.
        fps_measure_first_pts = (int64_t)info.pts;
        fps_measure_count = 0;
    }

    // Backpressure - we carry only the decoder buffer's prime_fd, and the decoder recycles
    // that buffer (MAX_FRAMES pool) shortly after we enqueue. If the encode pipeline falls
    // behind (e.g. OSD blend can't sustain 60fps), a backlogged frame's fd gets reused for a
    // newer decoded frame before we composite it -> we'd read the wrong (jumped-ahead)
    // content and the queue would only grow. Cap the backlog so the DVR always processes
    // recent frames; dropped frames are absorbed by the real-PTS frame durations.
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (dvrQueue.size() >= DVR_MAX_PENDING_FRAMES) {
            return; // pipeline behind — drop this frame to stay current
        }
        dvrQueue.push({ .command = dvr_rpc::RPC_FRAME, .frame_info = info });
    }
    cv.notify_one();
}

void Dvr::drop_pending_frames() {
    std::queue<dvr_rpc> kept;
    while (!dvrQueue.empty()) {
        if (dvrQueue.front().command != dvr_rpc::RPC_FRAME) {
            kept.push(dvrQueue.front());
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

    fps_measure_first_pts = -1;
    fps_measure_count = 0;
}

void Dvr::restart() {
    {
        std::lock_guard<std::mutex> lock(mtx);
        drop_pending_frames();
        fps_measure_first_pts = -1;
        fps_measure_count = 0;
        detected_fps.store(0);
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
            int64_t pts = (int64_t)rpc.frame_info.pts;
            if (_ready_to_write && segment_limit_ms > 0 && segment_start_pts >= 0 &&
                pts - segment_start_pts >= segment_limit_ms) {
                spdlog::info("[ DVR ] segment time limit reached, starting new file");
                rotate_recording_file();
            }
            if (!_ready_to_write) {
                if (writer.is_open() && video_frm_width > 0 && video_frm_height > 0 &&
                    detected_fps.load() > 0) {
                    init();
                }
                if (!_ready_to_write) {
                    spdlog::debug("[ DVR ] RPC_FRAME: _ready_to_write=0 measuring fps, fps={}), skipping",
                                  detected_fps.load());
                    break; // awaiting fps - drop this frame
                }
            }
            if (segment_start_pts < 0) {
                segment_start_pts = pts;
            }
            encode_and_write(rpc.frame_info);
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
    std::string rec_dir = pathObj.parent_path().string();
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
    std::string mp4_filename = generate_filename();
    if (mp4_filename.empty()) {
        return -1;
    }
    if (!writer.open(mp4_filename, mp4_fragmentation_mode)) {
        return -1;
    }
    current_filename = mp4_filename;

    frames_submitted = 0;
    frames_written   = 0;
    last_written_pts = -1;
    while (!submitted_pts.empty()) {
        submitted_pts.pop();
    }

    osd_publish_bool_fact("dvr.recording", NULL, 0, true);
    dvr_enabled = 1;
    return 0;
}

void Dvr::init() {
    int fps = detected_fps.load();
    if (video_frm_width == 0 || video_frm_height == 0 || fps <= 0) {
        spdlog::warn("[ DVR ] invalid video params {}x{} @{}",
                     video_frm_width, video_frm_height, fps);
        return;
    }

    // Tear down any previous encoder/compositor (re-init on resolution change).
    _ready_to_write = 0;
    encoder.cleanup();
    osd.cleanup();

    int enc_w, enc_h, enc_hor, enc_ver, mp4_w, mp4_h;
    if (mode == RecordingMode::VideoWithOsd) {
        if (!osd.init((int)disp_width, (int)disp_height,
            (int)video_frm_width, (int)video_frm_height)) {
            return;
        }
        enc_w = (int)disp_width; 
        enc_h = (int)disp_height;
        enc_hor = osd.get_enc_hor_stride(); 
        enc_ver = osd.get_enc_ver_stride();
        mp4_w = (int)disp_width;
        mp4_h = (int)disp_height;
        spdlog::info("[ DVR ] setting up dvr encoder {}x{} (video {}x{}) @{}fps bitrate={} H265 [OSD+RGA]",
                     disp_width, disp_height, video_frm_width, video_frm_height,
                     fps, dvr_bitrate);
    }
    else {
        enc_w = (int)video_frm_width;
        enc_h = (int)video_frm_height;
        enc_hor = (int)((video_frm_width  + 15) & ~15u);
        enc_ver = (int)((video_frm_height + 15) & ~15u);
        mp4_w = (int)video_frm_width;
        mp4_h = (int)video_frm_height;
        spdlog::info("[ DVR ] setting up dvr encoder {}x{} @{}fps bitrate={} H265 [zero-copy]",
                     video_frm_width, video_frm_height, fps, dvr_bitrate);
    }

    if (!encoder.init(enc_w, enc_h, enc_hor, enc_ver, fps, dvr_bitrate)) {
        osd.cleanup();
        return;
    }

    if (!writer.begin_video(mp4_w, mp4_h)) {
        encoder.cleanup();
        osd.cleanup();
        return;
    }

    _ready_to_write = 1;
    spdlog::info("[ DVR ] encoder ready");
}

int Dvr::next_frame_duration() {
    // Duration (in 90kHz MP4 ticks) for the next frame written to the MP4, derived from the
    // real wall-clock PTS delta between consecutive frames. This makes the recorded duration
    // match real time regardless of the actual capture rate or any dropped frames — unlike a
    // fixed 90000/fps, which fast-forwards when fewer frames are produced than the nominal
    // rate (and slows down when more are). PTS is in ms; 1ms = 90 ticks at 90kHz. The first
    // frame and any implausible delta fall back to the nominal 1/fps duration. Packets are
    // emitted in submission order (IPPP, no B-frames), so the FIFO front always matches the
    // packet being written.
    int duration = MP4_TIMEBASE_90K / detected_fps.load();
    if (!submitted_pts.empty()) {
        int64_t pts = submitted_pts.front();
        submitted_pts.pop();
        if (last_written_pts >= 0) {
            int64_t delta = pts - last_written_pts;
            if (delta > 0 && delta < 10000) { // sane gap: < 10s between frames
                duration = (int)(delta * MS_TO_90K);
            }
        }
        last_written_pts = pts;
    }
    return duration;
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

void Dvr::encode_and_write(dvr_frame_info info) {
    if (!encoder.ready() || !_ready_to_write) {
        spdlog::warn("[ DVR ] encode_and_write skipped — encoder not ready");
        return;
    }

    // Write any packets produced by the previous submission (it has had a full frame
    // interval to complete).
    encoder.drain([this](const uint8_t *data, int len) {
        if (writer.write_nal(data, len, next_frame_duration())) {
            frames_written++;
        }
    });

    if (mode == RecordingMode::VideoWithOsd) {
        MppBuffer buf = osd.compose(info);
        if (!buf) {
            return;
        }
        if (encoder.submit(buf, (int64_t)info.pts,
                           osd.get_disp_width(), osd.get_disp_height(),
                           osd.get_enc_hor_stride(), osd.get_enc_ver_stride()) == 0) {
            submitted_pts.push((int64_t)info.pts);
            frames_submitted++;
        }
    }
    else {
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
            submitted_pts.push((int64_t)info.pts);
            frames_submitted++;
        }
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

    encoder.cleanup();
    osd.cleanup();

    bool empty = (frames_written == 0);
    writer.close();
    _ready_to_write = 0;

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
    segment_start_pts = -1;
}

void Dvr::stop() {
    finalize_current_file();
    osd_publish_bool_fact("dvr.recording", NULL, 0, false);
    dvr_enabled = 0;
}

void Dvr::rotate_recording_file() {
    finalize_current_file();
    if (start() != 0) {
        spdlog::error("[ DVR ] failed to open next recording file");
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
