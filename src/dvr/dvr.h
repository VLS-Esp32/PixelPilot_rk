#ifndef DVR_H
#define DVR_H

#include <queue>
#include <mutex>
#include <string>
#include <condition_variable>

#include "dvr_common.h"
#include "mpp_encoder.h"
#include "mp4_writer.h"
#include "storage_guard.h"

class Dvr {
public:
    explicit Dvr(dvr_thread_params params);
    ~Dvr();

    void frame(dvr_frame_info info);
    // Writeback (WYSIWYG) ingress: fed the composited display output from the display thread
    // (VideoWithOsdWriteback mode) instead of decoded frames from the decode thread.
    void writeback_frame(dvr_frame_info info);
    void set_video_params(uint32_t video_frm_width, uint32_t video_frm_height);
    void restart();
    void start_recording();
    void stop_recording();
    void toggle_recording();
    // Latch the DVR off for the rest of the process from any thread (the display thread uses this
    // when writeback commits keep failing). Returns immediately; the DVR thread finalizes the file.
    void disable(const std::string &reason);
    void shutdown();

    static void *__THREAD__(void *context);
private:
    void enqueue_dvr_command(dvr_rpc rpc);
    void drop_pending_frames();

    void loop();
    int  start();
    bool open_output_file();
    void stop();
    void fail(const std::string &reason, bool fatal);
    void rotate_recording_file();
    void finalize_current_file();
    void init();
    std::string generate_filename();
    int  next_frame_duration();
    MppBuffer import_decoder_buffer(const dvr_frame_info &info);
    void encode_and_write(dvr_frame_info info);
    void encode_and_write_wb(dvr_frame_info info);
    void update_storage_status(bool force_update);

    std::queue<dvr_rpc> dvrQueue;
    std::mutex mtx;
    std::condition_variable cv;

    char *filename_template;
    int  mp4_fragmentation_mode = 0;
    int  dvr_bitrate = 8000000;
    int64_t segment_limit_ms = 0;
    int64_t segment_video_ticks = 0;
    RecordingMode mode = RecordingMode::VideoOnly;

    std::string rec_dir;
    StorageGuard storage;
    uint64_t max_file_bytes = 0;
    uint64_t session_free_at_start = 0;  // free bytes at recording start; mid-recording est baseline
    bool     session_free_known = false; // false if statvfs failed: skip the free-space estimates
    dev_t    session_dev = 0;
    bool     session_dev_known = false;
    int64_t  last_storage_check_ms = 0;

    dev_t storage_status_dev = 0;
    bool storage_status_dev_known = false;
    uint64_t storage_total_bytes = 0;
    bool storage_total_known = false;

    std::string recording_storage_error;

    bool recording_armed = false;

    // Recording frame rate = the display refresh rate (the writeback recording is a screen capture,
    // so its natural rate is the display's).
    int enc_fps = 0;

    uint32_t video_frm_width = 0;
    uint32_t video_frm_height = 0;

    // Writeback mode geometry (VideoWithOsdWriteback): the composited buffer the display thread
    // hands us, always NV12.
    uint32_t wb_enc_width = 0;
    uint32_t wb_enc_height = 0;
    uint32_t wb_enc_hor_stride = 0;   // bytes (Y stride for NV12, or BGRA byte stride)
    uint32_t wb_enc_ver_stride = 0;   // aligned rows (matches the WB buffer's CbCr plane offset)
    int      wb_pending_index = -1;

    int _ready_to_write = 0;
    int init_attempts = 0;        // failed encoder/muxer setups for the current file
    int frame_error_streak = 0;   // consecutive frames lost to import/fence/submit errors

    MppEncoder    encoder;
    Mp4Writer     writer;

    std::string current_filename;

    uint32_t frames_submitted = 0;
    uint32_t frames_written   = 0;   // frames handed to the writer (enqueued), not yet on disk
    std::queue<int64_t> submitted_pts;   // FIFO of submitted frame pts (ms), encode order
    int64_t  rec_start_pts = -1;         // feed-pts (ms) of the first frame of the current segment
    int      last_good_duration = 0;     // last computed duration (90k ticks); fallback
};

#endif
