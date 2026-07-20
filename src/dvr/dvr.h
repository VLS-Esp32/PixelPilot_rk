#ifndef DVR_H
#define DVR_H

#include <queue>
#include <mutex>
#include <atomic>
#include <string>
#include <condition_variable>

#include "dvr_common.h"
#include "mpp_encoder.h"
#include "osd_compositor.h"
#include "mp4_writer.h"
#include "storage_guard.h"

class Dvr {
public:
    explicit Dvr(dvr_thread_params params);
    virtual ~Dvr();

    void frame(dvr_frame_info info);
    void set_video_params(uint32_t video_frm_width, uint32_t video_frm_height);
    void restart();
    void start_recording();
    void stop_recording();
    void toggle_recording();
    void shutdown();

    static void *__THREAD__(void *context);
private:
    void enqueue_dvr_command(dvr_rpc rpc);
    void drop_pending_frames();

    void loop();
    int  start();
    void stop();
    void rotate_recording_file();
    void finalize_current_file();
    void init();
    std::string generate_filename();
    int  next_frame_duration();
    MppBuffer import_decoder_buffer(const dvr_frame_info &info);
    void encode_and_write(dvr_frame_info info);

    std::queue<dvr_rpc> dvrQueue;
    std::mutex mtx;
    std::condition_variable cv;

    char *filename_template;
    int  mp4_fragmentation_mode = 0;
    bool dvr_filenames_with_sequence = false;
    int  dvr_bitrate = 8000000;
    int64_t segment_limit_ms = 0;
    int64_t segment_start_pts = -1;
    RecordingMode mode = RecordingMode::VideoOnly;

    std::string rec_dir;
    StorageGuard storage;
    uint64_t max_file_bytes = 0;
    int64_t  last_storage_check_ms = 0;

    std::atomic<int> detected_fps{0};
    int64_t  fps_measure_first_pts = -1;
    uint32_t fps_measure_count = 0;

    uint32_t video_frm_width = 0;
    uint32_t video_frm_height = 0;
    uint32_t disp_width = 0;   // encoder output: display res, or video native if no display
    uint32_t disp_height = 0;

    int _ready_to_write = 0;

    MppEncoder    encoder;
    OsdCompositor osd;
    Mp4Writer     writer;

    std::string current_filename;

    uint32_t frames_submitted = 0;
    uint32_t frames_written   = 0;
    std::queue<int64_t> submitted_pts;   // FIFO of submitted frame PTS (ms), in encode order
    int64_t  last_written_pts = -1;      // PTS (ms) of last frame written to the MP4
};

#endif
