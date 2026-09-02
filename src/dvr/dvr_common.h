#ifndef DVR_COMMON_H
#define DVR_COMMON_H

#include <cstdint>
#include <cstddef>
#include <atomic>

struct video_params {
    uint32_t video_frm_width;
    uint32_t video_frm_height;
};

struct dvr_frame_info {
    int      prime_fd;
    // Geometry, VideoOnly (decode-tap) only: the writeback path leaves these zero and encodes with
    // the fixed wb_enc_* geometry of the writeback buffers instead.
    uint32_t hor_stride;
    uint32_t ver_stride;
    uint32_t width;
    uint32_t height;
    size_t   buf_size;
    uint64_t pts;
    // Writeback path only (VideoWithOsdWriteback): capture-completion fence and pool slot to
    // release once encoded. -1 for the decode-tap VideoOnly path.
    int      fence_fd = -1;
    int      wb_index = -1;
};

struct dvr_thread_params {
    char *filename_template;
    int mp4_fragmentation_mode = 0;
    bool enable_osd_in_dvr = false;
    int dvr_bitrate = 8000000;
    int dvr_segment_minutes = 0;
    uint64_t dvr_min_free_bytes = 200ULL * 1024 * 1024;
    bool dvr_require_mount = false;
    uint32_t display_fps = 0;   // display refresh rate (Hz); the DVR encodes at this rate
    // Writeback WYSIWYG capture (set when a DRM writeback connector was probed and OSD recording is
    // requested). The DVR encodes the composited display output the VOP wrote into the WB buffers.
    // Geometry is the WB buffer's; the format is always NV12 (encoder-native).
    bool     enable_wb = false;
    uint32_t wb_width = 0;
    uint32_t wb_height = 0;
    uint32_t wb_hor_stride_bytes = 0;
    uint32_t wb_ver_stride = 0;
    video_params video_p;
};

struct dvr_rpc {
    enum {
        RPC_FRAME,
        RPC_STOP,
        RPC_START,
        RPC_TOGGLE,
        RPC_SHUTDOWN,
        RPC_SET_PARAMS,
        RPC_DISABLE     // another thread hit an unrecoverable DVR fault; finalize and stay off
    } command;
    dvr_frame_info frame_info;
};

enum class RecordingMode {
    VideoOnly,             // zero-copy - decoded frame submitted straight to the encoder
    VideoWithOsdWriteback  // DRM writeback - encode the composited display output (video+OSD)
};

// The single cross-thread DVR state. Written by the DVR thread (start/stop/fail), the mavlink
// thread (stop_recording) and main (shutdown); read by the decode and display threads to decide
// whether to feed frames. Disabled is a latch: an unrecoverable failure sets it and nothing clears
// it for the rest of the process, so a broken DVR cannot retry in a loop.
enum class DvrState {
    Idle,       // not recording, but a start request would be honoured
    Recording,  // recording; ingress paths should feed frames
    Disabled    // unrecoverable failure - ignore every start/toggle/frame until restart
};

extern std::atomic<DvrState> dvr_state;

inline bool dvr_is_recording() {
    return dvr_state.load(std::memory_order_acquire) == DvrState::Recording;
}

inline bool dvr_is_disabled() {
    return dvr_state.load(std::memory_order_acquire) == DvrState::Disabled;
}

// Release a writeback buffer pool slot back to the display thread once the DVR has finished
// encoding it. Implemented in main.cpp; called from the DVR thread. Invalid index is ignored.
void pp_wb_release(int index);

#endif
