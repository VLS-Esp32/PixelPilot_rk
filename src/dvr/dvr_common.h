#ifndef DVR_COMMON_H
#define DVR_COMMON_H

#include <cstdint>
#include <cstddef>

struct video_params {
    uint32_t video_frm_width;
    uint32_t video_frm_height;
};

struct dvr_frame_info {
    int      prime_fd;
    uint32_t hor_stride;
    uint32_t ver_stride;
    uint32_t width;
    uint32_t height;
    size_t   buf_size;
    uint64_t pts;
};

struct dvr_thread_params {
    char *filename_template;
    int mp4_fragmentation_mode = 0;
    bool dvr_filenames_with_sequence = false;
    bool enable_osd_in_dvr = false;
    int dvr_bitrate = 8000000;
    int dvr_segment_minutes = 0;
    uint32_t display_width = 0;
    uint32_t display_height = 0;
    video_params video_p;
};

struct dvr_rpc {
    enum {
        RPC_FRAME,
        RPC_STOP,
        RPC_START,
        RPC_TOGGLE,
        RPC_SHUTDOWN,
        RPC_SET_PARAMS
    } command;
    dvr_frame_info frame_info;
};

// What gets recorded each frame.
enum class RecordingMode {
    VideoOnly,     // zero-copy - decoded frame submitted straight to the encoder
    VideoWithOsd   // RGA pipeline - video scaled + letterboxed with the OSD blended on top
};

extern int dvr_enabled;

#endif
