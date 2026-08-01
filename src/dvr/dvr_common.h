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
    uint64_t frame_seq;  // monotonic decoded-frame index; gives even, jitter-free frame spacing
    // Writeback path only (VideoWithOsdWriteback): capture-completion fence and pool slot to
    // release once encoded. -1 for the decode-tap VideoOnly path.
    int      fence_fd = -1;
    int      wb_index = -1;
};

struct dvr_thread_params {
    char *filename_template;
    int mp4_fragmentation_mode = 0;
    bool dvr_filenames_with_sequence = false;
    bool enable_osd_in_dvr = false;
    int dvr_bitrate = 8000000;
    int dvr_segment_minutes = 0;
    uint64_t dvr_min_free_bytes = 200ULL * 1024 * 1024;
    bool dvr_require_mount = false;
    uint32_t display_width = 0;
    uint32_t display_height = 0;
    uint32_t display_fps = 0;   // display refresh rate (Hz); the DVR encodes at this rate
    // Writeback WYSIWYG capture (set when a DRM writeback connector was probed and OSD recording is
    // requested). The DVR encodes the composited display output the VOP wrote into the WB buffers.
    // Geometry is the WB buffer's; wb_nv12 picks the encoder input format (NV12 native, else BGRA
    // with HW convert).
    bool     enable_wb = false;
    bool     wb_nv12 = false;
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
        RPC_SET_PARAMS
    } command;
    dvr_frame_info frame_info;
};

enum class RecordingMode {
    VideoOnly,             // zero-copy - decoded frame submitted straight to the encoder
    VideoWithOsdWriteback  // DRM writeback - encode the composited display output (video+OSD)
};

extern int dvr_enabled;

// Release a writeback buffer pool slot back to the display thread once the DVR has finished
// encoding it. Implemented in main.cpp; called from the DVR thread. Invalid index is ignored.
void pp_wb_release(int index);

#endif
