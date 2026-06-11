#ifndef DVR_OSD_COMPOSITOR_H
#define DVR_OSD_COMPOSITOR_H

#include <cstdint>

#include <rockchip/rk_mpi.h>

#include "dvr_common.h"

// OsdCompositor - RGA pipeline that scales/letterboxes the decoded video into a
// display-sized frame and blends the OSD overlay on top, producing an NV12 buffer
// ready for the encoder. Owns its own MPP buffer group and a double-buffer so the
// encoder can read frame N while we compose frame N+1.
class OsdCompositor {
public:
    // Allocate buffers and compute the letterbox geometry for fitting video_w x video_h
    // frames into a disp_w x disp_h encoder frame. Returns false on failure.
    bool init(int disp_w, int disp_h, int video_w, int video_h);

    // Composite one decoded frame with the current OSD overlay. Returns the NV12 buffer
    // to submit to the encoder, or nullptr on failure.
    MppBuffer compose(const dvr_frame_info &info);

    int get_enc_hor_stride() const { return enc_hor_stride_; }
    int get_enc_ver_stride() const { return enc_ver_stride_; }
    int get_disp_width()     const { return disp_w_; }
    int get_disp_height()    const { return disp_h_; }

    void cleanup();

private:
    void clear_letterbox_bars(uint8_t *bgra);

    MppBufferGroup buf_grp = nullptr;
    MppBuffer input_buf[2] = {nullptr, nullptr};
    int       buf_index = 0;
    MppBuffer bgra_buf = nullptr;

    int disp_w_ = 0;
    int disp_h_ = 0;
    int enc_hor_stride_ = 0;
    int enc_ver_stride_ = 0;
    // letterbox rect of the video within the frame
    int lb_x_ = 0;
    int lb_y_ = 0;
    int lb_w_ = 0;
    int lb_h_ = 0;
};

#endif
