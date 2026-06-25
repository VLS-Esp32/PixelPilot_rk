#include <cstdint>
#include <cstring>
#include <algorithm>
#include <pthread.h>
#include <unistd.h>
#include <sys/mman.h>

#include <rockchip/rk_mpi.h>
#include <rga/im2d.h>

#include "spdlog/spdlog.h"

#include "osd_compositor.h"
#include "../drm.h"

extern struct modeset_output *output_list;
extern pthread_mutex_t osd_mutex;

static const uint8_t NV12_BLACK_Y  = 0x10;
static const uint8_t NV12_BLACK_UV = 0x80;

bool OsdCompositor::init(int disp_w, int disp_h, int video_w, int video_h) {
    disp_w_ = disp_w;
    disp_h_ = disp_h;
    enc_hor_stride_ = (disp_w_ + 63) & ~63;
    enc_ver_stride_ = (disp_h_ + 63) & ~63;

    // Letterbox: largest centered rect with the videos aspect ratio inside the display.
    lb_w_ = disp_w_;
    lb_h_ = disp_h_;
    float ratio = (float)video_w / (float)video_h;
    if ((float)lb_w_ / ratio > (float)lb_h_) {
        lb_w_ = ((int)((float)lb_h_ * ratio)) & ~1;
    }
    else {
        lb_h_ = ((int)((float)lb_w_ / ratio)) & ~1;
    }
    lb_x_ = ((disp_w_ - lb_w_) / 2) & ~1;
    lb_y_ = ((disp_h_ - lb_h_) / 2) & ~1;

    int ret = mpp_buffer_group_get_internal(&buf_grp, MPP_BUFFER_TYPE_DRM);
    if (ret) {
        spdlog::error("[ DVR OsdCompositor ] mpp_buffer_group_get_internal failed {}", ret);
        return false;
    }

    // Two NV12 encoder-input buffers (double-buffered) so the encoder can read one
    // while we compose into the other. Pre-fill black so letterbox bars start clean.
    for (int i = 0; i < 2; i++) {
        ret = mpp_buffer_get(buf_grp, &input_buf[i],
                             (size_t)enc_hor_stride_ * enc_ver_stride_ * 2);
        if (ret) {
            spdlog::error("[ DVR OsdCompositor ] mpp_buffer_get failed {}", ret);
            cleanup();
            return false;
        }
        uint8_t *p = (uint8_t *)mpp_buffer_get_ptr(input_buf[i]);
        memset(p, NV12_BLACK_Y, (size_t)enc_hor_stride_ * enc_ver_stride_);
        memset(p + (size_t)enc_hor_stride_ * enc_ver_stride_, NV12_BLACK_UV,
               (size_t)enc_hor_stride_ * enc_ver_stride_ / 2);
    }
    buf_index = 0;

    if (mpp_buffer_get(buf_grp, &bgra_buf, (size_t)disp_w_ * disp_h_ * 4)) {
        spdlog::warn("[ DVR OsdCompositor ] failed to allocate OSD blend buffer — OSD will be disabled");
        bgra_buf = nullptr;
    }

    spdlog::info("[ DVR OsdCompositor ] OSD compositor ready: {}x{} (video letterboxed to {}x{}+{},{})",
                 disp_w_, disp_h_, lb_w_, lb_h_, lb_x_, lb_y_);
    return true;
}

void OsdCompositor::clear_letterbox_bars(uint8_t *bgra) {
    // Clear only the letterbox bars (regions outside the video rect), not the whole
    // frame. The video rect is fully overwritten by improcess below, so only the bars
    // need re-blacking to stop OSD pixels accumulating there. Clearing the full
    // disp_w*disp_h*4 (~8MB) on CPU costs ~15ms/frame and capped the achievable fps.
    const size_t row_bytes = (size_t)disp_w_ * 4;
    if (lb_y_ > 0) {                                       // top bar
        memset(bgra, 0, (size_t)lb_y_ * row_bytes);
    }
    int bb_start = lb_y_ + lb_h_;
    if (bb_start < disp_h_) {                              // bottom bar
        memset(bgra + (size_t)bb_start * row_bytes, 0, (size_t)(disp_h_ - bb_start) * row_bytes);
    }
    int rb_start = lb_x_ + lb_w_;
    if (lb_x_ > 0 || rb_start < disp_w_) {                // left/right bars (pillarbox)
        size_t left_bytes  = (size_t)lb_x_ * 4;
        size_t right_bytes = (size_t)(disp_w_ - rb_start) * 4;
        for (int y = lb_y_; y < lb_y_ + lb_h_; y++) {
            uint8_t *row = bgra + (size_t)y * row_bytes;
            if (left_bytes) {
                memset(row, 0, left_bytes);
            }
            if (right_bytes) {
                memset(row + (size_t)rb_start * 4, 0, right_bytes);
            }
        }
    }
}

MppBuffer OsdCompositor::compose(const dvr_frame_info &info) {
    MppBuffer out = input_buf[buf_index];
    if (!out) {
        return nullptr;
    }
    void *enc_ptr = mpp_buffer_get_ptr(out);
    if (!enc_ptr) {
        spdlog::warn("[ DVR OsdCompositor ] mpp_buffer_get_ptr returned null");
        return nullptr;
    }

    // Source - decoded NV12 frame at native video resolution.
    rga_buffer_t src_vid = wrapbuffer_fd(info.prime_fd,
                                         (int)info.width, (int)info.height,
                                         RK_FORMAT_YCbCr_420_SP,
                                         (int)info.hor_stride, (int)info.ver_stride);
    // Destination - encoder input buffer at display resolution.
    rga_buffer_t dst_enc = wrapbuffer_virtualaddr(enc_ptr,
                                                  disp_w_, disp_h_,
                                                  RK_FORMAT_YCbCr_420_SP,
                                                  enc_hor_stride_, enc_ver_stride_);

    im_rect      src_rect   = {0, 0, (int)info.width, (int)info.height};
    im_rect      lb_rect    = {lb_x_, lb_y_, lb_w_, lb_h_};
    rga_buffer_t empty_buf  = {};
    im_rect      empty_rect = {};

    bool copy_ok = false;
    if (output_list && bgra_buf) {
        pthread_mutex_lock(&osd_mutex);
        unsigned int osd_idx = output_list->osd_buf_switch;
        pthread_mutex_unlock(&osd_mutex);

        struct modeset_buf *osd_buf = &output_list->osd_bufs[osd_idx];
        if (osd_buf->prime_fd >= 0 && osd_buf->width > 0) {
            clear_letterbox_bars((uint8_t *)mpp_buffer_get_ptr(bgra_buf));

            rga_buffer_t src_osd = wrapbuffer_fd(osd_buf->prime_fd,
                                                 (int)osd_buf->width, (int)osd_buf->height,
                                                 RK_FORMAT_BGRA_8888,
                                                 (int)(osd_buf->stride / 4), (int)osd_buf->height);
            rga_buffer_t rga_bgra = wrapbuffer_virtualaddr(
                mpp_buffer_get_ptr(bgra_buf), disp_w_, disp_h_, RK_FORMAT_BGRA_8888);

            // Scale NV12 video into the letterbox region of the BGRA frame
            IM_STATUS s1 = improcess(src_vid, rga_bgra, empty_buf,
                                     src_rect, lb_rect, empty_rect, 0);
            // Blend display-res OSD over the BGRA frame (no scaling - same size)
            IM_STATUS s2 = (s1 == IM_STATUS_SUCCESS)
                ? imblend(src_osd, rga_bgra, IM_ALPHA_BLEND_SRC_OVER)
                : IM_STATUS_FAILED;
            // Convert blended BGRA -> NV12 for the encoder
            IM_STATUS s3 = (s2 == IM_STATUS_SUCCESS)
                ? imcvtcolor(rga_bgra, dst_enc, RK_FORMAT_BGRA_8888, RK_FORMAT_YCbCr_420_SP)
                : IM_STATUS_FAILED;

            if (s3 == IM_STATUS_SUCCESS) {
                copy_ok = true;
            }
            else {
                spdlog::warn("[ DVR OsdCompositor ] OSD pipeline failed steps {}/{}/{}, falling back to plain copy",
                             (int)s1, (int)s2, (int)s3);
            }
        }
    }

    if (!copy_ok) {
        // No OSD available (or blend failed) - scale the video straight into the encoder
        // buffer; if even that fails, fall back to a CPU copy of the raw frame.
        IM_STATUS rga_ret = improcess(src_vid, dst_enc, empty_buf,
                                      src_rect, lb_rect, empty_rect, 0);
        if (rga_ret == IM_STATUS_SUCCESS) {
            copy_ok = true;
        } else {
            spdlog::warn("[ DVR OsdCompositor ] RGA scale failed ({}), using CPU fallback", (int)rga_ret);
            long real_size = lseek(info.prime_fd, 0, SEEK_END);
            if (real_size < 0 || (size_t)real_size < info.buf_size) {
                spdlog::warn("[ DVR OsdCompositor ] deprecated/undersized buffer (fd {} real {} < {}), dropping frame",
                             info.prime_fd, real_size, info.buf_size);
                return nullptr;
            }
            void *src_ptr = mmap(nullptr, info.buf_size, PROT_READ, MAP_SHARED, info.prime_fd, 0);
            if (src_ptr != MAP_FAILED) {
                size_t copy_sz = std::min(info.buf_size,
                                          (size_t)enc_hor_stride_ * enc_ver_stride_ * 2);
                memcpy(enc_ptr, src_ptr, copy_sz);
                munmap(src_ptr, info.buf_size);
                copy_ok = true;
            }
            else {
                spdlog::warn("[ DVR OsdCompositor ] mmap failed for prime_fd {}", info.prime_fd);
                return nullptr;
            }
        }
    }

    buf_index ^= 1;
    return out;
}

void OsdCompositor::cleanup() {
    if (bgra_buf) {
        mpp_buffer_put(bgra_buf);
        bgra_buf = nullptr;
    }
    for (int i = 0; i < 2; i++) {
        if (input_buf[i]) {
            mpp_buffer_put(input_buf[i]);
            input_buf[i] = nullptr;
        }
    }
    if (buf_grp) {
        mpp_buffer_group_put(buf_grp);
        buf_grp = nullptr;
    }
    buf_index = 0;
}
