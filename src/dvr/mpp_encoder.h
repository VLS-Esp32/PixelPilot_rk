#ifndef DVR_MPP_ENCODER_H
#define DVR_MPP_ENCODER_H

#include <cstdint>
#include <functional>

#include <rockchip/rk_mpi.h>

class MppEncoder {
public:
    // Configure an H265 encoder for the given input geometry. Input is always NV12 (MPP_FMT_YUV420SP)
    // - both the DRM writeback buffers and the decoder's zero-copy output use it - so `hor_stride`
    // is in pixels.
    bool init(int width, int height, int hor_stride, int ver_stride, int fps, int bitrate);

    // Re-publish input strides at runtime — the zero-copy decoder may align its
    // buffers differently than the estimate used at init().
    void sync_strides(int hor_stride, int ver_stride);
    int  get_hor_stride() const { return cfg_hor_stride; }
    int  get_ver_stride() const { return cfg_ver_stride; }

    int submit(MppBuffer buf, int64_t pts, int width, int height,
               int hor_stride, int ver_stride);
    void drain(const std::function<void(const uint8_t *, int)> &on_nal);
    void flush(const std::function<bool(const uint8_t *, int)> &on_nal);

    void cleanup();
    bool ready() const { return ctx != nullptr; }

private:
    // flush() waits for the encoder to emit its buffered frames. Per-call timeout keeps
    // encode_get_packet returning so the overall deadline can bound a wedged encoder.
    static constexpr int FLUSH_PACKET_TIMEOUT_MS = 200;
    static constexpr int FLUSH_DEADLINE_MS       = 3000;

    MppCtx    ctx = nullptr;
    MppApi   *mpi = nullptr;
    MppEncCfg cfg = nullptr;
    int cfg_hor_stride = 0;
    int cfg_ver_stride = 0;
};

#endif
