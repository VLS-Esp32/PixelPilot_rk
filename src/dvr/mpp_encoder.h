#ifndef DVR_MPP_ENCODER_H
#define DVR_MPP_ENCODER_H

#include <cstdint>
#include <functional>

#include <rockchip/rk_mpi.h>

class MppEncoder {
public:
    // Configure an H265 encoder for the given input geometry
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
    MppCtx    ctx = nullptr;
    MppApi   *mpi = nullptr;
    MppEncCfg cfg = nullptr;
    int cfg_hor_stride = 0;
    int cfg_ver_stride = 0;
};

#endif
