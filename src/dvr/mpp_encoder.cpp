#include <cstdint>
#include <chrono>
#include <thread>

#include <rockchip/rk_mpi.h>

#include "spdlog/spdlog.h"

#include "mpp_encoder.h"

// Backstop only, and a frame COUNT (rc:gop = fps * this), so its real-time interval drifts with the
// capture rate. The DVR's KEYFRAME_INTERVAL_90K in dvr.cpp is the active clock and always fires
// first; this just guarantees keyframes if request_idr() ever stops working.
static const int GOP_SECONDS = 2;

bool MppEncoder::init(int width, int height, int hor_stride, int ver_stride,
                      int fps, int bitrate) {
    cfg_hor_stride = hor_stride;
    cfg_ver_stride = ver_stride;

    int ret = mpp_create(&ctx, &mpi);
    if (ret) {
        spdlog::error("[ DVR MppEncoder ] mpp_create failed {}", ret);
        return false;
    }

    ret = mpp_init(ctx, MPP_CTX_ENC, MPP_VIDEO_CodingHEVC);
    if (ret) {
        spdlog::error("[ DVR MppEncoder ] mpp_init failed {}", ret);
        cleanup();
        return false;
    }

    ret = mpp_enc_cfg_init(&cfg);
    if (ret) {
        spdlog::error("[ DVR MppEncoder ] mpp_enc_cfg_init failed {}", ret);
        cleanup();
        return false;
    }

    ret = mpi->control(ctx, MPP_ENC_GET_CFG, cfg);
    if (ret) {
        spdlog::error("[ DVR MppEncoder ] MPP_ENC_GET_CFG failed {}", ret);
        cleanup();
        return false;
    }

    // prep: input frame geometry
    mpp_enc_cfg_set_s32(cfg, "prep:width",       width);
    mpp_enc_cfg_set_s32(cfg, "prep:height",      height);
    mpp_enc_cfg_set_s32(cfg, "prep:hor_stride",  hor_stride);
    mpp_enc_cfg_set_s32(cfg, "prep:ver_stride",  ver_stride);
    mpp_enc_cfg_set_s32(cfg, "prep:format",      MPP_FMT_YUV420SP);

    // rc: rate control (VBR around the target bitrate)
    mpp_enc_cfg_set_s32(cfg, "rc:mode",          MPP_ENC_RC_MODE_VBR);
    mpp_enc_cfg_set_s32(cfg, "rc:bps_target",    bitrate);
    mpp_enc_cfg_set_s32(cfg, "rc:bps_max",       bitrate * 3 / 2);
    mpp_enc_cfg_set_s32(cfg, "rc:bps_min",       bitrate / 2);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_in_num",    fps);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_in_denorm", 1);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_out_num",   fps);
    mpp_enc_cfg_set_s32(cfg, "rc:fps_out_denorm",1);
    mpp_enc_cfg_set_s32(cfg, "rc:gop",           fps * GOP_SECONDS);

    mpp_enc_cfg_set_s32(cfg, "h265:profile", 1);   // main
    mpp_enc_cfg_set_s32(cfg, "h265:level",   120);

    ret = mpi->control(ctx, MPP_ENC_SET_CFG, cfg);
    if (ret) {
        spdlog::error("[ DVR MppEncoder ] MPP_ENC_SET_CFG failed {}", ret);
        cleanup();
        return false;
    }

    {
        // Headers (VPS/SPS/PPS) are prepended to each IDR frame, so every output packet
        // is one self-contained frame (matters for the real-PTS FIFO in Dvr).
        MppEncHeaderMode header_mode = MPP_ENC_HEADER_MODE_EACH_IDR;
        ret = mpi->control(ctx, MPP_ENC_SET_HEADER_MODE, &header_mode);
        if (ret) {
            spdlog::error("[ DVR MppEncoder ] MPP_ENC_SET_HEADER_MODE failed {}", ret);
            cleanup();
            return false;
        }
    }

    {
        // Non-blocking output - encode_get_packet returns immediately when no packet is
        // ready. Without this the drain loop blocks forever after the first packet.
        RK_S64 timeout = 0;
        ret = mpi->control(ctx, MPP_SET_OUTPUT_TIMEOUT, &timeout);
        if (ret) {
            spdlog::error("[ DVR MppEncoder ] MPP_SET_OUTPUT_TIMEOUT failed {}", ret);
            cleanup();
            return false;
        }
    }

    return true;
}

void MppEncoder::sync_strides(int hor_stride, int ver_stride) {
    cfg_hor_stride = hor_stride;
    cfg_ver_stride = ver_stride;
    mpp_enc_cfg_set_s32(cfg, "prep:hor_stride", hor_stride);
    mpp_enc_cfg_set_s32(cfg, "prep:ver_stride", ver_stride);
    mpi->control(ctx, MPP_ENC_SET_CFG, cfg);
    spdlog::info("[ DVR MppEncoder ] zero-copy strides synced to {}x{}", hor_stride, ver_stride);
}

void MppEncoder::request_idr() {
    if (!ctx) {
        return;
    }
    mpi->control(ctx, MPP_ENC_SET_IDR_FRAME, nullptr);
}

int MppEncoder::submit(MppBuffer buf, int64_t pts, int width, int height,
                       int hor_stride, int ver_stride) {
    MppFrame frame = nullptr;
    mpp_frame_init(&frame);
    mpp_frame_set_width(frame,      width);
    mpp_frame_set_height(frame,     height);
    mpp_frame_set_hor_stride(frame, hor_stride);
    mpp_frame_set_ver_stride(frame, ver_stride);
    mpp_frame_set_fmt(frame,        MPP_FMT_YUV420SP);
    mpp_frame_set_pts(frame,        (RK_S64)pts);
    mpp_frame_set_buffer(frame,     buf);

    int ret = mpi->encode_put_frame(ctx, frame);
    mpp_frame_deinit(&frame);
    if (ret) {
        spdlog::warn("[ DVR MppEncoder ] encode_put_frame failed {}", ret);
    }
    return ret;
}

void MppEncoder::drain(const std::function<void(const uint8_t *, int)> &on_nal) {
    // Collect all currently-available encoded packets. Called at the start of each
    // encode so the previous submissions output (which has had a full frame interval
    // to complete) is written before we reuse an input buffer.
    MppPacket packet = nullptr;
    while (true) {
        int ret = mpi->encode_get_packet(ctx, &packet);
        if (ret == MPP_ERR_TIMEOUT || !packet) {
            break;
        }
        if (ret) {
            spdlog::warn("[ DVR MppEncoder ] encode_get_packet failed {}", ret);
            break;
        }
        const uint8_t *data = (const uint8_t *)mpp_packet_get_pos(packet);
        int len = (int)mpp_packet_get_length(packet);
        if (len > 0) {
            on_nal(data, len);
        }
        mpp_packet_deinit(&packet);
        packet = nullptr;
    }
}

void MppEncoder::flush(const std::function<bool(const uint8_t *, int)> &on_nal) {
    // Send an EOS frame to flush internally buffered frames.
    MppFrame eos_frame = nullptr;
    mpp_frame_init(&eos_frame);
    mpp_frame_set_eos(eos_frame, 1);
    mpi->encode_put_frame(ctx, eos_frame);
    mpp_frame_deinit(&eos_frame);

    // Wait for each packet instead of returning immediately: with timeout=0 the loop would exit
    // before the encoder finishes flushing, leaving the following reset() to block for 8+ seconds.
    // A finite per-call timeout (rather than -1) keeps every call returning, so the overall deadline
    // below can bound the flush if the encoder wedges - otherwise shutdown hangs until SIGKILL and
    // the recording loses its moov.
    RK_S64 block = FLUSH_PACKET_TIMEOUT_MS;
    mpi->control(ctx, MPP_SET_OUTPUT_TIMEOUT, &block);

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(FLUSH_DEADLINE_MS);
    MppPacket packet = nullptr;
    while (true) {
        if (std::chrono::steady_clock::now() >= deadline) {
            spdlog::warn("[ DVR MppEncoder ] flush did not complete within {}ms, giving up",
                         FLUSH_DEADLINE_MS);
            break;
        }
        int ret = mpi->encode_get_packet(ctx, &packet);
        if (ret == MPP_ERR_TIMEOUT || (!ret && !packet)) {
            // Nothing ready yet - keep waiting until the deadline. The short sleep bounds CPU use in
            // case the timeout control above did not take and this call returns immediately.
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        if (ret) {
            spdlog::warn("[ DVR MppEncoder ] encode_get_packet failed during flush {}", ret);
            break;
        }
        bool is_eos = mpp_packet_get_eos(packet);
        int  len    = (int)mpp_packet_get_length(packet);
        bool stop   = false;
        if (len > 0) {
            const uint8_t *data = (const uint8_t *)mpp_packet_get_pos(packet);
            if (!on_nal(data, len)) {  // callback declined (frame cap reached)
                stop = true;
            }
        }
        mpp_packet_deinit(&packet);
        packet = nullptr;
        if (is_eos || stop) {
            break;
        }
    }
}

void MppEncoder::cleanup() {
    if (cfg) {
        mpp_enc_cfg_deinit(cfg);
        cfg = nullptr;
    }
    if (ctx) {
        mpi->reset(ctx);
        mpp_destroy(ctx);
        ctx = nullptr;
        mpi = nullptr;
    }
    cfg_hor_stride = 0;
    cfg_ver_stride = 0;
}
