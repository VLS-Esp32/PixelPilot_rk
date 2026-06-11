/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * Copyright (C) 2025 Dmytro S <dmytriysemenchuk@gmail.com>
*/

#include "rtp_codec_detector.hpp"

#include <iostream>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>
#include "spdlog/spdlog.h"

#include "rtp-demuxer.h"
#include "common.h"

extern int signal_flag;

static VideoCodec detect_rtp_codec(const uint8_t* payload, int payload_len)
{
    if (!payload || payload_len < 64)
        return VideoCodec::UNKNOWN;

    uint8_t b0 = payload[0];
    uint8_t b1 = payload[1];

    bool h264_found = false;
    bool h265_found = false;

    // ================= H.264 (RFC 6184, RTP payload) =================
    //
    // b0: F(1) | NRI(2) | Type(5)
    //
    uint8_t h264_f = (b0 >> 7) & 0x01;
    uint8_t h264_type = b0 & 0x1F;

    if (h264_f == 0) {
        //   1  - non-IDR slice
        //   5  - IDR slice
        //   7  - SPS
        //   8  - PPS
        if (h264_type == 1 || h264_type == 5 || h264_type == 7 || h264_type == 8) {
            h264_found = true;
        }

        // --- Fragmentation units: FU-A / FU-B (types 28/29) ---
        // RTP payload:
        //   b0 = FU indicator (F|NRI|Type=28/29)
        //   b1 = FU header: S(1) | E(1) | R(1) | Type(5)
        //
        if ((h264_type == 28 || h264_type == 29) && payload_len >= 2) {
            uint8_t fu_hdr = b1;
            uint8_t fu_s = (fu_hdr >> 7) & 0x01;
            uint8_t fu_e = (fu_hdr >> 6) & 0x01;
            uint8_t fu_type = fu_hdr & 0x1F;

            if (fu_s && !fu_e && (fu_type == 1 || fu_type == 5)) {
                h264_found = true;
            }
        }
    }

    // ================= H.265 (RFC 7798, RTP payload) =================
    //
    // PayloadHdr:
    //   b0: F(1) | Type(6) | LayerId_high(1)
    //   b1: LayerId_low(5) | TID(3)
    //
    uint8_t h265_f = (b0 >> 7) & 0x01;
    uint8_t h265_type = (b0 >> 1) & 0x3F;  // 6-bit Type
    uint8_t h265_tid_p = b1 & 0x07;  // TID+1 (1..7)

    if (h265_f == 0 && h265_tid_p != 0) {
        // --- Single NAL unit packet ---
        //   19, 20 - IDR slices
        //   32     - VPS
        //   33     - SPS
        //   34     - PPS
        if (h265_type == 19 || h265_type == 20 || h265_type == 32 || h265_type == 33 || h265_type == 34) {
            h265_found = true;
        }

        // --- Fragmentation Unit (FU), Type=49 ---
        if (h265_type == 49 && payload_len >= 3) {
            uint8_t fu_hdr = payload[2];
            uint8_t fu_s = (fu_hdr >> 7) & 0x01;
            uint8_t fu_e = (fu_hdr >> 6) & 0x01;
            uint8_t fu_type = fu_hdr & 0x3F; // 6-bit FuType

            if (fu_s && !fu_e && (fu_type == 19 || fu_type == 20)) {
                h265_found = true;
            }
        }
    }

    if (h264_found && !h265_found)
        return VideoCodec::H264;
    if (h265_found && !h264_found)
        return VideoCodec::H265;

    return VideoCodec::UNKNOWN;
}

// callback for codec detection, called until the codec is found
static int detect_codec_cb(void* param, const void* packet, int bytes, uint32_t timestamp, int flags)
{
    VideoCodec* codec_ptr = (VideoCodec*)param;
    VideoCodec codec = detect_rtp_codec(static_cast<const uint8_t*>(packet), bytes); // need cast here
    if (codec != VideoCodec::UNKNOWN) {
        *codec_ptr = codec;
        return 1; // Stop demuxing after detection
    }
    return 0; // Continue demuxing
}

RtpCodecDecoder::RtpCodecDecoder(std::shared_ptr<std::atomic<bool>> running) :
    m_running(std::move(running))
{}

VideoCodec RtpCodecDecoder::detect_codec(int socket_fd)
{
    // RTP demuxer for detection
    uint8_t buffer[1600];
    struct pollfd fds[] = { { .fd = socket_fd, .events = POLLIN } };
    VideoCodec detected_codec = VideoCodec::UNKNOWN;

    rtp_demuxer_t* demuxer = rtp_demuxer_create(
        100, 90000, 0, NULL, detect_codec_cb, &detected_codec);

    if (!demuxer) {
        spdlog::error("[ RtpCodecDecoder ] Failed to create RTP demuxer for codec detection");
        return VideoCodec::UNKNOWN;
    }

    while (m_running->load() && !signal_flag && detected_codec == VideoCodec::UNKNOWN) {
        int ret = poll(fds, 1, 5000);
        if (ret < 0) { if (errno == EINTR) continue; perror("poll"); break; }
        if (ret == 0) continue;
        if (fds[0].revents & POLLIN) {
            struct sockaddr_in peer; socklen_t len = sizeof(peer);
            ssize_t n = recvfrom(socket_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)&peer, &len);
            if (n > 0) rtp_demuxer_input(demuxer, buffer, (int)n);
        }
    }

    rtp_demuxer_destroy(&demuxer);

    if (detected_codec == VideoCodec::UNKNOWN) {
        spdlog::error("[ RtpCodecDecoder ] Failed to detect codec from RTP stream!");
        return VideoCodec::UNKNOWN;
    }

    spdlog::info("[ RtpCodecDecoder ] Codec detected: {}", codec_type_name(detected_codec));
    return detected_codec;
}

VideoCodec RtpCodecDecoder::get_detected_codec()
{
    spdlog::info("RtpCodecDecoder::get_detected_codec: {}", codec_type_name(detected_codec));
    return detected_codec;
}
