/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * Copyright (C) 2025 Dmytro S <dmytriysemenchuk@gmail.com>
 */

#ifndef RTP_CODEC_DETECTOR_H
#define RTP_CODEC_DETECTOR_H

#include "common.h"
#include <cstdint>
#include <memory>

VideoCodec rtp_classify_codec(const uint8_t *rtp_packet, int len);


class RtpCodecDecoder {
public:
    explicit RtpCodecDecoder(std::shared_ptr<std::atomic<bool>> running);
    ~RtpCodecDecoder() = default;

    VideoCodec detect_codec(int socket_fd);
    VideoCodec get_detected_codec();
private:
    VideoCodec detected_codec = VideoCodec::UNKNOWN;
    std::shared_ptr<std::atomic<bool>> m_running;
};

#endif // RTP_CODEC_DETECTOR_H