/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * Copyright (C) 2025 Dmytro S <dmytriysemenchuk@gmail.com>
 */

#include "rtp_receiver.hpp"

#include "spdlog/spdlog.h"
#include <cstring>
#include <stdexcept>
#include <cassert>
#include <sstream>
#include <iostream>
#include <memory>
#include <utility>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/un.h>
#include <fcntl.h>
#include <pthread.h>
#include <iostream>

extern int signal_flag;

#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>

#include <thread>

#include "rtp-demuxer.h"
#include "rtp-profile.h"

extern "C" {
#include "osd.h"
}

static const int CODEC_MISMATCH_CONFIRM = 5;

// Main callback for packet processing after codec is known
static int main_rtp_cb(void* param, const void* packet, int bytes, uint32_t timestamp, int flags)
{
    RtpReceiver* rtp_receiver = (RtpReceiver *)param;

#if 0 // debug rtp
    spdlog::debug("[RTP] Codec: {}, Packet size: {}, Timestamp: {}, Flags: 0x{:x}\n", codec_type_name(rtp_receiver->get_video_codec()), bytes, timestamp, flags);
    if (bytes == 25) {
        spdlog::debug("SPS :");
        for (int i = 0; i < bytes && i < 25; i++) {
            spdlog::debug("{:02x} ", ((const uint8_t*)packet)[i]);
        }
        spdlog::debug("\n");
    }

    if (bytes == 4) {
        spdlog::debug("PPS :");
        for (int i = 0; i < bytes && i < 4; i++) {
            spdlog::debug("{:02x} ", ((const uint8_t*)packet)[i]);
        }
        spdlog::debug("\n");
    }
#endif
    rtp_receiver->m_cb((void*)packet, bytes, false);

    return 0;
}

RtpReceiver::RtpReceiver(const std::string& address, int port)
{
    m_running = std::make_shared<std::atomic<bool>>(false);
    m_socket_handler = std::make_shared<SocketHandler>(address, port);
    m_codec_detector = std::make_unique<RtpCodecDecoder>(m_running);
}


RtpReceiver::RtpReceiver(const char *unix_socket)
{
    m_running = std::make_shared<std::atomic<bool>>(false);
    m_socket_handler = std::make_shared<SocketHandler>(unix_socket);
    m_codec_detector = std::make_unique<RtpCodecDecoder>(m_running);
}

void RtpReceiver::start_receiving(NEW_FRAME_CALLBACK cb)
{
    m_cb = cb;

    if (m_running->load()) {
        spdlog::info("[ RTP ] Already running RTP receiver thread");
        return;
    }

    if (m_rtp_receiver_thread) {
        m_rtp_receiver_thread.reset();
    }
    m_running->store(true);
    m_rtp_receiver_thread = std::make_unique<std::thread>(&RtpReceiver::rtp_receiver_thread, this);
}

void RtpReceiver::stop_receiving()
{
    m_running->store(false);
    if (m_rtp_receiver_thread->joinable()) {
        m_rtp_receiver_thread->join();
    }
}

void RtpReceiver::init()
{
    if (!m_socket_handler->is_socket_connected() && !m_socket_handler->init_connection()) {
        spdlog::error("[ RTP ] Failed to initialize socket");
        assert(false);
    }
    m_socket = m_socket_handler->get_socket_fd();
    if (m_video_codec == VideoCodec::UNKNOWN)
    {
        m_running->store(true);
        m_video_codec = m_codec_detector->detect_codec(m_socket_handler->get_socket_fd());
        if (m_video_codec == VideoCodec::UNKNOWN) {
            if (signal_flag) {
                spdlog::info("[ RTP ] Codec detection aborted (shutdown requested)");
                m_running->store(false);
                return;
            }
            spdlog::error("[ RTP ] Failed to detect codec");
            assert(false);
        }
        m_running->store(false);
    }
    spdlog::info("[ RTP ] Rtp receiver initialized");
}

void RtpReceiver::rtp_receiver_thread()
{
    std::string codec_name = codec_type_name(this->m_video_codec);
    
    if (codec_name == "UNKNOWN") {
        spdlog::error("[ RTP ] Unsupported codec detected: {}", codec_name);
        return;
    }

    // Create main demuxer for packet processing
    struct rtp_demuxer_t* demuxer = rtp_demuxer_create(
            0, 90000, RTP_PAYLOAD_DYNAMIC,
            codec_name.c_str(),
            main_rtp_cb, this);

    if (!demuxer) {
        spdlog::error("[ RTP ] Failed to create main RTP demuxer");
        return;
    }

    uint8_t buffer[1600];
    struct pollfd fds[] = { { .fd = m_socket, .events = POLLIN } };
    // packet reception loop
    while (this->m_running->load()) {
        int ret = poll(fds, 1, 1000);
        if (ret < 0) { if (errno == EINTR) continue; perror("poll"); break; }
        if (ret == 0) {
            continue;
        }
        if (fds[0].revents & POLLIN) {
            struct sockaddr_in peer; socklen_t len = sizeof(peer);
            ssize_t n = recvfrom(m_socket, buffer, sizeof(buffer), 0, (struct sockaddr*)&peer, &len);
            if (n > 0) {
                VideoCodec packet_codec = rtp_classify_codec(buffer, (int)n);
                if (packet_codec != VideoCodec::UNKNOWN && packet_codec != m_video_codec) {
                    m_codec_mismatch_count++;
                    if (m_codec_mismatch_count >= CODEC_MISMATCH_CONFIRM) {
                        spdlog::info("[ RTP ] Stream codec changed {} -> {}",
                                     codec_type_name(m_video_codec), codec_type_name(packet_codec));
                        m_video_codec = packet_codec;
                        m_codec_mismatch_count = 0;
                        m_cb(nullptr, 0, true);
                        continue;
                    }
                } else if (packet_codec == m_video_codec) {
                    m_codec_mismatch_count = 0;
                }
                rtp_demuxer_input(demuxer, buffer, (int)n);
            }
        }
    }

    rtp_demuxer_destroy(&demuxer);

    spdlog::info("[ RTP ] Exiting RTP receiver thread running: {}", this->m_running->load());

    return;
}

VideoCodec RtpReceiver::get_video_codec()
{
    return m_video_codec;
}
