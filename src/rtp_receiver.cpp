/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * Copyright (C) 2025 Dmytro S <dmytriysemenchuk@gmail.com>
 */

#include "rtp_receiver.hpp"

#include "spdlog/spdlog.h"
#include <cstring>
#include <chrono>
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
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>

#include <thread>

#include "rtp-demuxer.h"
#include "rtp-profile.h"

#include "wfbcli.hpp"

extern "C" {
#include "osd.h"
}

extern std::atomic<bool> drone_connected;

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
    static auto prev_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    // check difference between packets more then 30 sec if true check if new codec is present
    if (std::chrono::duration_cast<std::chrono::seconds>(current_time - prev_time).count() > 30) {
        spdlog::info("[ RTP ] Time gap between packets more than 30 seconds. Checking new codec...");
        if (rtp_receiver->new_codec_detected()) {
            rtp_receiver->m_cb((void*)packet, bytes, true);
        }
    }
    else {
        rtp_receiver->m_cb((void*)packet, bytes, false);
    }
    prev_time = current_time;

    return 0;
}

RtpReceiver::RtpReceiver(int port)
{
    m_running = std::make_shared<std::atomic<bool>>(false);
    m_socket_handler = std::make_shared<SocketHandler>(port);
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
    auto current_time = std::chrono::steady_clock::now();
    auto prev_time = current_time;
    // packet reception loop
    while (this->m_running->load()) {
        int ret = poll(fds, 1, 1000);
        if (ret < 0) { if (errno == EINTR) continue; perror("poll"); break; }
        if (ret == 0) {
            current_time = std::chrono::steady_clock::now();

            if (std::chrono::duration_cast<std::chrono::seconds>(current_time - prev_time).count() > 15) {
                spdlog::info("[ RTP ] No packets received during 15 sec");
                if (drone_connected.load())
                {
                    drone_connected.store(false);
                    void *batch = osd_batch_init(4);

                    osd_add_clear_fact(batch, "video.displayed_frame", nullptr, 0);
                    osd_add_clear_fact(batch, "rtp.received_bytes", nullptr, 0);
                    osd_add_clear_fact(batch, "video.width", nullptr, 0);
                    osd_add_clear_fact(batch, "video.height", nullptr, 0);

                    osd_publish_batch(batch);
                    clear_osd_wfbcli();
                }
                prev_time = current_time;
            }
            continue;
        }
        if (fds[0].revents & POLLIN) {
            struct sockaddr_in peer; socklen_t len = sizeof(peer);
            ssize_t n = recvfrom(m_socket, buffer, sizeof(buffer), 0, (struct sockaddr*)&peer, &len);
            if (n > 0) {
                rtp_demuxer_input(demuxer, buffer, (int)n);
                if (!drone_connected.load())
                {
                    drone_connected.store(true);
                }
                prev_time = std::chrono::steady_clock::now();
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


bool RtpReceiver::new_codec_detected()
{
    VideoCodec new_codec = m_codec_detector->detect_codec(m_socket_handler->get_socket_fd());
    if (new_codec != VideoCodec::UNKNOWN && m_video_codec != new_codec) {
        m_video_codec = new_codec;
        spdlog::info("[ RTP ] New codec detected: {}", codec_type_name(m_video_codec));
        return true;
    }
    return false;
}
