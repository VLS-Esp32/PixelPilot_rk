/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * Copyright (C) 2025 Dmytro S <dmytriysemenchuk@gmail.com>
 */

#include "socket_handler.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "spdlog/spdlog.h"


SocketHandler::SocketHandler(const std::string& address, int port): m_port{port}, m_ip{address}
{

}

SocketHandler::SocketHandler(const char *unix_socket) : m_unix_socket{unix_socket}
{

}

SocketHandler::~SocketHandler()
{
    close(m_socket);
}

bool SocketHandler::init_local_socket()
{
    m_socket = socket(AF_UNIX, SOCK_DGRAM, 0);

    if (m_socket < 0) {
        perror("socket");
        return false;
    }

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;

    // Abstract socket: Start sun_path with a null byte, then copy the rest.
    // The "@" in logs is a placeholder for the null byte.
    addr.sun_path[0] = '\0';  // First byte is null
    strncpy(addr.sun_path + 1, m_unix_socket.c_str(), sizeof(addr.sun_path) - 2);  // Leave room for null
    addr.sun_path[sizeof(addr.sun_path) - 1] = '\0';  // Ensure null-terminated

    // Length = sizeof(sun_family) + 1 (null byte) + strlen(path)
    socklen_t addr_len = sizeof(addr.sun_family) + 1 + strlen(m_unix_socket.c_str());

    if (bind(m_socket, (struct sockaddr*)&addr, addr_len) < 0) {
        perror("bind");
        close(m_socket);
        return false;
    }

    spdlog::info("[ SocketHandler ] Bound successfully to unix socket: @{}", m_unix_socket.c_str());
    return true;
}

bool SocketHandler::init_internet_socket()
{
    m_socket = socket(AF_INET, SOCK_DGRAM, 0);

    if (m_socket < 0) {
        perror("socket");
        return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    // addr.sin_addr.s_addr = INADDR_ANY; // TODO: should we use this ?
    addr.sin_port = htons(m_port);

    if (inet_pton(AF_INET, m_ip.c_str(), &addr.sin_addr) != 1) {
        spdlog::error("[ SocketHandler ] Invalid IP: {}", m_ip);
        close(m_socket);
        return false;
    }
    if (bind(m_socket, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(m_socket);
        return false;
    }

    // TODO: if we use specific ip as in "inet_aton"
    spdlog::info("[ SocketHandler ] Listening on {}:{}", m_ip.c_str(), m_port);
    // TODO: if we use INADDR_ANY -> "0.0.0.0"
    // spdlog::info("[ SocketHandler ] Listening on 0.0.0.0:{}", m_port);

    return true;
}

bool SocketHandler::init_connection()
{
    if (socket_connected)
    {
        spdlog::info("[ SocketHandler ] Connection is already open");
        return true;
    }

    if (m_unix_socket.empty()) {
        socket_connected = init_internet_socket();
    }
    else {
        socket_connected = init_local_socket();
    }

    if (!socket_connected) {
        spdlog::error("[ SocketHandler ] Failed to initialize connection");
    }

    return socket_connected;
}

bool SocketHandler::is_socket_connected()
{
    return socket_connected;
}

int SocketHandler::get_socket_fd()
{
    return m_socket;
}