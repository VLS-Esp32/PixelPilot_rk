/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * Copyright (C) 2025 Dmytro S <dmytriysemenchuk@gmail.com>
 */

#ifndef SOCKET_HANDLER_H
#define SOCKET_HANDLER_H

#include <string>

class SocketHandler {
public:
    explicit SocketHandler(const std::string& address, int port);
    explicit SocketHandler(const char *sock);
    ~SocketHandler();

    bool init_local_socket();
    bool init_internet_socket();
    bool init_connection();
    bool is_socket_connected();
    int get_socket_fd();
private:
    int m_socket;
    int m_port;
    
    std::string m_ip;
    std::string m_unix_socket;

    bool socket_connected = false;
};

#endif // SOCKET_HANDLER_H

