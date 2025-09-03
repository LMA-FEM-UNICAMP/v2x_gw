//
// Created by Gabriel Toffanetto
// Description:
// Handle with UDP communication with Cohda device to send and fill message queue from server
//
// Author(s): "Gabriel Toffanetto"
// Copyright: "Copyright 2025"
// Credits: ["Christoph Pilz"]
// License: "BSD-3-clause"
// Version: "1.0.0"
// Maintainer: "Gabriel Toffanetto"
// E-Mail: "gabriel.rocha@ieee.org"
// Status = "Production"
//

#ifndef UDP_Bridge_HPP_
#define UDP_Bridge_HPP_

#include <thread>
#include <functional>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <queue>
#include <mutex>
#include <string>

class UDPBridge
{
public:
    UDPBridge();
    UDPBridge(int udp_port_rx, int udp_port_tx, std::string udp_ip_tx, std::queue<std::pair<void *, size_t>> *incoming, std::mutex *incoming_mutex);
    UDPBridge(int udp_port_rx, int udp_port_tx, std::string udp_ip_tx, std::queue<std::pair<void *, size_t>> *incoming, std::mutex *incoming_mutex, int buffer_size);
    ~UDPBridge();

    void configure();
    void listen();
    void stop();
    void start();
    void callback();
    bool send(std::pair<void *, size_t> msg_tx);

private:
    // UDP sender
    int udp_port_tx_;
    std::string udp_ip_tx_;
    int sockfd_tx_;
    struct sockaddr_in dest_addr_;

    // UDP listener
    int udp_port_rx_;
    int sockfd_rx_;
    int buffer_size_;
    char *buffer_rx_;
    struct sockaddr_in server_addr_;
    struct sockaddr_in client_addr_;

    bool listening;
    std::thread listener_thread_;

    std::queue<std::pair<void *, size_t>> *incoming_;

    std::mutex *incoming_mutex_;
};

#endif // UDP_Bridge_HPP_
