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

#include "server/UDPBridge.h"

UDPBridge::UDPBridge(int udp_port_rx, int udp_port_tx, std::string udp_ip_tx, std::queue<std::pair<void *, size_t>> *incoming, std::mutex *incoming_mutex)
{
    udp_port_rx_ = udp_port_rx;
    udp_ip_tx_ = udp_ip_tx;
    udp_port_tx_ = udp_port_tx;
    buffer_size_ = 1048;
    incoming_ = incoming;
    incoming_mutex_ = incoming_mutex;
    configure();
}

UDPBridge::UDPBridge(int udp_port_rx, int udp_port_tx, std::string udp_ip_tx, std::queue<std::pair<void *, size_t>> *incoming, std::mutex *incoming_mutex, int buffer_size)
{
    udp_port_rx_ = udp_port_rx;
    udp_ip_tx_ = udp_ip_tx;
    udp_port_tx_ = udp_port_tx;
    buffer_size_ = buffer_size;
    incoming_ = incoming;
    incoming_mutex_ = incoming_mutex;
    configure();
}

void UDPBridge::configure()
{
    // Listener
    buffer_rx_ = new char[buffer_size_];

    char buffer_rx_[buffer_size_];

    sockfd_rx_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_rx_ < 0)
    {
        perror("socket");
        return;
    }

    server_addr_.sin_family = AF_INET;
    server_addr_.sin_addr.s_addr = INADDR_ANY;
    server_addr_.sin_port = htons(udp_port_rx_); // UDP port

    if (bind(sockfd_rx_, (struct sockaddr *)&server_addr_, sizeof(server_addr_)) < 0)
    {
        perror("bind");
        close(sockfd_rx_);
        return;
    }

    // Sender
    sockfd_tx_ = socket(AF_INET, SOCK_DGRAM, 0); // UDP socket

    if (sockfd_tx_ < 0)
    {
        perror("socket");
        close(sockfd_tx_);
        return;
    }

    dest_addr_.sin_family = AF_INET;
    dest_addr_.sin_port = htons(udp_port_tx_);                  // target UDP port
    dest_addr_.sin_addr.s_addr = inet_addr(udp_ip_tx_.c_str()); // target IP
}

UDPBridge::~UDPBridge()
{
    this->stop();

    delete buffer_rx_;

    close(sockfd_rx_);
}

void UDPBridge::start()
{
    listening = true;
    listener_thread_ = std::thread([this]()
                                   { this->listen(); });
}

void UDPBridge::listen()
{
    socklen_t addr_len = sizeof(client_addr_);

    std::cout << "Listening on UDP port " << udp_port_rx_ << " ..." << std::endl;

    while (listening)
    {
        ssize_t n = recvfrom(sockfd_rx_, buffer_rx_, sizeof(buffer_rx_), 0,
                             (struct sockaddr *)&client_addr_, &addr_len);
        if (n > 0)
        {
            // Creating a copy of the message
            char *msg_cpy = new char[sizeof(buffer_rx_)];

            // Coping message
            memcpy(msg_cpy, buffer_rx_, sizeof(buffer_rx_));

            // Pushing new data on incoming messages queue
            incoming_mutex_->lock();
            incoming_->push(std::make_pair((void *)msg_cpy, sizeof(buffer_rx_)));
            incoming_mutex_->unlock();

            // ? Debug
            std::cout << "Data received through UDP port." << std::endl;
        }
    }
}

void UDPBridge::stop()
{
    listening = false;
}

bool UDPBridge::send(std::pair<void *, size_t> msg_tx)
{
    ssize_t sent = sendto(sockfd_tx_, msg_tx.first, msg_tx.second, 0,
                          (struct sockaddr *)&dest_addr_, sizeof(dest_addr_));

    if (sent < 0)
    {
        perror("sendto");
        return 0;
    }
    else
    {
        std::cout << "Sent " << sent << " bytes" << std::endl;
        return 1;
    }
}
