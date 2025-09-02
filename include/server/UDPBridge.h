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

    void listen();
    void stop();
    void start();
    void callback();
    bool send(std::pair<void *, size_t> msg_tx);

private:
    int udp_port_rx_;
    int udp_port_tx_;
    std::string udp_ip_tx_;
    int sockfd_;
    int buffer_size_;

    bool listening;
    std::thread listener_thread_;

    std::queue<std::pair<void *, size_t>> *incoming_;

    std::mutex *incoming_mutex_;
};

#endif // UDP_Bridge_HPP_
