#include "server/UDPBridge.h"

UDPBridge::UDPBridge()
{
}

UDPBridge::UDPBridge(int udp_port_rx, int udp_port_tx, std::string udp_ip_tx, std::queue<std::pair<void *, size_t>> *incoming, std::mutex *incoming_mutex)
{
    udp_port_rx_ = udp_port_rx;
    udp_ip_tx_ = udp_ip_tx;
    udp_port_tx_ = udp_port_tx;
    buffer_size_ = 1048;
    incoming_ = incoming;
    incoming_mutex_ = incoming_mutex;
}

UDPBridge::UDPBridge(int udp_port_rx, int udp_port_tx, std::string udp_ip_tx, std::queue<std::pair<void *, size_t>> *incoming, std::mutex *incoming_mutex, int buffer_size)
{
    udp_port_rx_ = udp_port_rx;
    udp_ip_tx_ = udp_ip_tx;
    udp_port_tx_ = udp_port_tx;
    buffer_size_ = buffer_size;
}

UDPBridge::~UDPBridge()
{
    this->stop();
}

void UDPBridge::start()
{
    listening = true;
    listener_thread_ = std::thread([this]()
                                   { this->listen(); });
}

void UDPBridge::listen()
{
    struct sockaddr_in server_addr{}, client_addr{};
    socklen_t addr_len = sizeof(client_addr);
    char buffer[buffer_size_];

    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0)
    {
        perror("socket");
        return;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(udp_port_rx_);

    if (bind(sockfd_, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        perror("bind");
        close(sockfd_);
        return;
    }

    std::cout << "Listening on UDP port " << udp_port_rx_ << " ..." << std::endl;

    while (listening)
    {
        ssize_t n = recvfrom(sockfd_, buffer, sizeof(buffer), 0,
                             (struct sockaddr *)&client_addr, &addr_len);
        if (n > 0)
        {
            // Creating a copy of the message
            char *msg_cpy = new char[sizeof(buffer)];

            // Coping message
            memcpy(msg_cpy, buffer, sizeof(buffer));

            // Pushing new data on incoming messages queue
            incoming_mutex_->lock();
            incoming_->push(std::make_pair((void *)msg_cpy, sizeof(buffer)));
            incoming_mutex_->unlock();
        }
    }

    close(sockfd_);
}

void UDPBridge::stop()
{
    listening = false;
}

bool UDPBridge::send(std::pair<void *, size_t> msg_tx)
{
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0); // UDP socket
    if (sockfd < 0)
    {
        perror("socket");
        return 0;
    }

    struct sockaddr_in dest_addr{};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(udp_port_tx_);                  // target UDP port
    dest_addr.sin_addr.s_addr = inet_addr(udp_ip_tx_.c_str()); // target IP

    ssize_t sent = sendto(sockfd, msg_tx.first, msg_tx.second, 0,
                          (struct sockaddr *)&dest_addr, sizeof(dest_addr));

    if (sent < 0)
    {
        perror("sendto");

        close(sockfd);
        return 0;
    }
    else
    {
        std::cout << "Sent " << sent << " bytes" << std::endl;
        close(sockfd);
        return 1;
    }
}
