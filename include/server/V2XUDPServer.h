#ifndef V2X_UDP_SERVER_H
#define V2X_UDP_SERVER_H

#include <queue>
#include <string>
#include <utility>
#include <mutex>
#include <memory>

#include "server/V2XServer.h"
#include "server/UDPBridge.h"

class V2XUDPServer : public V2XServer
{
public:
    /// Sets up and connects to the server
    /// \param gateway_node reference to the ros2 gateway_node
    /// \param v2x_m_handler reference to the message handlers
    V2XUDPServer(rclcpp::Node *gateway_node, std::map<MsgType, V2XMHandler *> v2x_m_handler);

    /// Disconnects from the server
    ~V2XUDPServer();

    /// Checks if the server is still alive
    /// \return true, if alive; false, otherwise
    bool IsAlive() override;

    /// Receive server diagnostics
    /// \return server diagnostic key value pairs
    virtual std::vector<diagnostic_msgs::msg::KeyValue> GetDiagnostics() override;

    /// Sends V2X messages to the server
    /// \param msgs a queue of bytestream messages to send
    void SendMessages(std::queue<std::pair<void *, size_t>> msgs) override;

protected:
    /// Read the configuration for the ZMQ server
    void ReadConfig() override;

private:
    std::unique_ptr<UDPBridge> udp_bridge_;

    int udp_port_rx_;
    int udp_port_tx_;
    std::string udp_ip_tx_;

    // necessary CAM constants
    std::string SERVER_TYPE;
    std::string SERVER_ADDRESS;
    std::string SERVER_TOPIC_FILTER_HB;
    std::string SERVER_TOPIC_FILTER_V2X;
    long SERVER_RECEIVE_PORT;
    long SERVER_SEND_PORT;
    long SERVER_CYCLE_TIME_MS;
};

#endif // V2X_UDP_SERVER_H