
#include "server/V2XUDPServer.h"

V2XUDPServer::V2XUDPServer(rclcpp::Node *gateway_node, std::map<MsgType, V2XMHandler *> v2x_m_handler)
    : V2XServer(gateway_node, v2x_m_handler)
{

    // configure
    ReadConfig();

    udp_bridge_ = std::make_unique<UDPBridge>(udp_port_tx_, udp_port_rx_, udp_ip_tx_, &incoming_, &incoming_mutex_, 2048);

    // create timers
    timer_ = rclcpp::create_timer(GetNode(), GetNode()->get_clock(),
                                  rclcpp::Duration::from_nanoseconds(RCL_MS_TO_NS(SERVER_CYCLE_TIME_MS)),
                                  std::bind(&V2XUDPServer::Process, this));
}

V2XUDPServer::~V2XUDPServer()
{
    udp_bridge_->stop();
}

void V2XUDPServer::SendMessages(std::queue<std::pair<void *, size_t>> msgs)
{
    // variables
    std::pair<void *, size_t> msg_tx;

    while (!msgs.empty())
    {
        // create v2x_msg
        msg_tx = msgs.front();

        // free memory of msg
        delete ((char *)msgs.front().first);

        // pop msg from queue
        msgs.pop();

        if (udp_bridge_->send(msg_tx))
        {
            // update counter
            client_sent_messages_++;
            client_last_message_sent_ = GetNode()->get_clock()->now();
        }
        else
        {
            RCLCPP_WARN(GetNode()->get_logger(), "Failed to sent message");
        }
    }
}

// --- --- --- protected methods --- --- ---

void V2XUDPServer::ReadConfig()
{

    GetNode()->declare_parameter("rx_udp_port", 2000);
    GetNode()->get_parameter("rx_udp_port", udp_port_rx_);
    GetNode()->declare_parameter("tx_udp_port", 2000);
    GetNode()->get_parameter("tx_udp_port", udp_port_tx_);
    GetNode()->declare_parameter("udp_ip_tx_", "127.0.0.1");
    GetNode()->get_parameter("udp_ip_tx_", udp_ip_tx_);

    GetNode()->declare_parameter("server.cycle_time_ms", 100);
    GetNode()->get_parameter("server.cycle_time_ms", SERVER_CYCLE_TIME_MS);
}

bool V2XUDPServer::IsAlive()
{
    RCLCPP_ERROR(GetNode()->get_logger(), "V2XUDPServer::IsAlive() not yet implemented");
    throw "NotImplemented";

    return false;
}

std::vector<diagnostic_msgs::msg::KeyValue> V2XUDPServer::GetDiagnostics()
{
    std::vector<diagnostic_msgs::msg::KeyValue> values;
    diagnostic_msgs::msg::KeyValue key_value;

    // status - general
    key_value.key = "v2x_server.is_active";
    key_value.value = std::to_string(is_active_);
    values.push_back(key_value);
    key_value.key = "v2x_server.is_configured_";
    key_value.value = std::to_string(is_configured_);
    values.push_back(key_value);
    key_value.key = "v2x_server.is_connected_";
    key_value.value = std::to_string(is_connected_);
    values.push_back(key_value);

    // status - client
    key_value.key = "v2x_server.client_received_messages_";
    key_value.value = std::to_string(client_received_messages_);
    values.push_back(key_value);
    key_value.key = "v2x_server.client_sent_messages_";
    key_value.value = std::to_string(client_sent_messages_);
    values.push_back(key_value);
    key_value.key = "v2x_server.client_last_message_received_";
    key_value.value = std::to_string(client_last_message_received_.nanoseconds());
    values.push_back(key_value);
    key_value.key = "v2x_server.client_last_message_sent_";
    key_value.value = std::to_string(client_last_message_sent_.nanoseconds());
    values.push_back(key_value);

    // status - server
    key_value.key = "v2x_server.server_connection_data_";
    key_value.value = server_connection_data_;
    values.push_back(key_value);
    key_value.key = "v2x_server.server_heartbeat_counter_";
    key_value.value = std::to_string(server_heartbeat_counter_);
    values.push_back(key_value);
    key_value.key = "v2x_server.server_heartbeat_message_";
    key_value.value = server_heartbeat_message_;
    values.push_back(key_value);
    key_value.key = "v2x_server.server_received_messages_";
    key_value.value = std::to_string(server_received_messages_);
    values.push_back(key_value);
    key_value.key = "v2x_server.server_sent_messages_";
    key_value.value = std::to_string(server_sent_messages_);
    values.push_back(key_value);
    key_value.key = "v2x_server.server_last_message_received_";
    key_value.value = std::to_string(server_last_message_received_.nanoseconds());
    values.push_back(key_value);
    key_value.key = "v2x_server.server_last_message_sent_";
    key_value.value = std::to_string(server_last_message_sent_.nanoseconds());
    values.push_back(key_value);

    return values;
}