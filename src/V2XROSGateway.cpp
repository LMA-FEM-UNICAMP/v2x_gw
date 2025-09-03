//
// Created by Christoph Pilz
// Updated by Gabriel Toffanetto
//
// Description:
// This is the main file for the ROS2 node
// - it handles the server connection for rx/tx of V2X messages
// - it handles the execution of Handler methods for the single V2X messages
//
// Author(s): "Christoph Pilz, Markus Schratter"
// Copyright: "Copyright 2023, vehicleCAPTAIN toolbox"
// Credits: ["Christoph Pilz", "Markus Schratter"]
// License: "BSD-3-clause"
// Version: "1.0.0"
// Maintainer: "Gabriel Toffanetto"
// E-Mail: "gabriel.rocha@ieee.org"
// Status = "Production"
//

#include "V2XROSGateway.h"

/*
 * TODO
 * [] Read Only for Raspi
 */

using namespace std;

V2XROSGateway::V2XROSGateway() : Node("v2x_gw")
{
    is_node_initialized_ = false;

    // read configuration
    if (!readConfig())
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid configuration. Shutdown the node.");
        rclcpp::shutdown();
    }

    // init handlers
    v2x_m_handler_[MsgType::kCAM] = new CAMHandler(this);
    v2x_m_handler_[MsgType::kCPM] = new CPMHandler(this);
    v2x_m_handler_[MsgType::kDENM] = new DENMHandler(this);
    v2x_m_handler_[MsgType::kIVIM] = new IVIMHandler(this);
    v2x_m_handler_[MsgType::kNone] = new ExampleMHandler(this); // keep it, it is used for decoding

    // init server
    v2x_server_ = new V2XUDPServer(this, v2x_m_handler_);

    // create publishers
    diagnostics_pub_ =
        this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics/vilma_v2x", 1);

    // create subscribers

    // create timers
    timer_ = rclcpp::create_timer(this, this->get_clock(), 10ms, std::bind(&V2XROSGateway::process, this));

    is_node_initialized_ = true;
}

V2XROSGateway::~V2XROSGateway()
{
    // delete in reverse order

    // delete server
    delete v2x_server_;

    // delete message handler
    for (auto elem : v2x_m_handler_)
    {
        delete elem.second;
    }
}

bool V2XROSGateway::readConfig()
{
    // Config arguments

    // RCLCPP_INFO(this->get_logger(), "Center_line: %s and racing_line: %s", m_center_line_file.c_str(), m_racing_line_file.c_str());

    return true;
}

void V2XROSGateway::process(void)
{
    rclcpp::Time current_timestamp = this->get_clock()->now();
    rclcpp::Clock ros_clock(RCL_ROS_TIME);

    static unsigned int alive = 0;
    static rclcpp::Time prev_process_timestamp = current_timestamp;

    // prepare diagnostics_array
    diagnostics_.header.stamp = current_timestamp;
    diagnostics_.header.frame_id = "";
    diagnostics_.status.clear();

    diagnostic_msgs::msg::DiagnosticStatus diagnostic_status;
    diagnostic_status.name = "v2x_sender";
    diagnostic_status.hardware_id = "vilma_V2x";

    // update status
    diagnostic_status.level = diagnostic_status.OK;
    diagnostic_status.message = "OK";

    // get update time
    diagnostic_msgs::msg::KeyValue key_value;
    key_value.key = "Update Time";
    key_value.value = std::to_string(getTimeDifferenceSeconds(current_timestamp, prev_process_timestamp));
    diagnostic_status.values.push_back(key_value);

    // V2X handler diagnostics
    for (auto const &handler : v2x_m_handler_)
    {
        if (handler.first != MsgType::kNone)
        {
            std::vector<diagnostic_msgs::msg::KeyValue> handler_diagnostics = handler.second->GetDiagnostics();
            diagnostic_status.values.insert(std::end(diagnostic_status.values), std::begin(handler_diagnostics), std::end(handler_diagnostics));
        }
    }

    // V2X server diagnostics
    std::vector<diagnostic_msgs::msg::KeyValue> server_values = v2x_server_->GetDiagnostics();
    diagnostic_status.values.insert(std::end(diagnostic_status.values), std::begin(server_values), std::end(server_values));

    diagnostics_.status.push_back(diagnostic_status);

    // send diagnostics_array with 10Hz or if an error is present
    if (alive % 10 == 0 || (diagnostics_.status.size() > 0 && diagnostics_.status.at(0).level > 0))
    {
        diagnostics_pub_->publish(diagnostics_);
    }

    prev_process_timestamp = current_timestamp;
    alive++;
}

double V2XROSGateway::getTimeDifferenceSeconds(rclcpp::Time A, rclcpp::Time B)
{
    return (abs((A - B).seconds()));
}

// Main
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<V2XROSGateway>());

    rclcpp::shutdown();
    return 0;
}
