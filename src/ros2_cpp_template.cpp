/*!*******************************************************************************************
 *  \file       ros2_cpp_template.cpp
 *  \brief      Class implementation
 *  \authors    Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include <ros2_cpp_template/ros2_cpp_template.hpp>

using namespace ros2_cpp_template;
using namespace std::chrono_literals;

Ros2CppTemplate::Ros2CppTemplate(const std::string &namespace_) : Node(namespace_) {
  // Parameters
  this->declare_parameter<double>("timer_freq", 0.5);
  double timer_freq = this->get_parameter("timer_freq").as_double();
  RCLCPP_INFO(this->get_logger(), "Timer frequency: '%f'", timer_freq);

  this->declare_parameter<std::string>("topic_pub", "my_topic");
  std::string topic_pub = this->get_parameter("topic_pub").as_string();
  RCLCPP_INFO(this->get_logger(), "Topic publisher: '%s'", topic_pub.c_str());

  this->declare_parameter<std::string>("topic_sub", "my_topic");
  std::string topic_sub = this->get_parameter("topic_sub").as_string();
  RCLCPP_INFO(this->get_logger(), "Topic subscriber: '%s'", topic_sub.c_str());

  this->declare_parameter<std::string>("service_name", "my_service");
  std::string service_name = this->get_parameter("service_name").as_string();
  RCLCPP_INFO(this->get_logger(), "Service name: '%s'", service_name.c_str());

  this->declare_parameter<std::string>("service_client", "my_service");
  std::string service_client_name = this->get_parameter("service_client").as_string();
  RCLCPP_INFO(this->get_logger(), "Service client: '%s'", service_client_name.c_str());

  // Suscribers
  if (topic_sub != "") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        topic_sub, 10,
        std::bind(&Ros2CppTemplate::subscription_callback, this, std::placeholders::_1));
  }

  // Publishers
  if (topic_pub != "") {
    publisher_ = this->create_publisher<std_msgs::msg::String>(topic_pub, 10);
  }

  // Timers
  timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0f / timer_freq),
                                   std::bind(&Ros2CppTemplate::timer_callback, this));

  // Services
  if (service_name != "") {
    service_ = this->create_service<std_srvs::srv::SetBool>(
        service_name, std::bind(&Ros2CppTemplate::service_callback, this, std::placeholders::_1,
                                std::placeholders::_2));
  }

  // Services clients
  if (service_client_name != "") {
    callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(callback_group_, this->get_node_base_interface());

    service_client_ = this->create_client<std_srvs::srv::SetBool>(
        service_client_name, rmw_qos_profile_services_default, callback_group_);
    service_request_       = std::make_shared<std_srvs::srv::SetBool::Request>();
    service_request_->data = true;
    if (!service_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Service not available within 1 second.");
    }
  }

  RCLCPP_INFO(this->get_logger(), "Ros2CppTemplate initialized\n");
}

Ros2CppTemplate::~Ros2CppTemplate() {}

void Ros2CppTemplate::timer_callback() {
  // Check if publisher has been initialized
  if (publisher_) {
    // Publishers
    auto message = std_msgs::msg::String();
    message.data = "Working!";
    RCLCPP_INFO(this->get_logger(), "TOPIC PUBLISH");
    RCLCPP_INFO(this->get_logger(), "Msg: '%s' \n", message.data.c_str());
    publisher_->publish(message);
  }

  // Check if service client has been initialized
  if (service_client_) {
    call_service();
  }
}

void Ros2CppTemplate::subscription_callback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "TOPIC CALLBACK");
  RCLCPP_INFO(this->get_logger(), "Msg: '%s' \n", msg->data.c_str());
}

void Ros2CppTemplate::service_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  RCLCPP_INFO(this->get_logger(), "SERVICE CALLBACK");
  RCLCPP_INFO(this->get_logger(), "request: '%s'", request->data ? "true" : "false");

  response->success = true;
  response->message = "service message";

  RCLCPP_INFO(this->get_logger(), "response: '%s'", response->success ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "message: '%s' \n", response->message.c_str());
}

void Ros2CppTemplate::call_service() {
  RCLCPP_INFO(this->get_logger(), "SERVICE CALL");
  if (service_client_ && service_client_->service_is_ready()) {
    auto result_future = service_client_->async_send_request(service_request_);
    if (callback_group_executor_.spin_until_future_complete(result_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto result = result_future.get();
      RCLCPP_INFO(this->get_logger(), "response value: '%s' \n",
                  result->success ? "true" : "false");
      RCLCPP_INFO(this->get_logger(), "response message: '%s' \n", result->message.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service\n");
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Service client not available\n");
  }
}
