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
  subscription_ = this->create_subscription<std_msgs::msg::String>(
      topic_sub, 10,
      std::bind(&Ros2CppTemplate::subscription_callback, this, std::placeholders::_1));

  custom_subscription_ = this->create_subscription<ros2_cpp_template::msg::MyTemplateMsg>(
      topic_sub, 10,
      std::bind(&Ros2CppTemplate::custom_subscription_callback, this, std::placeholders::_1));

  // Publishers
  publisher_ = this->create_publisher<std_msgs::msg::String>(topic_pub, 10);

  custom_publisher_ = this->create_publisher<ros2_cpp_template::msg::MyTemplateMsg>(topic_pub, 10);

  // Timers
  timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0f / timer_freq),
                                   std::bind(&Ros2CppTemplate::timer_callback, this));

  // Services
  service_ = this->create_service<std_srvs::srv::SetBool>(
      service_name, std::bind(&Ros2CppTemplate::service_callback, this, std::placeholders::_1,
                              std::placeholders::_2));

  custom_service_ = this->create_service<ros2_cpp_template::srv::MyTemplateSrv>(
      service_name, std::bind(&Ros2CppTemplate::custom_service_callback, this,
                              std::placeholders::_1, std::placeholders::_2));

  // Services clients
  service_client_        = this->create_client<std_srvs::srv::SetBool>(service_client_name);
  service_request_       = std::make_shared<std_srvs::srv::SetBool::Request>();
  service_request_->data = true;
  while (!service_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      break;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  custom_service_client_ =
      this->create_client<ros2_cpp_template::srv::MyTemplateSrv>(service_client_name);
  custom_service_request_ = std::make_shared<ros2_cpp_template::srv::MyTemplateSrv::Request>();
  custom_service_request_->input = true;
  while (!custom_service_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      break;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
}

Ros2CppTemplate::~Ros2CppTemplate() {}

void Ros2CppTemplate::subscription_callback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

void Ros2CppTemplate::custom_subscription_callback(
    const ros2_cpp_template::msg::MyTemplateMsg::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data ? "true" : "false");
}

void Ros2CppTemplate::timer_callback() {
  // Publishers
  auto message = std_msgs::msg::String();
  message.data = "Working!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);

  // Services clients
  auto result_future = service_client_->async_send_request(service_request_);
  RCLCPP_INFO(this->get_logger(), "Service called with request: '%s'",
              service_request_->data ? "true" : "false");

  // Wait for the result.
  if (result_future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
    service_response_ = result_future.get();
    RCLCPP_INFO(this->get_logger(), "Service called with response value: '%s'",
                service_response_->success ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Service called with response message: '%s'",
                service_response_->message.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }

  // Custom services clients
  auto custom_result_future = custom_service_client_->async_send_request(custom_service_request_);
  RCLCPP_INFO(this->get_logger(), "Custom Service called with request: '%s'",
              custom_service_request_->input ? "true" : "false");

  // Wait for the result.
  if (custom_result_future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
    custom_service_response_ = custom_result_future.get();
    RCLCPP_INFO(this->get_logger(), "Custom Service called with response value: '%s'",
                custom_service_response_->output ? "true" : "false");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }
}

void Ros2CppTemplate::service_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Service called with request: '%s'",
              request->data ? "true" : "false");

  response->success = true;
  response->message =
      "Service called with request: '" + std::string(request->data ? "true" : "false") + "'";

  RCLCPP_INFO(this->get_logger(), "Service called with response: '%s'",
              response->success ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "Service called with message: '%s'", response->message.c_str());
}

void Ros2CppTemplate::custom_service_callback(
    const std::shared_ptr<ros2_cpp_template::srv::MyTemplateSrv::Request> request,
    std::shared_ptr<ros2_cpp_template::srv::MyTemplateSrv::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Custom Service called with request: '%s'",
              request->input ? "true" : "false");

  response->output = true;

  RCLCPP_INFO(this->get_logger(), "Custom Service called with response: '%s'",
              response->output ? "true" : "false");
}