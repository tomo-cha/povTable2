// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef READ_WRITE_NODE_HPP_
#define READ_WRITE_NODE_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/sync4_set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/sync4_get_position.hpp"


class Sync4ReadWriteNode : public rclcpp::Node
{
public:
  using Sync4SetPosition = dynamixel_sdk_custom_interfaces::msg::Sync4SetPosition;
  using Sync4GetPosition = dynamixel_sdk_custom_interfaces::srv::Sync4GetPosition;


  Sync4ReadWriteNode();
  virtual ~Sync4ReadWriteNode();

private:
  rclcpp::Subscription<Sync4SetPosition>::SharedPtr set_position_subscriber_;
  rclcpp::Service<Sync4GetPosition>::SharedPtr get_position_server_;

  int present_position;
};

#endif  // READ_WRITE_NODE_HPP_
