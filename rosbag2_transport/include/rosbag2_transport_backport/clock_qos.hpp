// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef ROSBAG2_TRANSPORT_BACKPORT__CLOCK_QOS_HPP_
#define ROSBAG2_TRANSPORT_BACKPORT__CLOCK_QOS_HPP_

#include "rclcpp/qos.hpp"

#include "rcpputils/shared_library.hpp"

namespace rosbag2_transport
{

class ClockQoS : public rclcpp::QoS
{
public:
  RCLCPP_PUBLIC
  explicit
  ClockQoS(
    const rclcpp::QoSInitialization & qos_initialization = rclcpp::KeepLast(1));
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT_BACKPORT__CLOCK_QOS_HPP_
