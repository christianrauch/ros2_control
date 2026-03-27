// Copyright 2026 Christian Rauch
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

#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include <geometry_msgs/msg/accel_stamped.hpp>
#include <semantic_components/semantic_component_interface.hpp>

namespace semantic_components
{
class Accelerometer : public SemanticComponentInterface<geometry_msgs::msg::AccelStamped>
{
public:
  explicit Accelerometer(const std::string & name)
  : SemanticComponentInterface(
      name, {
              {name + "/" + "linear_acceleration.x"},
              {name + "/" + "linear_acceleration.y"},
              {name + "/" + "linear_acceleration.z"},
              {name + "/" + "angular_acceleration.x"},
              {name + "/" + "angular_acceleration.y"},
              {name + "/" + "angular_acceleration.z"},
            })
  {
  }

  /// Return linear acceleration.
  /**
   * Return linear acceleration [m/s^2]
   *
   * \return array of size 3 with linear acceleration values (x, y, z).
   */
  std::array<double, 3> get_linear_acceleration() const
  {
    update_data_from_interfaces();
    return std::array<double, 3>{{data_[0], data_[1], data_[2]}};
  }

  /// Return angular acceleration.
  /**
   * Return angular acceleration [rad/s^2]
   *
   * \return array of size 3 with angular acceleration values (x, y, z).
   */
  std::array<double, 3> get_angular_acceleration() const
  {
    update_data_from_interfaces();
    return std::array<double, 3>{{data_[3], data_[4], data_[5]}};
  }

  /// Return message with linear and angular acceleration
  /**
   * Constructs and return a message from the current values.
   * \return message from values
   */
  bool get_values_as_message(geometry_msgs::msg::AccelStamped & message) const
  {
    update_data_from_interfaces();

    message.accel.linear.x = data_[0];
    message.accel.linear.y = data_[1];
    message.accel.linear.z = data_[2];

    message.accel.angular.x = data_[3];
    message.accel.angular.y = data_[4];
    message.accel.angular.z = data_[5];

    return true;
  }

private:
  /**
   * @brief Update the data array from the state interfaces.
   * @note This method is thread-safe and non-blocking.
   * @note This method might return stale data if the data is not updated. This is to ensure that
   * the data from the sensor is not discontinuous.
   */
  void update_data_from_interfaces() const
  {
    for (auto i = 0u; i < data_.size(); ++i)
    {
      const std::optional<double> data = state_interfaces_[i].get().get_optional();
      if (data.has_value())
      {
        data_[i] = data.value();
      }
    }
  }

  mutable std::array<double, 6> data_;
};

}  // namespace semantic_components
