// Copyright 2026 PAL Robotics SL.
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

/*
 * Authors: Christian Rauch, Subhas Das, Denis Stogl, Victor Lopez
 */

#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"
#include "semantic_components/accelerometer.hpp"

class SensorTest;

// implementing and friending so we can access member variables
template <typename S>
class TestableSensor : public S
{
  FRIEND_TEST(SensorTest, validate_all);

public:
  explicit TestableSensor(const std::string & name) : S(name) {}

  virtual ~TestableSensor() = default;
};

class SensorTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    full_interface_names_.reserve(size_);
    for (auto index = 0u; index < size_; ++index)
    {
      full_interface_names_.emplace_back(sensor_name_ + "/" + accel_interface_names_[index]);
    }
  }

  void TearDown() override { sensor_.reset(nullptr); };

protected:
  static constexpr size_t size_ = 6;
  const std::string sensor_name_ = "test_accel";
  std::array<double, 3> angular_acceleration_values_ = {{7.7, 8.8, 9.9}};
  std::array<double, 3> linear_acceleration_values_ = {{4.4, 5.5, 6.6}};
  std::unique_ptr<TestableSensor<semantic_components::Accelerometer>> sensor_;

  std::vector<std::string> full_interface_names_;
  const std::vector<std::string> accel_interface_names_ = {
    "linear_acceleration.x",  "linear_acceleration.y",  "linear_acceleration.z",
    "angular_acceleration.x", "angular_acceleration.y", "angular_acceleration.z",
  };
};

TEST_F(SensorTest, validate_all)
{
  // create the sensor
  sensor_ = std::make_unique<TestableSensor<semantic_components::Accelerometer>>(sensor_name_);

  // validate the component name
  ASSERT_EQ(sensor_->name_, sensor_name_);

  // validate the space reserved for interface_names_ and state_interfaces_
  // Note : Using capacity() for state_interfaces_ as no such interfaces are defined yet
  ASSERT_EQ(sensor_->interface_names_.size(), size_);
  ASSERT_EQ(sensor_->state_interfaces_.capacity(), size_);

  // validate the default interface_names_
  ASSERT_TRUE(std::equal(
    sensor_->interface_names_.begin(), sensor_->interface_names_.end(),
    full_interface_names_.begin(), full_interface_names_.end()));

  // assign values to linear acceleration
  auto linear_acceleration_x = std::make_shared<hardware_interface::StateInterface>(
    sensor_name_, accel_interface_names_[0], &linear_acceleration_values_[0]);
  auto linear_acceleration_y = std::make_shared<hardware_interface::StateInterface>(
    sensor_name_, accel_interface_names_[1], &linear_acceleration_values_[1]);
  auto linear_acceleration_z = std::make_shared<hardware_interface::StateInterface>(
    sensor_name_, accel_interface_names_[2], &linear_acceleration_values_[2]);

  // assign values to angular acceleration
  auto angular_acceleration_x = std::make_shared<hardware_interface::StateInterface>(
    sensor_name_, accel_interface_names_[3], &angular_acceleration_values_[0]);
  auto angular_acceleration_y = std::make_shared<hardware_interface::StateInterface>(
    sensor_name_, accel_interface_names_[4], &angular_acceleration_values_[1]);
  auto angular_acceleration_z = std::make_shared<hardware_interface::StateInterface>(
    sensor_name_, accel_interface_names_[5], &angular_acceleration_values_[2]);

  // create local state interface vector
  std::vector<hardware_interface::LoanedStateInterface> temp_state_interfaces;
  temp_state_interfaces.reserve(size_);

  // insert the interfaces in jumbled sequence
  temp_state_interfaces.emplace_back(angular_acceleration_y);
  temp_state_interfaces.emplace_back(linear_acceleration_y);
  temp_state_interfaces.emplace_back(linear_acceleration_z);
  temp_state_interfaces.emplace_back(angular_acceleration_z);
  temp_state_interfaces.emplace_back(angular_acceleration_x);
  temp_state_interfaces.emplace_back(linear_acceleration_x);

  // now call the function to make them in order like interface_names
  sensor_->assign_loaned_state_interfaces(temp_state_interfaces);

  // validate the count of state_interfaces_
  ASSERT_EQ(sensor_->state_interfaces_.size(), size_);

  // validate the linear_acceleration values
  std::array<double, 3> temp_linear_acceleration_values = sensor_->get_linear_acceleration();
  ASSERT_EQ(temp_linear_acceleration_values, linear_acceleration_values_);

  // validate get_values_as_message
  geometry_msgs::msg::AccelStamped temp_message;
  ASSERT_TRUE(sensor_->get_values_as_message(temp_message));

  ASSERT_EQ(temp_message.accel.linear.x, linear_acceleration_values_[0]);
  ASSERT_EQ(temp_message.accel.linear.y, linear_acceleration_values_[1]);
  ASSERT_EQ(temp_message.accel.linear.z, linear_acceleration_values_[2]);

  ASSERT_EQ(temp_message.accel.angular.x, angular_acceleration_values_[0]);
  ASSERT_EQ(temp_message.accel.angular.y, angular_acceleration_values_[1]);
  ASSERT_EQ(temp_message.accel.angular.z, angular_acceleration_values_[2]);

  // release the state_interfaces_
  sensor_->release_interfaces();

  // validate the count of state_interfaces_
  ASSERT_EQ(sensor_->state_interfaces_.size(), 0u);
}
