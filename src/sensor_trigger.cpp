// Copyright 2022 Tier IV, Inc.
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

#include <sensor_trigger/sensor_trigger.hpp>

SensorTrigger::SensorTrigger()
{
  std::string gpio_mapping_file;
  private_nh_.getParam("frame_rate", fps_);
  private_nh_.getParam("phase", phase_);
  private_nh_.getParam("gpio_name", gpio_name_);
  private_nh_.getParam("pulse_width_ms", pulse_width_ms_);
  private_nh_.getParam("gpio_mapping_file", gpio_mapping_file);

  gpio_mapping_ = YAML::LoadFile(gpio_mapping_file);

  if (!get_gpio_chip_and_line()) {
    ROS_ERROR_STREAM("No valid trigger GPIO specified. Not using triggering on GPIO name " << gpio_name_ << ".");
    exit(1);
  }

  if (!gpio_handler_.init_gpio_pin(gpio_chip_, gpio_line_, GPIO_OUTPUT)) {
    ROS_ERROR_STREAM("Failed to initialize GPIO trigger. Not using triggering on GPIO chip number "
                      << gpio_chip_ << "line number " << gpio_line_ << ".");
    exit(1);
  }

  if (fps_ < 1.0) {
    ROS_ERROR_STREAM("Unable to trigger slower than 1 fps. Not using triggering on GPIO chip number "
                      << gpio_chip_ << "line number " << gpio_line_ << ".");
    exit(1);
  }

  trigger_time_publisher_ = nh_.advertise<std_msgs::Time>("trigger_time", 1000);
}

SensorTrigger::~SensorTrigger(){}

void SensorTrigger::run()
{
  std_msgs::Time trigger_time_msg;

  // Start on the first time after TOS
  int64_t start_nsec;
  int64_t end_nsec;
  int64_t target_nsec;
  int64_t interval_nsec = (int64_t)(1e9 / fps_);
  int64_t pulse_width = pulse_width_ms_ * 1e6;  // millisecond -> nanoseconds
  int64_t wait_nsec = 0;
  int64_t now_nsec = 0;
  // Fix this later to remove magic numbers
  if (std::abs(phase_) <= 1e-7) {
    start_nsec = 0;
  } else {
    start_nsec = interval_nsec * (int64_t)(phase_ * 10) / 3600;
  }
  target_nsec = start_nsec;
  end_nsec = start_nsec - interval_nsec + 1e9;

  while (ros::ok()) {
    // Do triggering stuff
    // Check current time - assume ROS uses best clock source
    do {
      now_nsec = ros::Time::now().nsec;
      if (now_nsec < end_nsec) {
        while (now_nsec > target_nsec) {
          target_nsec = target_nsec + interval_nsec;
        }
        // FIX: what about very small phases and fast framerates giving a negative number?
        wait_nsec = target_nsec - now_nsec - 1e7;
      } else {
        target_nsec = start_nsec;
        wait_nsec = 1e9 - now_nsec + start_nsec - 1e7;
      }
      // Keep waiting for half the remaining time until the last millisecond.
      // This is required as sleep_for tends to oversleep significantly
      if (wait_nsec > 1e7) {
        ros::Duration(0, wait_nsec/2).sleep();
      }
    } while (wait_nsec > 1e7);
    // std::lock_guard<std::mutex> guard(iomutex_);
    // Block the last millisecond
    now_nsec = ros::Time::now().nsec;
    if (start_nsec == end_nsec) {
      while (now_nsec > 1e7) {
        now_nsec = ros::Time::now().nsec;
      }
    } else if (now_nsec < end_nsec) {
      while (now_nsec < target_nsec) {
        now_nsec = ros::Time::now().nsec;
      }
    } else {
      while (now_nsec > end_nsec || now_nsec < start_nsec) {
        now_nsec = ros::Time::now().nsec;
      }
    }
    // Trigger!
    bool to_high = gpio_handler_.set_gpio_pin_state(GPIO_HIGH);
    ros::Duration(0, pulse_width).sleep();
    int64_t now_sec = (ros::Time::now().nsec - pulse_width) / 1e9; // subtract pulse width to correct timestamp
    trigger_time_msg.data.sec = (int32_t)now_sec;
    trigger_time_msg.data.nsec = (uint32_t)now_nsec;
    trigger_time_publisher_.publish(trigger_time_msg);
    bool to_low = gpio_handler_.set_gpio_pin_state(GPIO_LOW);
    target_nsec = target_nsec + interval_nsec >= 1e9 ? start_nsec : target_nsec + interval_nsec;
    if (!(to_high && to_low)) {
      ROS_ERROR_STREAM("Failed to set GPIO status: " << strerror(errno));
      exit(1);
    }
  }
}

bool SensorTrigger::get_gpio_chip_and_line()
{
  if (
    gpio_mapping_[gpio_name_] && gpio_mapping_[gpio_name_]["chip"] &&
    gpio_mapping_[gpio_name_]["line"]) {
    gpio_chip_ = gpio_mapping_[gpio_name_]["chip"].as<unsigned int>(),
    gpio_line_ = gpio_mapping_[gpio_name_]["line"].as<unsigned int>();
    return true;
  }
  return false;
}