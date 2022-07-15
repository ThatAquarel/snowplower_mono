/**
*
*  \author     Paul Bovbel <pbovbel@clearpathrobotics.com>
*  \copyright  Copyright (c) 2014-2015, Clearpath Robotics, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Clearpath Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to code@clearpathrobotics.com
*
*/

#include "hw_base/hw_hardware.h"
#include <boost/assign/list_of.hpp>
#include <iostream>

namespace
{
  const uint8_t LEFT = 0, RIGHT = 1;
};

namespace hw_base
{

  /**
  * Initialize Hw hardware
  */
  HwHardware::HwHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq)
    :
    nh_(nh),
    private_nh_(private_nh)
    // system_status_task_(hw_status_msg_),
    // power_status_task_(hw_status_msg_),
    // safety_status_task_(hw_status_msg_),
    // software_status_task_(hw_status_msg_, target_control_freq)
  {
    private_nh_.param<double>("wheel_diameter", wheel_diameter_, 0.3302);
    private_nh_.param<double>("max_accel", max_accel_, 5.0);
    private_nh_.param<double>("max_speed", max_speed_, 1.0);
    private_nh_.param<double>("polling_timeout_", polling_timeout_, 10.0);

    std::string port;
    private_nh_.param<std::string>("port", port, "/dev/prolific");

    // horizon_legacy::connect(port);
    // horizon_legacy::configureLimits(max_speed_, max_accel_);
    resetTravelOffset();
    registerControlInterfaces();
  }

  /**
  * Get current encoder travel offsets from MCU and bias future encoder readings against them
  */
  void HwHardware::resetTravelOffset()
  {
    // horizon_legacy::Channel<clearpath::DataEncoders>::Ptr enc = horizon_legacy::Channel<clearpath::DataEncoders>::requestData(
    //   polling_timeout_);
    // if (enc)
    // {
    //   for (int i = 0; i < 4; i++)
    //   {
    //     joints_[i].position_offset = linearToAngular(enc->getTravel(i % 2));
    //   }
    // }
    // else
    // {
    //   ROS_ERROR("Could not get encoder data to calibrate travel offset");
    // }
  }


  /**
  * Register interfaces with the RobotHW interface manager, allowing ros_control operation
  */
  void HwHardware::registerControlInterfaces()
  {
    ros::V_string joint_names = boost::assign::list_of("front_left_wheel")
      ("front_right_wheel")("rear_left_wheel")("rear_right_wheel");
    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
      hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                              &joints_[i].position, &joints_[i].velocity,
                                                              &joints_[i].effort);
      joint_state_interface_.registerHandle(joint_state_handle);

      hardware_interface::JointHandle joint_handle(
        joint_state_handle, &joints_[i].velocity_command);
      velocity_joint_interface_.registerHandle(joint_handle);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
  }

  /**
  * Pull latest speed and travel measurements from MCU, and store in joint structure for ros_control
  */
  void HwHardware::updateJointsFromHardware()
  {

    // horizon_legacy::Channel<clearpath::DataEncoders>::Ptr enc = horizon_legacy::Channel<clearpath::DataEncoders>::requestData(
    //   polling_timeout_);
    // if (enc)
    // {
    //   ROS_DEBUG_STREAM("Received travel information (L:" << enc->getTravel(LEFT) << " R:" << enc->getTravel(RIGHT) << ")");
    //   for (int i = 0; i < 4; i++)
    //   {
    //     double delta = linearToAngular(enc->getTravel(i % 2)) - joints_[i].position - joints_[i].position_offset;

    //     // detect suspiciously large readings, possibly from encoder rollover
    //     if (std::abs(delta) < 1.0)
    //     {
    //       joints_[i].position += delta;
    //     }
    //     else
    //     {
    //       // suspicious! drop this measurement and update the offset for subsequent readings
    //       joints_[i].position_offset += delta;
    //       ROS_DEBUG("Dropping overflow measurement from encoder");
    //     }
    //   }
    // }

    // horizon_legacy::Channel<clearpath::DataDifferentialSpeed>::Ptr speed = horizon_legacy::Channel<clearpath::DataDifferentialSpeed>::requestData(
    //   polling_timeout_);
    // if (speed)
    // {
    //   ROS_DEBUG_STREAM("Received linear speed information (L:" << speed->getLeftSpeed() << " R:" << speed->getRightSpeed() << ")");
    //   for (int i = 0; i < 4; i++)
    //   {
    //     if (i % 2 == LEFT)
    //     {
    //       joints_[i].velocity = linearToAngular(speed->getLeftSpeed());
    //     }
    //     else
    //     { // assume RIGHT
    //       joints_[i].velocity = linearToAngular(speed->getRightSpeed());
    //     }
    //   }
    // }
  }

  /**
  * Get latest velocity commands from ros_control via joint structure, and send to MCU
  */
  void HwHardware::writeCommandsToHardware()
  {
    // double diff_speed_left = angularToLinear(joints_[LEFT].velocity_command);
    // double diff_speed_right = angularToLinear(joints_[RIGHT].velocity_command);

    // limitDifferentialSpeed(diff_speed_left, diff_speed_right);

    // horizon_legacy::controlSpeed(diff_speed_left, diff_speed_right, max_accel_, max_accel_);

    std::cout << "HwHardware::writeCommandsToHardware ";
    std::cout << "velocity_command " << joints_[LEFT].velocity_command << " " << joints_[RIGHT].velocity_command << " ";
    std::cout << "velocity " << joints_[LEFT].velocity << " " << joints_[RIGHT].velocity << " ";
    std::cout << std::endl;
  }

  /**
  * Update diagnostics with control loop timing information
  */
  void HwHardware::reportLoopDuration(const ros::Duration &duration)
  {
    // software_status_task_.updateControlFrequency(1 / duration.toSec());
  }

  /**
  * Scale left and right speed outputs to maintain ros_control's desired trajectory without saturating the outputs
  */
  void HwHardware::limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right)
  {
    double large_speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));

    if (large_speed > max_speed_)
    {
      diff_speed_left *= max_speed_ / large_speed;
      diff_speed_right *= max_speed_ / large_speed;
    }
  }

  /**
  * Hw reports travel in metres, need radians for ros_control RobotHW
  */
  double HwHardware::linearToAngular(const double &travel) const
  {
    return travel / wheel_diameter_ * 2;
  }

  /**
  * RobotHW provides velocity command in rad/s, Hw needs m/s,
  */
  double HwHardware::angularToLinear(const double &angle) const
  {
    return angle * wheel_diameter_ / 2;
  }


}  // namespace hw_base
