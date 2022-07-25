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
#include <sstream>
#include <CppLinuxSerial/SerialPort.hpp>
#include <vector>
#include <cassert>
#include <cstring>

using namespace mn::CppLinuxSerial;

SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_230400, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);

#define BUF_SIZE 32
#define FRAME_START_FLAG 0x7E
#define FRAME_END_FLAG 0x7D

enum commands : uint8_t
{
  NUL = 0x00,
  SYN = 0xAB,
  ACK = 0x4B,
  NAK = 0x5A,
  ERR = 0x3C,

  CTRL = 0x01,
  ODOM = 0x02
};

struct packet_t
{
  commands cmd;
  size_t len;
  uint8_t *buf;

  packet_t(uint8_t *_buf, size_t _len) : cmd(NUL),
                                         len(_len),
                                         buf(_buf){};
  packet_t(commands _cmd, uint8_t *_buf, size_t _len) : cmd(_cmd),
                                                        len(_len),
                                                        buf(_buf){};
};

uint16_t comm_crc16(uint8_t *data_p, size_t len)
{
  if (len == 0 || data_p == nullptr)
    return 0xFFFF;

  uint8_t x;
  uint16_t crc = 0xFFFF;

  while (len--)
  {
    x = crc >> 8 ^ *data_p++;
    x ^= x >> 4;
    crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x << 5)) ^ ((uint16_t)x);
  }
  return crc;
}

void comm_init()
{
  serialPort.SetTimeout(-1);
  serialPort.Open();
}

std::vector<uint8_t> r_buffer;
std::vector<uint8_t> temp_r_buffer;
void fill_r_buffer()
{
  if (!serialPort.Available() && r_buffer.size())
    return;

  temp_r_buffer.clear();
  serialPort.ReadBinary(temp_r_buffer);
  r_buffer.insert(r_buffer.end(), temp_r_buffer.begin(), temp_r_buffer.end());
}

void comm_recv_packet(struct packet_t *packet)
{
find_packet:
  fill_r_buffer();

  while (r_buffer.front() != FRAME_START_FLAG)
  {
    if (!r_buffer.size())
      goto find_packet;
    r_buffer.erase(r_buffer.begin());
  }
  if (r_buffer.size() < 4)
    goto find_packet;

  r_buffer.erase(r_buffer.begin());

  packet->cmd = static_cast<commands>(r_buffer.front());
  r_buffer.erase(r_buffer.begin());

  uint16_t requested_len = r_buffer.at(0) | (r_buffer.at(1) << 8);
  r_buffer.erase(r_buffer.begin(), r_buffer.begin() + 2);
  if (requested_len > BUF_SIZE)
    requested_len = BUF_SIZE;
  packet->len = requested_len;

  while (r_buffer.size() < (requested_len + 3))
    fill_r_buffer();

  std::copy(r_buffer.begin(), r_buffer.begin() + requested_len, packet->buf);
  r_buffer.erase(r_buffer.begin(), r_buffer.begin() + requested_len);

  uint16_t crc = r_buffer.at(0) | (r_buffer.at(1) << 8);
  r_buffer.erase(r_buffer.begin(), r_buffer.begin() + 2);

  if (comm_crc16(packet->buf, requested_len) != crc)
    packet->cmd = NAK;
  if (r_buffer.at(0) != FRAME_END_FLAG)
    packet->cmd = ERR;
}

std::vector<uint8_t> w_buffer;
void comm_send_packet(struct packet_t *packet)
{
  uint16_t crc = comm_crc16(packet->buf, packet->len);

  w_buffer.push_back(0x00);
  w_buffer.push_back(0x00);
  w_buffer.push_back(0x00);
  w_buffer.push_back(0x00);

  w_buffer.push_back(FRAME_START_FLAG);
  w_buffer.push_back(packet->cmd);

  uint16_t len = (uint16_t)packet->len;
  w_buffer.insert(w_buffer.end(), (uint8_t *)&len, (uint8_t *)&len + 2);
  if (packet->buf != nullptr && packet->len != 0)
    w_buffer.insert(w_buffer.end(), packet->buf, packet->buf + packet->len);
  w_buffer.insert(w_buffer.end(), (uint8_t *)&crc, (uint8_t *)&crc + 2);
  w_buffer.push_back(FRAME_END_FLAG);

  w_buffer.push_back(0x00);
  w_buffer.push_back(0x00);
  w_buffer.push_back(0x00);
  w_buffer.push_back(0x00);

  serialPort.WriteBinary(w_buffer);
  w_buffer.clear();
}

uint8_t packet_buffer[BUF_SIZE];
packet_t *data_packet = new packet_t(packet_buffer, BUF_SIZE);
packet_t *ack_packet = new packet_t(ACK, nullptr, 0);

void acknowledge()
{
  comm_recv_packet(ack_packet);
  assert(ack_packet->cmd == ACK && "SYN failed");
}

void send_recv_command(commands cmd, uint8_t *buf, size_t buf_len)
{
  data_packet->cmd = cmd;
  data_packet->len = buf_len;
  if (buf_len > BUF_SIZE)
    buf_len = BUF_SIZE;
  if (buf_len > 0 && buf != nullptr)
    std::memcpy(data_packet->buf, buf, buf_len);

  comm_send_packet(data_packet);
  acknowledge();

  comm_recv_packet(data_packet);
  acknowledge();
}

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
      : nh_(nh),
        private_nh_(private_nh)
  // system_status_task_(hw_status_msg_),
  // power_status_task_(hw_status_msg_),
  // safety_status_task_(hw_status_msg_),
  // software_status_task_(hw_status_msg_, target_control_freq)
  {
    private_nh_.param<double>("max_accel", max_accel_, 5.0);
    private_nh_.param<double>("max_speed", max_speed_, 1.0);
    private_nh_.param<double>("polling_timeout_", polling_timeout_, 10.0);

    std::string port;
    private_nh_.param<std::string>("port", port, "/dev/prolific");

    // serialPort = new SerialPort("/dev/ttyUSB0", BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    // serialPort->SetTimeout(-1); // Block when reading until any data is received
    // serialPort->Open();

    // serialPort->Write("1,0,255,1,0,255");

    comm_init();

    // horizon_legacy::connect(port);
    // horizon_legacy::configureLimits(max_speed_, max_accel_);
    resetTravelOffset();
    registerControlInterfaces();
  }

  HwHardware::~HwHardware()
  {
    serialPort->Close();
    delete serialPort;
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
    ros::V_string joint_names = boost::assign::list_of("front_left_wheel")("front_right_wheel")("rear_left_wheel")("rear_right_wheel");
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
    send_recv_command(ODOM, nullptr, 0);

    int m0, m1;

    int pulses[2];
    std::memcpy((uint8_t*)pulses, data_packet->buf, 8);

    for (int i = 0; i < 4; i++)
    {
      double rad = pulses[i%2] / 475.0 * 6.283185307;
      double delta = rad - joints_[i].position - joints_[i].position_offset;

      if (std::abs(delta) < 1.0)
      {
        joints_[i].position += delta;
      }
      else
      {
        joints_[i].position_offset += delta;
        ROS_DEBUG("Dropping overflow measurement from encoder");
      }
    }

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
    float a = joints_[LEFT].velocity_command;
    float b = joints_[RIGHT].velocity_command;

    int vf, vr;
    vf = std::abs(static_cast<int>(a / 3.1415926 * 255)) & 0xff;
    vr = std::abs(static_cast<int>(b / 3.1415926 * 255)) & 0xff;

    uint8_t proto[6] = {(uint8_t)vf, a > 0, a <= 0, (uint8_t)vr, b > 0, b <= 0};
    send_recv_command(CTRL, proto, 6);
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

} // namespace hw_base
