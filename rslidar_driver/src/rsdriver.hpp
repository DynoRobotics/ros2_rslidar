/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *	Copyright (C) 2019, Dyno Robotics, Fredrik Löfgren
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver interface for the RSLIDAR 3D LIDARs
 */
#ifndef _RSDRIVER_H_
#define _RSDRIVER_H_

#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
#include "rslidar_msgs/msg/rslidar_packet.hpp"
#include "rslidar_msgs/msg/rslidar_scan.hpp"
#include "input.hpp"

using std::placeholders::_1;


namespace rslidar_driver
{
class rslidarDriver : public rclcpp::Node
{
public:
  /**
 * @brief rslidarDriver
 * @param node          raw packet output topic
 * @param private_nh    通过这个节点传参数
 */
  rslidarDriver();

  ~rslidarDriver();

  bool poll(void);
  void difopPoll(void);

private:
  /// Callback for dynamic reconfigure
  void on_parameter_event(const rcl_interfaces::msg::ParameterEvent::SharedPtr event, rclcpp::Logger logger);
  /// Callback for skip num for time synchronization
  void skipNumCallback(const std_msgs::msg::Int32::SharedPtr skip_num);

  // configuration parameters
  struct
  {
    std::string frame_id;  ///< tf frame ID
    std::string model;     ///< device model name
    int npackets;          ///< number of packets to collect
    double rpm;            ///< device rotation rate (RPMs)
    double time_offset;    ///< time in seconds added to each  time stamp
    int cut_angle;
  } config_;

  std::shared_ptr<Input> msop_input_;
  std::shared_ptr<Input> difop_input_;
  rclcpp::Publisher<rslidar_msgs::msg::RslidarScan>::SharedPtr msop_output_;
  rclcpp::Publisher<rslidar_msgs::msg::RslidarPacket>::SharedPtr difop_output_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr output_sync_;
  // Converter convtor_;
  /** diagnostics updater */
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  std::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
  std::thread difop_thread_;
  
  // add for time synchronization
  bool time_synchronization_;
  uint32_t skip_num_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr skip_num_sub_;
};

}  // namespace rslidar_driver

#endif
