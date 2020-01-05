/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver node for the Robosense 3D LIDARs.
 */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "rsdriver.hpp"

using namespace rslidar_driver;
volatile sig_atomic_t flag = 1;

static void my_handler(int sig)
{
  flag = 0;
}

int main(int argc, char* argv[])
{
  signal(SIGINT, my_handler);
  rclcpp::init(argc, argv);
  
  // start the driver
  auto node = std::make_shared<rslidar_driver::rslidarDriver>();
  
  // loop until shut down or end of file
  while (rclcpp::ok() && node->poll())
  {
    rclcpp::spin_some(node);
  }
  
  rclcpp::shutdown();
  return 0;
}
