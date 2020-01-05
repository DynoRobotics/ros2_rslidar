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
 *  ROS driver node for the Robosense 3D LIDARs.
 */
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rsdriver.hpp"

using namespace rslidar_driver;

int main(int argc, char* argv[])
{
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
