/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2016 Robosense, Tony Zhang
 *  Copyright (C) 2019 Dyno Robotics, Fredrik Löfgren
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS node converts raw RSLIDAR LIDAR packets to PointCloud2.

*/
#include <memory>
#include "convert.hpp"

/** Main node entry point. */
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rslidar_pointcloud::Convert>());
  rclcpp::shutdown();
  return 0;
}
