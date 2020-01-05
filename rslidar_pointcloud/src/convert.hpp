/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017 Robosense, Tony Zhang
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Robosense 3D LIDAR packets to PointCloud2.

*/
#ifndef _CONVERT_H_
#define _CONVERT_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rslidar_msgs/msg/rslidar_packet.hpp>
#include <rslidar_msgs/msg/rslidar_scan.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "rawdata.hpp"

namespace rslidar_pointcloud
{
class Convert : public rclcpp::Node
{
public:
  Convert();

  ~Convert()
  {
  }

private:
  
  void processScan(const rslidar_msgs::msg::RslidarScan::SharedPtr skip_num);

  std::shared_ptr<rslidar_rawdata::RawData> data_;
  rclcpp::Subscription<rslidar_msgs::msg::RslidarScan>::SharedPtr rslidar_scan_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_;
};

}  // namespace rslidar_pointcloud
#endif
