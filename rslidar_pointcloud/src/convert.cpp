/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017 Robosense, Tony Zhang
 *	Copyright (C) 2019, Dyno Robotics, Fredrik Löfgren
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw RSLIDAR 3D LIDAR packets to PointCloud2.

*/
#include "rslidar_pointcloud/convert.hpp"

namespace rslidar_pointcloud
{

Convert::Convert() : Convert(rclcpp::NodeOptions()) {}

/** @brief Constructor. */
Convert::Convert(const rclcpp::NodeOptions& options) : Node("cloud_node", options), data_(new rslidar_rawdata::RawData(this))
{
  this->declare_parameter("output_points_topic", rclcpp::ParameterValue("rslidar_points"));

  data_->loadConfigFile();  // load lidar parameters
  this->get_parameter_or("model", model_, std::string("RS16"));

  // advertise output point cloud (before subscribing to input data)
  std::string output_points_topic;
  this->get_parameter("output_points_topic", output_points_topic);
  output_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_points_topic, 10);


  // subscribe to rslidarScan packets
  std::string input_packets_topic;
  this->get_parameter_or("input_packets_topic", input_packets_topic, std::string("rslidar_packets"));
  rslidar_scan_ = this->create_subscription<rslidar_msgs::msg::RslidarScan>(input_packets_topic, 10, std::bind(&Convert::processScan, this, std::placeholders::_1));
}

/** @brief Callback for raw scan messages. */
  void Convert::processScan(const rslidar_msgs::msg::RslidarScan::UniquePtr scanMsg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);
  outPoints->header.frame_id = scanMsg->header.frame_id;
  outPoints->clear();
  if (model_ == "RS16")
  {
    outPoints->height = 16;
    outPoints->width = 24 * (int)scanMsg->packets.size();
    outPoints->is_dense = false;
    outPoints->resize(outPoints->height * outPoints->width);
  }
  else if (model_ == "RS32" || model_ == "RSBPEARL" || model_ == "RSBPEARL_MINI")
  {
    outPoints->height = 32;
    outPoints->width = 12 * (int)scanMsg->packets.size();
    outPoints->is_dense = false;
    outPoints->resize(outPoints->height * outPoints->width);
  }

  // process each packet provided by the driver

  data_->block_num = 0;
  for (size_t i = 0; i < scanMsg->packets.size(); ++i)
  {
    data_->unpack(scanMsg->packets[i], outPoints);
  }
  sensor_msgs::msg::PointCloud2 outMsg;
  pcl::toROSMsg(*outPoints, outMsg);
  outMsg.header.stamp = scanMsg->header.stamp;

  output_->publish(outMsg);
}
}  // namespace rslidar_pointcloud
