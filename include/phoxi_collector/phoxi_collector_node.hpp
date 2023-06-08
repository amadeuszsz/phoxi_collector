// Copyright 2022 Amadeusz Szymko
// Perception for Physical Interaction Laboratory at Poznan University of Technology
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

#ifndef PHOXI_COLLECTOR__PHOXI_COLLECTOR_NODE_HPP_
#define PHOXI_COLLECTOR__PHOXI_COLLECTOR_NODE_HPP_

#include <memory>
#include <string>
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/io/pcd_io.h"
#include "phoxi_collector/visibility_control.hpp"
#include "phoxi_camera_msgs/srv/phoxi_cloud.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "std_srvs/srv/empty.hpp"


namespace phoxi_collector
{

class PHOXI_COLLECTOR_PUBLIC PhoxiCollectorNode : public rclcpp::Node
{
public:
  explicit PhoxiCollectorNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_depth_sub_;
  rclcpp::Client<phoxi_camera_msgs::srv::PhoxiCloud>::SharedPtr cloud_client_;
  rclcpp::Client<phoxi_camera_msgs::srv::PhoxiCloud>::SharedFuture cloud_future_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr collect_service_;
  rclcpp::Node::SharedPtr service_node_;
  phoxi_camera_msgs::srv::PhoxiCloud_Request::SharedPtr cloud_request_;
  void cameraRgbCallback(const sensor_msgs::msg::Image & msg);
  void cameraDepthCallback(const sensor_msgs::msg::Image & msg);
  sensor_msgs::msg::Image img_rgb_msg_;
  sensor_msgs::msg::Image img_depth_msg_;
  void collectService(
    const std_srvs::srv::Empty::Request::SharedPtr request,
    std_srvs::srv::Empty::Response::SharedPtr response);
  std::string save_dir_;
  bool use_phoxi_;
  bool use_external_rgb_;
  bool use_external_depth_;
};


}  // namespace phoxi_collector
#endif  // PHOXI_COLLECTOR__PHOXI_COLLECTOR_NODE_HPP_
