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

#include "phoxi_collector/phoxi_collector_node.hpp"


using namespace std::chrono_literals;

namespace phoxi_collector
{
using ServiceResponseFuture = rclcpp::Client<phoxi_camera_msgs::srv::PhoxiCloud>::SharedFuture;

PhoxiCollectorNode::PhoxiCollectorNode(const rclcpp::NodeOptions & options)
: Node("phoxi_collector", options)
{
  camera_rgb_sub_ =
    this->create_subscription<sensor_msgs::msg::Image>(
    "image_raw", 1,
    std::bind(&PhoxiCollectorNode::cameraRgbCallback, this, std::placeholders::_1));
  camera_depth_sub_ =
    this->create_subscription<sensor_msgs::msg::Image>(
    "depth_raw", 1,
    std::bind(&PhoxiCollectorNode::cameraDepthCallback, this, std::placeholders::_1));
  cloud_client_ = this->create_client<phoxi_camera_msgs::srv::PhoxiCloud>(
    "/phoxi/phoxi_camera/cloud");
  collect_service_ =
    this->create_service<std_srvs::srv::Empty>(
    "~/collect_data",
    std::bind(
      &PhoxiCollectorNode::collectService, this, std::placeholders::_1,
      std::placeholders::_2));
  cloud_request_ = std::make_shared<phoxi_camera_msgs::srv::PhoxiCloud::Request>();
  save_dir_ = this->declare_parameter("save_dir", "");
  use_phoxi_ = this->declare_parameter("use_phoxi", true);
  use_external_rgb_ = this->declare_parameter("use_external_rgb", true);
  use_external_depth_ = this->declare_parameter("use_external_depth", true);
  if (save_dir_.empty()) {
    save_dir_.append(std::getenv("HOME"));
    save_dir_.append("/.PhotoneoPhoXiControl/");
  } else {
    if (save_dir_.back() != '/') {
      save_dir_.append("/");
    }
  }
}

void PhoxiCollectorNode::cameraRgbCallback(const sensor_msgs::msg::Image & msg)
{
  img_rgb_msg_ = msg;
}

void PhoxiCollectorNode::cameraDepthCallback(const sensor_msgs::msg::Image & msg)
{
  img_depth_msg_ = msg;
}

void PhoxiCollectorNode::collectService(
  const std_srvs::srv::Empty::Request::SharedPtr request,
  std_srvs::srv::Empty::Response::SharedPtr response)
{
  auto stamp = this->get_clock()->now();
  const std::string filepath_rgb_img = save_dir_ + "rgb_" + std::to_string(stamp.seconds()) +
    ".png";
  const std::string filepath_depth_img = save_dir_ + "depth_" + std::to_string(stamp.seconds()) +
    ".png";
  const std::string filepath_cloud = save_dir_ + "cloud_" + std::to_string(stamp.seconds()) +
    ".pcd";

  if (use_external_rgb_) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img_rgb_msg_, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    cv::imwrite(filepath_rgb_img, cv_ptr->image);
    RCLCPP_INFO(this->get_logger(), "RGB image saved to %s", filepath_rgb_img.c_str());
  }

  if (use_external_depth_) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img_depth_msg_, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    cv::imwrite(filepath_depth_img, cv_ptr->image);
    RCLCPP_INFO(this->get_logger(), "Depth image saved to %s", filepath_depth_img.c_str());
  }

  if (use_phoxi_) {
    auto response_received_callback = [this, filepath_cloud](ServiceResponseFuture future) {
        auto result = future.get();
        auto cloud_msg = result->cloud;
        pcl::PointCloud<pcl::PointXYZ> pcd;
        pcl::fromROSMsg(cloud_msg, pcd);
        pcl::io::savePCDFileBinaryCompressed(filepath_cloud, pcd);
        RCLCPP_INFO(this->get_logger(), "Pointcloud saved to %s", filepath_cloud.c_str());
      };

    auto future_result =
      cloud_client_->async_send_request(cloud_request_, response_received_callback);
  }
}

}  // namespace phoxi_collector

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(phoxi_collector::PhoxiCollectorNode)
