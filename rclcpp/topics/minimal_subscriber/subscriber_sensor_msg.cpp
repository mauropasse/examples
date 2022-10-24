// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>

class DepthDataNode : public rclcpp::Node
{
public:
  DepthDataNode()
  : Node("minimal_subscriber")
  {
    depth_data_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/depth/image_raw",
      10,
      [this](sensor_msgs::msg::Image::UniquePtr msg) {
        depth_msg_ = std::move(msg);
        uint32_t height = depth_msg_->height;
        uint32_t width = depth_msg_->width;
        // height = row
        RCLCPP_INFO(this->get_logger(), "Image (depth) height '%d' - width: '%d'", height, width);

        std::cout << "step: " << depth_msg_->step << std::endl;
        std::cout << "bytes per pixel: " << depth_msg_->step / width << std::endl;

        for (uint32_t i = 0; i << width; i++) {
          for (uint32_t j = 0; j << height; j++) {
            std::cout << depth_msg_->data[i+j] << " ";
          }
          std::cout << std::endl;
        }

      });
  }

  void publish_compressed_depth_frame(){};

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_data_sub_;
  sensor_msgs::msg::Image::UniquePtr depth_msg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthDataNode>());
  rclcpp::shutdown();
  return 0;
}
