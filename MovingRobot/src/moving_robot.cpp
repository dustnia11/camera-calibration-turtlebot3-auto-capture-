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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#define _USE_MATH_DEFINES
#include <cmath>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

double goal_x=2;
double goal_y=2;
double goal_angle=atan2(goal_y,goal_x);

class MovingRobot : public rclcpp::Node
{
public:
	MovingRobot()
  : Node("moving_robot"), count_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1000);
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 1000, std::bind(&MovingRobot::topic_callback, this, _1));
  }
  
private:
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
  {
    double x=msg->pose.pose.orientation.x;
    double y=msg->pose.pose.orientation.y;
    double z=msg->pose.pose.orientation.z;
    double w=msg->pose.pose.orientation.w;
    
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    auto message = geometry_msgs::msg::Twist();
    if(abs(goal_x-msg->pose.pose.position.x>0.1)){
    	if (abs(goal_angle-yaw)>0.03){
	    message.linear.x=0;
	    message.angular.z=0.2;
        }
    	else {
	    message.linear.x=0.2;
	    message.angular.z=0;
        }
    }
    else {
	message.linear.x=0;
    }
    RCLCPP_INFO(this->get_logger(), "I heard: '%f'", yaw);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%f.2'", message.linear.x);
    publisher_->publish(message);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MovingRobot>());
  rclcpp::shutdown();
  return 0;
}
