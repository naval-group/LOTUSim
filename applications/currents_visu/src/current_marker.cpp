#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>


class CubeSubPub : public rclcpp::Node
{



public:
    CubeSubPub()
    : Node("cube_sub_pub"), int_id(0)
    {
        pose_subscription = this->create_subscription<geometry_msgs::msg::Pose>(
            "/cube_currents", 100, std::bind(&CubeSubPub::pose_callback, this, std::placeholders::_1));
        marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>("arrow_currents_marker", 100);
        
    }

    

private:
    int int_id; 

    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received current: [U:%.2f, V:%.2f, W:%.2f]", msg->orientation.x, msg->orientation.y, msg->orientation.z);
        
        
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "/my_frame";
        marker.header.stamp = this->get_clock()->now();
     
    // Convert the time_point to a duration in milliseconds (or any other desired unit)
      
    // Convert the duration to an integer
        marker.ns = "arrow_currents";
        marker.id =  int_id++;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = msg->position.x;
        marker.pose.position.y = msg->position.y;
        marker.pose.position.z = msg->position.z;
      	marker.pose.orientation.x = msg->orientation.x;
    	marker.pose.orientation.y = msg->orientation.y;
     	marker.pose.orientation.z = msg->orientation.z;


        marker.scale.x = 1;
        marker.scale.z = 0.2;
        marker.scale.y = 0.2;
        
        marker.color.a = 1.0;  // Alpha
        marker.color.r = 0.0;  // Red
        marker.color.g = 0.0;  // Green
        marker.color.b = 1.0;  // Blue
        
        marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

        marker_publisher->publish(marker);   
        
        RCLCPP_INFO(this->get_logger(), "Publishing corresponding arrow marker in RViz...");
        
    }
    
   
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CubeSubPub>());
    rclcpp::shutdown();
    return 0;
}

