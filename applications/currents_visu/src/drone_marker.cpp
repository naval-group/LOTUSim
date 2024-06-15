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
#include <ament_index_cpp/get_package_share_directory.hpp>

class DroneSubPub : public rclcpp::Node
{



public:
    DroneSubPub()
    : Node("drone_sub_pub")
    {
        pose_drone_sub = this->create_subscription<geometry_msgs::msg::Pose>(
            "/model/tethys/pose", 100, std::bind(&DroneSubPub::pose_callback, this, std::placeholders::_1));
        marker_drone_pub = this->create_publisher<visualization_msgs::msg::Marker>("drone_marker", 100);
        
    }

    
private:
    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received pose: [%.2f, %.2f, %.2f]", msg->position.x, msg->position.y, msg->position.z);
        
   
        
        visualization_msgs::msg::Marker marker;
        // Set the frame ID and timestamp. See the TF tutorials for information on these.
        marker.header.frame_id = "/my_frame";
        marker.header.stamp = rclcpp::Clock().now();

        // Set the namespace and id for this marker. This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "basic_shapes";
        marker.id = 0;

        // Set the marker type
        // Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        // Set the marker action
        // Options are ADD, DELETE, and DELETEALL
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = msg->position.x;
        marker.pose.position.y = msg->position.y;
        marker.pose.position.z = msg->position.z;

     	
     	// Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 3;
        marker.scale.z = 3;
        marker.scale.y = 3;

        // Set the color -- be sure to set alpha to something non-zero!
        


        marker.color.r = 1;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("currents_visu");
        marker.mesh_resource = "file://" + package_share_directory + "/meshes/tethys.dae";
        marker.mesh_use_embedded_materials = true;



        
    // Set the lifetime of the marker -- 0 indicates forever
        marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

        

     

    // Puberlish the marker
       marker_drone_pub->publish(marker);
        
        RCLCPP_INFO(this->get_logger(), "Publishing corresponding object marker in RViz...");
        
        
        
        
    }
    
   
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_drone_sub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_drone_pub;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneSubPub>());
    rclcpp::shutdown();
    return 0;
}

