#ifndef GPS_TO_ODOM_HPP
#define GPS_TO_ODOM_HPP

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <gps_tools/conversions.h>
#include <liquidai_msgs/msg/gps.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

namespace gps_tools {

class UtmOdometryComponent : public rclcpp::Node {
public:
    UtmOdometryComponent(const rclcpp::NodeOptions &options);

private:
    void odomCB(const liquidai_msgs::msg::GPS::SharedPtr fix);

private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<liquidai_msgs::msg::GPS>::SharedPtr fix_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::string tf_child_link_frame_id_;
    std::string tf_parent_link_frame_id_;
    std::string odom_parent_frame_id_;
    std::string odom_child_frame_id_;
    double rot_cov_;
    bool append_zone_;
    double prev_yaw_;
    rclcpp::Time prev_time_;

    bool zero_origin_;
    float origin_x_, origin_y_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

} // namespace gps_tools
#endif // GPS_TO_ODOM_HPP