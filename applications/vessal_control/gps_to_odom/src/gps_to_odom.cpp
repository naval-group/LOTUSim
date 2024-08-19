/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include "gps_tools/gps_to_odom.hpp"

namespace gps_tools {

UtmOdometryComponent::UtmOdometryComponent(const rclcpp::NodeOptions &options)
    : Node("utm_odometry_node", options)
    , fix_sub_(nullptr)
    , rot_cov_(99999.0)
    , append_zone_(false)
    , prev_yaw_(0)
    , prev_time_(rclcpp::Clock{}.now())
    , zero_origin_(true)
{
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    declare_parameter("append_zone", false);
    declare_parameter("odom_child_frame_id", "");
    declare_parameter("odom_parent_frame_id", "");
    declare_parameter("rot_covariance", 99999.0);

    // Needed to get the orientation of base_link to know the speed in
    // base_link
    declare_parameter("tf_child_link_frame_id", "");
    declare_parameter("tf_parent_link_frame_id", "");
    declare_parameter("zero_origin", true);

    get_parameter_or("rot_covariance", rot_cov_, rot_cov_);
    get_parameter_or(
        "odom_child_frame_id",
        odom_child_frame_id_,
        odom_child_frame_id_);
    get_parameter_or(
        "odom_parent_frame_id",
        odom_parent_frame_id_,
        odom_parent_frame_id_);
    get_parameter_or("append_zone", append_zone_, append_zone_);
    get_parameter_or(
        "tf_child_link_frame_id",
        tf_child_link_frame_id_,
        tf_child_link_frame_id_);
    get_parameter_or(
        "tf_parent_link_frame_id",
        tf_parent_link_frame_id_,
        tf_parent_link_frame_id_);
    get_parameter_or("zero_origin", zero_origin_, zero_origin_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    fix_sub_ = this->create_subscription<liquidai_msgs::msg::GPS>(
        "gps",
        10,
        std::bind(&UtmOdometryComponent::odomCB, this, std::placeholders::_1));
}

void UtmOdometryComponent::odomCB(const liquidai_msgs::msg::GPS::SharedPtr fix)
{
    if (fix->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
        RCLCPP_DEBUG(this->get_logger(), "No fix.");
        return;
    }

    if (fix->header.stamp.nanosec == 0 && fix->header.stamp.sec == 0) {
        return;
    }

    double northing, easting;
    std::string zone;

    LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

    if (odom_pub_) {
        nav_msgs::msg::Odometry odom;

        odom.header.stamp = fix->header.stamp;

        if (odom_parent_frame_id_.empty()) {
            if (append_zone_) {
                odom.header.frame_id = fix->header.frame_id + "/utm_" + zone;
            } else {
                odom.header.frame_id = fix->header.frame_id;
            }
        } else {
            if (append_zone_) {
                odom.header.frame_id = odom_parent_frame_id_ + "/utm_" + zone;
            } else {
                odom.header.frame_id = odom_parent_frame_id_;
            }
        }

        odom.child_frame_id = odom_child_frame_id_;

        if (zero_origin_) {
            origin_x_ = easting;
            origin_y_ = northing;
            zero_origin_ = false;
        }

        odom.pose.pose.position.x = easting - origin_x_;
        odom.pose.pose.position.y = northing - origin_y_;
        odom.pose.pose.position.z = fix->altitude;

        odom.pose.pose.orientation.x = 0;
        odom.pose.pose.orientation.y = 0;
        odom.pose.pose.orientation.z = 0;
        odom.pose.pose.orientation.w = 1;

        // Use ENU covariance to build XYZRPY covariance
        std::array<double, 36> covariance = {
            {fix->position_covariance[0],
             fix->position_covariance[1],
             fix->position_covariance[2],
             0,
             0,
             0,
             fix->position_covariance[3],
             fix->position_covariance[4],
             fix->position_covariance[5],
             0,
             0,
             0,
             fix->position_covariance[6],
             fix->position_covariance[7],
             fix->position_covariance[8],
             0,
             0,
             0,
             0,
             0,
             0,
             rot_cov_,
             0,
             0,
             0,
             0,
             0,
             0,
             rot_cov_,
             0,
             0,
             0,
             0,
             0,
             0,
             rot_cov_}};

        odom.pose.covariance = covariance;

        // TODO: To be removed. Cheat method to populate orientation
        // and angular velocity through tf.
        // Getting velocity relative to base_link.
        // Preferably from ekf of sensors and stuff
        if (!tf_child_link_frame_id_.empty() &&
            !tf_parent_link_frame_id_.empty()) {
            tf2::Vector3 vel_tf(
                fix->linear_velocity.x,
                fix->linear_velocity.y,
                fix->linear_velocity.z);

            geometry_msgs::msg::TransformStamped transformStamped;
            try {
                transformStamped = tf_buffer_->lookupTransform(
                    tf_parent_link_frame_id_,
                    tf_child_link_frame_id_,
                    tf2::TimePointZero);

                odom.pose.pose.orientation =
                    transformStamped.transform.rotation;

                tf2::Quaternion quat_tf(
                    transformStamped.transform.rotation.x,
                    transformStamped.transform.rotation.y,
                    transformStamped.transform.rotation.z,
                    transformStamped.transform.rotation.w);

                double yaw = tf2::getYaw<tf2::Quaternion>(quat_tf);
                rclcpp::Time now = rclcpp::Clock{}.now();
                odom.twist.twist.angular.z =
                    (yaw - prev_yaw_) / (now - prev_time_).seconds();
                prev_time_ = now;
                prev_yaw_ = yaw;

                double yawt, pitcht, rollt;
                tf2::getEulerYPR<tf2::Quaternion>(quat_tf, yawt, pitcht, rollt);
                RCLCPP_DEBUG(
                    this->get_logger(),
                    "Transform %s->%s %f,%f,%f %f,%f,%f",
                    tf_parent_link_frame_id_.c_str(),
                    tf_child_link_frame_id_.c_str(),
                    transformStamped.transform.translation.x,
                    transformStamped.transform.translation.y,
                    transformStamped.transform.translation.z,
                    pitcht,
                    rollt,
                    yawt);

                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = fix->header.stamp;
                t.header.frame_id = odom_parent_frame_id_;
                t.child_frame_id = odom_child_frame_id_;
                t.transform.translation.x = odom.pose.pose.position.x;
                t.transform.translation.y = odom.pose.pose.position.y;
                t.transform.translation.z = odom.pose.pose.position.z;
                t.transform.rotation = transformStamped.transform.rotation;
                tf_broadcaster_->sendTransform(t);

                auto new_vec = tf2::quatRotate(quat_tf.inverse(), vel_tf);
                odom.twist.twist.linear.x = new_vec.getX();
                odom.twist.twist.linear.y = new_vec.getY();
                odom.twist.twist.linear.z = new_vec.getZ();

                RCLCPP_DEBUG(
                    this->get_logger(),
                    "Vel gps: %f,%f,%f Vel base: %f,%f,%f",
                    vel_tf.getX(),
                    vel_tf.getY(),
                    vel_tf.getZ(),
                    new_vec.getX(),
                    new_vec.getY(),
                    new_vec.getZ());
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Could not transform %s->%s",
                    tf_child_link_frame_id_.c_str(),
                    tf_parent_link_frame_id_.c_str());
            }
        }
        odom_pub_->publish(odom);
    }
}

}  // namespace gps_tools
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(gps_tools::UtmOdometryComponent)
