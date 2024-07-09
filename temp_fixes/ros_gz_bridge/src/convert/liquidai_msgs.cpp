
#include <limits>

#include "convert/utils.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "gz/msgs/pose.pb.h"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "ros_gz_bridge/convert/liquidai_msgs.hpp"

namespace ros_gz_bridge {
template <>
void convert_ros_to_gz(
    const liquidai_msgs::msg::GPS &ros_msg, gz::msgs::NavSat &gz_msg)
{
    convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
    gz_msg.set_latitude_deg(ros_msg.latitude);
    gz_msg.set_longitude_deg(ros_msg.longitude);
    gz_msg.set_altitude(ros_msg.altitude);
    gz_msg.set_frame_id(ros_msg.header.frame_id);

    // Not supported in sensor_msgs::NavSatFix.
    gz_msg.set_velocity_east(ros_msg.linear_velocity.x);
    gz_msg.set_velocity_north(ros_msg.linear_velocity.y);
    gz_msg.set_velocity_up(ros_msg.linear_velocity.z);
}

template <>
void convert_gz_to_ros(
    const gz::msgs::NavSat &gz_msg, liquidai_msgs::msg::GPS &ros_msg)
{
    convert_gz_to_ros(gz_msg.header(), ros_msg.header);
    ros_msg.header.frame_id = frame_id_gz_to_ros(gz_msg.frame_id());
    ros_msg.latitude = gz_msg.latitude_deg();
    ros_msg.longitude = gz_msg.longitude_deg();
    ros_msg.altitude = gz_msg.altitude();

    geometry_msgs::msg::Vector3 vel;
    vel.x = gz_msg.velocity_east();
    vel.y = gz_msg.velocity_north();
    vel.z = gz_msg.velocity_up();
    ros_msg.linear_velocity = vel;

    // position_covariance is not supported in gz::msgs::NavSat.
    ros_msg.position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    ros_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
}

template <>
void convert_ros_to_gz(
    const liquidai_msgs::msg::AISArray &ros_msg,
    gz_liquidai_msgs::msgs::AISArray &gz_msg)
{
    for (auto &r_msg : ros_msg.data) {
        gz_liquidai_msgs::msgs::AISArray_AIS *g_msg = gz_msg.add_data();
        g_msg->set_user_id(r_msg.user_id);
        g_msg->set_longitude(r_msg.longitude);
        g_msg->set_latitude(r_msg.latitude);
        g_msg->set_sog(r_msg.sog);
        g_msg->set_name(r_msg.name);
        // g_msg->set_true_heading(
        //     coord_opt.value().HeadingOffset().Degree());

        // Filler
        g_msg->set_navigation_status(r_msg.navigation_status);
        g_msg->set_rot(r_msg.rot);
        g_msg->set_positional_accuracy(r_msg.positional_accuracy);
        g_msg->set_true_heading(r_msg.true_heading);
        g_msg->set_cog(r_msg.cog);
    }
}

template <>
void convert_gz_to_ros(
    const gz_liquidai_msgs::msgs::AISArray &gz_msg,
    liquidai_msgs::msg::AISArray &ros_msg)
{
    for (auto &&g_msg : gz_msg.data()) {
        liquidai_msgs::msg::AIS r_msg;
        r_msg.user_id = g_msg.user_id();
        r_msg.name = g_msg.name();
        r_msg.navigation_status = g_msg.navigation_status();
        r_msg.rot = g_msg.rot();
        r_msg.sog = g_msg.sog();
        r_msg.positional_accuracy = g_msg.positional_accuracy();
        r_msg.longitude = g_msg.longitude();
        r_msg.latitude = g_msg.latitude();
        r_msg.true_heading = g_msg.true_heading();
        r_msg.cog = g_msg.cog();
        ros_msg.data.push_back(r_msg);
    }
}

template <>
void convert_ros_to_gz(
    const liquidai_msgs::msg::FloatStamped &ros_msg, gz::msgs::Double &gz_msg)
{
    convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
    gz_msg.set_data(ros_msg.data);
}

template <>
void convert_gz_to_ros(
    const gz::msgs::Double &gz_msg, liquidai_msgs::msg::FloatStamped &ros_msg)
{
    convert_gz_to_ros(gz_msg.header(), ros_msg.header);
    ros_msg.data = gz_msg.data();
}

template <>
void convert_ros_to_gz(
    const liquidai_msgs::msg::Xdyncmd &ros_msg,
    gz_liquidai_msgs::msgs::XdynCmd &gz_msg)
{
    gz_msg.set_name(ros_msg.vessel_name);
    for (auto &r_msg : ros_msg.cmd) {
        gz_liquidai_msgs::msgs::XdynCmd_ThrusterCmd *g_msg = gz_msg.add_cmd();
        g_msg->set_name(r_msg.name);
        g_msg->set_rpm(r_msg.rpm);
        g_msg->set_pd(r_msg.pd);
        g_msg->set_beta(r_msg.beta);
    }
}

template <>
void convert_gz_to_ros(
    const gz_liquidai_msgs::msgs::XdynCmd &gz_msg,
    liquidai_msgs::msg::Xdyncmd &ros_msg)
{
    ros_msg.vessel_name = gz_msg.name();
    for (auto &&g_msg : gz_msg.cmd()) {
        liquidai_msgs::msg::XdynThrustercmd r_msg;
        r_msg.name = g_msg.name();
        r_msg.rpm = g_msg.rpm();
        r_msg.pd = g_msg.pd();
        r_msg.beta = g_msg.beta();
        ros_msg.cmd.push_back(r_msg);
    }
}

template <>
void convert_ros_to_gz(
    const liquidai_msgs::msg::EntityPosition &ros_msg, gz::msgs::Pose &gz_msg)
{
    gz_msg.set_id(ros_msg.id);
    convert_ros_to_gz(ros_msg.position, *gz_msg.mutable_position());
    convert_ros_to_gz(ros_msg.rotation, *gz_msg.mutable_orientation());
}

template <>
void convert_gz_to_ros(
    const gz::msgs::Pose &gz_msg, liquidai_msgs::msg::EntityPosition &ros_msg)
{
    ros_msg.set__id(gz_msg.id());
    convert_gz_to_ros(gz_msg.position(), ros_msg.position);
    convert_gz_to_ros(gz_msg.orientation(), ros_msg.rotation);
}
} // namespace ros_gz_bridge