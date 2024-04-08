
#include "convert/utils.hpp"
#include "ros_gz_bridge/convert/builtin_interfaces.hpp"
#include "ros_gz_bridge/convert/liquidai_msgs.hpp"

namespace ros_gz_bridge {

template <>
void convert_ros_to_gz(
    const liquidai_msgs::msg::FloatStamped &ros_msg,
    ignition::msgs::Double &gz_msg)
{
    gz_msg.set_data(ros_msg.data);
}

template <>
void convert_gz_to_ros(
    const ignition::msgs::Double &gz_msg,
    liquidai_msgs::msg::FloatStamped &ros_msg)
{
    ros_msg.data = gz_msg.data();
}

} // namespace ros_gz_bridge
