#include "MyIMUSensor.hpp"

MyIMUSensor::MyIMUSensor(const rclcpp::NodeOptions &options)
    : SensorInterface("imu_sensor_node", options)
{
    sensor_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "bruhgudrhguigrhui",
        1,
        std::bind(&MyIMUSensor::sensorCallback, this, std::placeholders::_1));
}

void MyIMUSensor::sensorCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    lin_acc_value = msg->linear_acceleration;
}

void MyIMUSensor::fetchData()
{
    RCLCPP_INFO(this->get_logger(), std::to_string(lin_acc_value.x).c_str());
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(MyIMUSensor)