#include "SensorInterface.hpp"

class MyIMUSensor : public SensorInterface {
public:
    MyIMUSensor(const rclcpp::NodeOptions &options);
    void fetchData() override;

    void sensorCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sensor_sub_;
    geometry_msgs::msg::Vector3 lin_acc_value;
};