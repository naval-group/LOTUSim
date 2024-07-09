#include <chrono>
#include <cstdio>
#include <functional>
#include <liquidai_msgs/msg/entity_position.hpp>
#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

class AgentDemoCpp : public rclcpp::Node {
public:
    AgentDemoCpp()
        : rclcpp::Node("agent_demo_cpp_node")
    {
        declare_parameter<int>("gazebo_id");
        get_parameter<int>("gazebo_id", gazebo_id);

        pose_pub_ = this->create_publisher<liquidai_msgs::msg::EntityPosition>(
            "/GazeboPosition", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&AgentDemoCpp::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = liquidai_msgs::msg::EntityPosition();
        message.id = gazebo_id;
        x = x + 0.1;
        message.position.set__x(x);
        RCLCPP_INFO(
            this->get_logger(),
            "Publishing: '%s' on id %d",
            std::to_string(message.position.x).c_str(), gazebo_id);
        pose_pub_->publish(message);
    }
    float x = 0;
    int gazebo_id;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<liquidai_msgs::msg::EntityPosition>::SharedPtr pose_pub_;
};

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    printf("hello world agent_demo_cpp package\n");

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AgentDemoCpp>());
    rclcpp::shutdown();
    return 0;
}
