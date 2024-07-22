#include "liquidai_msgs/msg/add_entity.hpp"
#include "liquidai_msgs/srv/add_entity_srv.hpp"
#include "liquidai_msgs/srv/add_entity_srv_array.hpp"
#include "liquidai_msgs/srv/remove_entity.hpp"
#include <chrono>
#include <cstdio>
#include <functional>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <liquidai_msgs/msg/entity_position.hpp>
#include <memory>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>

class AgentDemoCpp : public rclcpp::Node {
public:
    AgentDemoCpp(rclcpp::NodeOptions &options)
        : rclcpp::Node("agent_demo_cpp_node", options)
    {
        declare_parameter<int>("gazebo_id");
        get_parameter<int>("gazebo_id", gazebo_id);

        pose_pub_ = this->create_publisher<liquidai_msgs::msg::EntityPosition>(
            "/effector_demo_sched", 10);
        debug_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/agent_demo_sched" + std::to_string(gazebo_id), 10);

        sensor_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/sensor_demo_sched",
            10,
            std::bind(
                &AgentDemoCpp::sensor_callback, this, std::placeholders::_1));

        // timer_ = timer_ = rclcpp::create_timer(
        //     this,
        //     this->get_clock(),
        //     std::chrono::milliseconds(500),
        //     std::bind(&AgentDemoCpp::timer_callback, this));
    }

public:
    void timer_callback()
    {
        std_msgs::msg::Int32 dbg_msg;
        dbg_msg.data = 0;
        debug_pub_->publish(dbg_msg);

        auto message = liquidai_msgs::msg::EntityPosition();
        message.id = gazebo_id;
        x = x + 0.1;
        message.position.set__x(x);
        RCLCPP_INFO(
            this->get_logger(),
            "Publishing: '%s' on id %d",
            std::to_string(message.position.x).c_str(),
            gazebo_id);
        pose_pub_->publish(message);

        dbg_msg.data = gazebo_id;
        debug_pub_->publish(dbg_msg);
    }

    void sensor_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        std_msgs::msg::Int32 dbg_msg;
        dbg_msg.data = 0;
        debug_pub_->publish(dbg_msg);
        dbg_msg.data = gazebo_id;
        debug_pub_->publish(dbg_msg);

        RCLCPP_INFO(this->get_logger(), "Faking work on id %d", gazebo_id);
        get_clock()->sleep_for(std::chrono::milliseconds(400));
        auto message = liquidai_msgs::msg::EntityPosition();
        message.id = gazebo_id;
        x = x + 0.1;
        message.position.set__x(x);
        RCLCPP_INFO(
            this->get_logger(),
            "Publishing: '%s' on id %d",
            std::to_string(message.position.x).c_str(),
            gazebo_id);
        pose_pub_->publish(message);

        debug_pub_->publish(dbg_msg);
        dbg_msg.data = 0;
        debug_pub_->publish(dbg_msg);
    }

private:
    float x = 0;
    int gazebo_id;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<liquidai_msgs::msg::EntityPosition>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr debug_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sensor_sub_;
};

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    printf("hello world agent_demo_cpp package\n");

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<AgentDemoCpp>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
