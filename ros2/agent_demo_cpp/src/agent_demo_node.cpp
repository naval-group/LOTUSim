#include "liquidai_msgs/msg/add_entity.hpp"
#include "liquidai_msgs/srv/add_entity_srv.hpp"
#include "liquidai_msgs/srv/add_entity_srv_array.hpp"
#include "liquidai_msgs/srv/get_id_by_name.hpp"
#include "liquidai_msgs/srv/remove_entity.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <liquidai_msgs/msg/entity_position.hpp>
#include <std_msgs/msg/int32.hpp>

#include <rclcpp/create_timer.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>

class AgentDemoCpp : public rclcpp::Node {
public:
    AgentDemoCpp(rclcpp::NodeOptions &options)
        : rclcpp::Node("agent_demo_cpp_node", options)
        , gazebo_id(0)
    {
        // declare_parameter<int>("gazebo_id");
        // get_parameter<int>("gazebo_id", gazebo_id);

        node_base_interface_ = this->get_node_base_interface();
        name_ = this->get_name();

        entity_management_client_node_ = std::make_shared<rclcpp::Node>("entity_management_node_");
        get_id_by_name_client_ =
            entity_management_client_node_->create_client<liquidai_msgs::srv::GetIdByName>(
                "/gz_get_id_by_name");

        while (!get_gazebo_id()) {
            RCLCPP_INFO(
                this->get_logger(), "Could not get Gazebo Id, trying again...");
        }

        pose_pub_ = this->create_publisher<liquidai_msgs::msg::EntityPosition>(
            "/pose_effector", 10);
        debug_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/agent_demo_sched" + std::to_string(gazebo_id), 10);
        gz_poses_sub_ =
            this->create_subscription<geometry_msgs::msg::PoseArray>(
                "/model/" + name_ + "/pose",
                10,
                std::bind(
                    &AgentDemoCpp::gz_pose_callback,
                    this,
                    std::placeholders::_1));
        sensor_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/pose_sensor",
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
        message.position.set__x(pose_.position.x + 0.1);
        message.position.set__y(pose_.position.y);
        message.position.set__z(pose_.position.z);
        RCLCPP_INFO(
            this->get_logger(),
            "Publishing: '%s' on id %d",
            std::to_string(message.position.x).c_str(),
            gazebo_id);
        pose_pub_->publish(message);

        dbg_msg.data = gazebo_id;
        debug_pub_->publish(dbg_msg);
    }

    bool get_gazebo_id()
    {
        while (!get_id_by_name_client_->wait_for_service(
            std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Interrupted while waiting for the service. Exiting.");
                exit(0);
            }
            RCLCPP_INFO(
                this->get_logger(), "Service not available, waiting again...");
        }

        auto request =
            std::make_shared<liquidai_msgs::srv::GetIdByName::Request>();
        request->set__entity_name(name_);
        auto res = get_id_by_name_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(entity_management_client_node_, res) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            int id = res.get()->id;
            if (id != 0) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Successfully got and set the ID %d from Gazebo!",
                    id);
                gazebo_id = id;
                return true;
            }
            else {
                RCLCPP_WARN(
                    this->get_logger(), "The Gazebo Id I got is 0.");
                return false;
            }
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service");
            return false;
        }
    }

    void sensor_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (gazebo_id == 0) {
            return;
        }

        std_msgs::msg::Int32 dbg_msg;
        dbg_msg.data = 0;
        debug_pub_->publish(dbg_msg);
        dbg_msg.data = gazebo_id;
        debug_pub_->publish(dbg_msg);

        RCLCPP_INFO(this->get_logger(), "Faking work on id %d", gazebo_id);
        get_clock()->sleep_for(std::chrono::milliseconds(350));
        auto message = liquidai_msgs::msg::EntityPosition();
        message.id = gazebo_id;
        message.position.set__x(pose_.position.x + 0.1);
        message.position.set__y(pose_.position.y);
        message.position.set__z(pose_.position.z);
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

    /// @brief Get the pose from Gazebo. It is sent by the Odometry system
    /// plugin.
    /// @param msg
    void gz_pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        pose_ = msg->poses.back();
    }

private:
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
    rclcpp::Node::SharedPtr entity_management_client_node_;
    geometry_msgs::msg::Pose pose_;
    std::string name_;
    int gazebo_id;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<liquidai_msgs::msg::EntityPosition>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr debug_pub_;
    rclcpp::Client<liquidai_msgs::srv::GetIdByName>::SharedPtr
        get_id_by_name_client_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sensor_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr
        gz_poses_sub_;
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
