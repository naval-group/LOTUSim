#ifndef AGENT_ENTITY_HH_
#define AGENT_ENTITY_HH_

#include "agent.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sdf/sdf.hh>
#include "liquidai_msgs/srv/get_id_by_name.hpp"

// An agent that is an entity in Gazebo
class AgentEntity : public Agent {
private:
    string sdf_file_;
    string sdf_filename_;
    string pose_str;
    bool configure_on_startup;
    rclcpp::Node::SharedPtr entity_management_client_node;
    rclcpp::Client<liquidai_msgs::srv::GetIdByName>::SharedPtr get_id_by_name_client_;
    int gazebo_id;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    AgentEntity(const rclcpp::NodeOptions &options);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

    void timer_callback();
};

#endif