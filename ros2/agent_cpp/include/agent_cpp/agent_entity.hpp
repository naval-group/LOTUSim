#ifndef AGENT_ENTITY_HH_
#define AGENT_ENTITY_HH_

#include "agent.hpp"

// An agent that is an entity in Gazebo
class AgentEntity : public Agent
{
private:
    string sdf_file_;
    string pose_str;
    string spawn_on_startup_;
    std::shared_ptr<rclcpp::Node> entity_management_client_node;

public:
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

    AgentEntity(const rclcpp::NodeOptions &options)
        : Agent("agent_entity", options)
    {
        get_parameter("sdf_file", sdf_file_);
        get_parameter("pose", pose_str);

        // // Client for entity management; not ideal for it to be in constructor, but useful
        // entity_management_client_node = rclcpp::Node::make_shared("entity_management_client_node");

        // // Spawn entity on Gazebo at startup, may be unstable
        // spawn();
    };

    bool spawn();

    bool despawn();
};

#endif