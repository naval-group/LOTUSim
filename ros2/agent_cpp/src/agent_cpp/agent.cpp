#include "agent.hpp"

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Agent::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
    // TODO: Unload the component?
}