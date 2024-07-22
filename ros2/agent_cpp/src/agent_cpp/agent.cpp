#include "agent.hpp"

Agent::Agent(const string &node_name, const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode(node_name, options)
{
    // We need to declare all parameters here
    declare_parameter("sdf_file", "");
    declare_parameter("sdf_filename", "");
    declare_parameter("pose", "");
    declare_parameter<bool>("configure_on_startup");

    // TODO: make sure this works
    auto param_change_callback =
        [this](std::vector<rclcpp::Parameter> parameters)
        -> rcl_interfaces::msg::SetParametersResult {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;
        for (auto parameter : parameters) {
            RCLCPP_INFO(
                this->get_logger(),
                "parameter '%s' is now: %s",
                parameter.get_name().c_str(),
                parameter.value_to_string().c_str());
        }
        return result;
    };

    param_change_callback_handle_ =
        this->add_on_set_parameters_callback(param_change_callback);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Agent::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
    // TODO: Unload the component?
}