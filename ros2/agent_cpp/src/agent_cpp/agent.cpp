#include "agent.hpp"

Agent::Agent(const string &node_name, const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode(node_name, options)
{
    // We need to declare all parameters here
    declare_parameter("sdf_file", "");
    declare_parameter("sdf_filename", "");
    declare_parameter("pose", "");
    declare_parameter<bool>("configure_on_startup");

    node_base_interface_ = this->get_node_base_interface();
    name_ = this->get_name();

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

/// @brief Change the Lifecycle state of itself, because methods such as on_configure cannot be directly called.
/// @param transition Set id to: 1 to configure, 2 to cleanup, 3 to activate, 4 to deactivate, 5 to shutdown
/// @return true if transition successful
bool Agent::self_change_state(lifecycle_msgs::msg::Transition transition)
{
    auto request = lifecycle_msgs::srv::ChangeState::Request();
    request.set__transition(transition);

    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client =
        this->create_client<lifecycle_msgs::srv::ChangeState>(
            name_ + "/change_state");

    // Wait for the service to be activated
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        // Print in the screen some information so the user knows what is
        // happening
        RCLCPP_INFO(
            this->get_logger(),
            "%s service not available, waiting again...",
            name_.c_str());
    }

    // Client sends its asynchronous request
    auto res = client->async_send_request(
        std::make_shared<lifecycle_msgs::srv::ChangeState::Request>(request));

    // Wait for the result
    if (rclcpp::spin_until_future_complete(node_base_interface_, res) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        if (res.get()->success) {
            RCLCPP_INFO(this->get_logger(), "The checks were successful!");
            return true;
        }
        else {
            RCLCPP_WARN(this->get_logger(), "The checks were not successful");
            return false;
        }
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service 'checks'");
        return false;
    }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Agent::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
    // TODO: Unload the component?
}