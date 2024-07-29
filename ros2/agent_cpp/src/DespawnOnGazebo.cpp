#include "DespawnOnGazebo.hpp"

DespawnOnGazebo::DespawnOnGazebo(
    liquidai_msgs::srv::RemoveEntity::Request request,
    rclcpp::Node::SharedPtr node)
    : DespawnInterface()
{
    node_ = node;

    this->remove_entity_client_ =
        node_->create_client<liquidai_msgs::srv::RemoveEntity>(
            "/gz_remove_entity");

    request_ = request;
};

bool DespawnOnGazebo::despawn()
{
    RCLCPP_INFO(
        this->node_->get_logger(),
        "Destroying ros2 node of agent %s",
        request_.name.c_str());

    // Wait for the service to be activated
    while (!remove_entity_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(
                this->node_->get_logger(),
                "Interrupted while waiting for the service. Exiting.");
            return false;
        }

        RCLCPP_INFO(
            this->node_->get_logger(),
            "%s service not available, waiting again...",
            request_.name.c_str());
    }

    auto res = remove_entity_client_->async_send_request(
        make_shared<liquidai_msgs::srv::RemoveEntity::Request>(request_));

    if (rclcpp::spin_until_future_complete(node_, res) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        if (res.get()->result) {
            RCLCPP_INFO(
                this->node_->get_logger(), "The checks were successful!");
            return true;
        }
        else {
            RCLCPP_WARN(
                this->node_->get_logger(),
                "The checks were not successful: %s",
                "bruh");
            return false;
        }
    }
    else {
        RCLCPP_ERROR(
            this->node_->get_logger(), "Failed to call service 'checks'");
        return false;
    }
}