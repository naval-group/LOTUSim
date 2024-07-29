#include "SpawnOnGazebo.hpp"

SpawnOnGazebo::SpawnOnGazebo(
    liquidai_msgs::srv::AddEntitySrv::Request request,
    rclcpp::Node::SharedPtr node)
    : SpawnInterface()
{
    node_ = node;

    this->add_entity_client_ =
        node_->create_client<liquidai_msgs::srv::AddEntitySrvArray>(
            "/gz_add_entity_V");

    request_ = request;
};

bool SpawnOnGazebo::spawn()
{
    RCLCPP_DEBUG(
        this->node_->get_logger(),
        "Creating ros2 node of agent %s",
        request_.data.name.c_str());

    // We send a create_multiple request through the gz_entity_management
    // package
    auto request_V = liquidai_msgs::srv::AddEntitySrvArray::Request();

    request_V.data.push_back(request_.data);

    // Wait for the service to be activated
    while (!this->add_entity_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(
                this->node_->get_logger(),
                "Interrupted while waiting for the service. Exiting.");
            return false;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "%s service not available, waiting again...",
            request_.data.name.c_str());
    }

    auto res = this->add_entity_client_->async_send_request(
        make_shared<liquidai_msgs::srv::AddEntitySrvArray::Request>(request_V));

    if (rclcpp::spin_until_future_complete(node_, res) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        // Get the response's success field to see if all checks passed
        if (res.get()->result) {
            RCLCPP_INFO(
                this->node_->get_logger(), "The checks were successful!");
            return true;
        }
        else {
            RCLCPP_WARN(
                this->node_->get_logger(), "The checks were not successful");
            return false;
        }
    }
    else {
        RCLCPP_ERROR(
            this->node_->get_logger(), "Failed to call service 'checks'");
        return false;
    }
}