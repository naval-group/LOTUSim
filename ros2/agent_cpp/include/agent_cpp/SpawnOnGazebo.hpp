#ifndef SPAWN_GZ_HH_
#define SPAWN_GZ_HH_

#include "SpawnInterface.hpp"

using namespace std;

class SpawnOnGazebo : public SpawnInterface {

public:
    SpawnOnGazebo(
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

    /// @brief Spawn command called on OnConfigure()
    /// @return Returns true if successfully spawned entity
    bool spawn() override
    {
        RCLCPP_DEBUG(
            rclcpp::get_logger("rclcpp"),
            "Creating ros2 node of agent %s",
            request_.data.name);

        // When send a create_multiple request through the gz_entity_management
        // package
        auto request_V = liquidai_msgs::srv::AddEntitySrvArray::Request();

        request_V.data.push_back(request_.data);

        // Wait for the service to be activated
        while (!this->add_entity_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(
                    rclcpp::get_logger("rclcpp"),
                    "Interrupted while waiting for the service. Exiting.");
                return false;
            }

            RCLCPP_INFO(
                rclcpp::get_logger("rclcpp"),
                "%s service not available, waiting again...",
                request_.data.name);
        }

        auto res = this->add_entity_client_->async_send_request(
            make_shared<liquidai_msgs::srv::AddEntitySrvArray::Request>(
                request_V));

        if (rclcpp::spin_until_future_complete(node_, res) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            // Get the response's success field to see if all checks passed
            if (res.get()->result) {
                RCLCPP_INFO(
                    rclcpp::get_logger("rclcpp"),
                    "The checks were successful!");
                return true;
            }
            else {
                RCLCPP_WARN(
                    rclcpp::get_logger("rclcpp"),
                    "The checks were not successful");
                return false;
            }
        }
        else {
            RCLCPP_ERROR(
                rclcpp::get_logger("rclcpp"),
                "Failed to call service 'checks'");
            return false;
        }
    }

private:
    rclcpp::Client<liquidai_msgs::srv::AddEntitySrvArray>::SharedPtr
        add_entity_client_;
    liquidai_msgs::srv::AddEntitySrv::Request request_;
    rclcpp::Node::SharedPtr node_;
};

#endif