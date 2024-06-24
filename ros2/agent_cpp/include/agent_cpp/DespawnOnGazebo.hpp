#ifndef DESPAWN_GZ_HH_
#define DESPAWN_GZ_HH_

#include "DespawnInterface.hpp"

using namespace std;

class DespawnOnGazebo : public DespawnInterface {
public:
    DespawnOnGazebo(
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

public:
    /// @brief Despawn command called on OnCleanup() or OnShutdown()
    /// @return Returns true if successfully despawned
    bool despawn() override
    {
        RCLCPP_DEBUG(
            rclcpp::get_logger("rclcpp"),
            "Destroying ros2 node of agent %s",
            request_.name);

        // Wait for the service to be activated
        while (!remove_entity_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(
                    rclcpp::get_logger("rclcpp"),
                    "Interrupted while waiting for the service. Exiting.");
                return false;
            }

            RCLCPP_INFO(
                rclcpp::get_logger("rclcpp"),
                "%s service not available, waiting again...",
                request_.name);
        }

        auto res = remove_entity_client_->async_send_request(
            make_shared<liquidai_msgs::srv::RemoveEntity::Request>(request_));

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
                    "The checks were not successful: %s",
                    "bruh");
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
    rclcpp::Client<liquidai_msgs::srv::RemoveEntity>::SharedPtr
        remove_entity_client_;
    liquidai_msgs::srv::RemoveEntity::Request request_;
    rclcpp::Node::SharedPtr node_;
};

#endif