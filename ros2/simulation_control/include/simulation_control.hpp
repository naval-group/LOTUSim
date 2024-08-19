#include "liquidai_msgs/srv/add_entity_srv.hpp"
#include "liquidai_msgs/srv/empty_srv.hpp"
#include "liquidai_msgs/srv/remove_entity.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdlib>
#include <iostream>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>


class SimulationControl : public rclcpp::Node {
public:
    SimulationControl(const rclcpp::NodeOptions &options);

    void ChangeStateOfAll(
        const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request>
            request,
        std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response> response);

    bool ChangeState(
        std::string agent_name,
        const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request>
            request);

private:
    rclcpp::Node::SharedPtr change_state_client_node;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr SC_step_control_client_;
    rclcpp::Service<lifecycle_msgs::srv::ChangeState>::SharedPtr
        SC_change_state_of_all;

    std::string exec_command(const char *cmd);
};