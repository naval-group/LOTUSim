#include "liquidai_msgs/srv/activate_slowdown.hpp"
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

std::string ENTITY_EVENTS_TOPIC = "/entity_events";

class SimulationControl : public rclcpp::Node {
public:
    SimulationControl(const rclcpp::NodeOptions &options)
        : rclcpp::Node("simulation_control_node", options)
    {
        change_state_client_node =
            rclcpp::Node::make_shared("change_state_client_node");

        SC_step_control_client_ =
            this->create_client<std_srvs::srv::SetBool>(
                "step_control_enable");
        activate_slowdown_publisher_ =
            this->create_publisher<std_msgs::msg::Bool>(
                "activate_slowdown", 10);
        this->SC_change_state_of_all =
            this->create_service<lifecycle_msgs::srv::ChangeState>(
                "SC_change_state_of_all",
                std::bind(
                    &SimulationControl::ChangeStateOfAll,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));
        this->SC_activate_slowdown =
            this->create_service<liquidai_msgs::srv::ActivateSlowdown>(
                "SC_activate_slowdown",
                std::bind(
                    &SimulationControl::ActivateSlowdown,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));
    }

    /// @brief Change state of all Lifecycle Node on the scene by transitioning
    /// them.
    /// @param request
    /// @param response
    void ChangeStateOfAll(
        const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request>
            request,
        std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response> response)
    {
        // Pause the simulation
        auto pause_request =
            std::make_shared<std_srvs::srv::SetBool::Request>();
        pause_request->data = true;
        auto pause_req_res =
            SC_step_control_client_->async_send_request(pause_request);

        // We fetch all lifecycle nodes. This is not optimal but it works.
        std::string command = "ros2 lifecycle nodes";
        std::string res = exec_command(command.data());
        RCLCPP_INFO(
            this->get_logger(),
            "Changing the state of the following agents: %s",
            res.c_str());

        // Split the string by newline
        std::string delimiter = "\n";
        size_t pos = 0;
        std::string token;
        std::vector<std::string> agent_names;
        while ((pos = res.find(delimiter)) != std::string::npos) {
            token = res.substr(0, pos);
            agent_names.push_back(token);
            res.erase(0, pos + delimiter.length());
        }
        int i = 1;
        int size = agent_names.size();
        int success = 0;
        // Change state of all lifecycle nodes agents
        for (auto agent_name : agent_names) {
            if (ChangeState(agent_name, request)) {
                success++;
            }

            RCLCPP_INFO(
                this->get_logger(),
                "%d out of %d agents have changed state to %d",
                i,
                size,
                request->transition.id);
            i++;
        }

        // UnPause the simulation
        pause_request->data = false;
        auto pause_req_res2 =
            SC_step_control_client_->async_send_request(pause_request);

        response->success = success == size;
    }

    bool ChangeState(
        std::string agent_name,
        const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request>
            request)
    {
        RCLCPP_DEBUG(
            this->get_logger(),
            "Changing the state of %s to %d",
            agent_name.c_str(),
            request->transition.id);

        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client =
            change_state_client_node
                ->create_client<lifecycle_msgs::srv::ChangeState>(
                    agent_name + "/change_state");

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
                agent_name.c_str());
        }

        // Client sends its asynchronous request
        auto res = client->async_send_request(request);

        // Wait for the result
        if (rclcpp::spin_until_future_complete(change_state_client_node, res) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            if (res.get()->success) {
                sleep(5);
                RCLCPP_INFO(this->get_logger(), "The checks were successful!");
                return true;
            }
            else {
                RCLCPP_WARN(
                    this->get_logger(), "The checks were not successful");
                return false;
            }
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service 'checks'");
            return false;
        }
    }

    void ActivateSlowdown(
        const std::shared_ptr<liquidai_msgs::srv::ActivateSlowdown::Request>
            request,
        std::shared_ptr<liquidai_msgs::srv::ActivateSlowdown::Response>
            response)
    {
        std_msgs::msg::Bool message;
        message.data = request->activation;
        activate_slowdown_publisher_->publish(message);

        response->success = true;
    }

private:
    rclcpp::Node::SharedPtr change_state_client_node;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr SC_step_control_client_;

    rclcpp::Service<lifecycle_msgs::srv::ChangeState>::SharedPtr
        SC_change_state_of_all;
    rclcpp::Service<liquidai_msgs::srv::ActivateSlowdown>::SharedPtr
        SC_activate_slowdown;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
        activate_slowdown_publisher_;

    std::string exec_command(const char *cmd)
    {
        char buffer[128];
        std::string result = "";
        FILE *pipe = popen(cmd, "r");
        if (!pipe)
            throw std::runtime_error("popen() failed!");
        try {
            while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
                result += buffer;
            }
        }
        catch (...) {
            pclose(pipe);
            throw;
        }
        pclose(pipe);
        return result;
    }
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(SimulationControl)