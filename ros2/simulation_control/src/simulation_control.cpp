#include "simulation_control.hpp"

std::string ENTITY_EVENTS_TOPIC = "/entity_events";

SimulationControl::SimulationControl(const rclcpp::NodeOptions &options)
    : rclcpp::Node("simulation_control_node", options)
{
    change_state_client_node =
        rclcpp::Node::make_shared("change_state_client_node");

    SC_step_control_client_ =
        this->create_client<std_srvs::srv::SetBool>("step_control_enable");
    this->SC_change_state_of_all =
        this->create_service<lifecycle_msgs::srv::ChangeState>(
            "SC_change_state_of_all",
            std::bind(
                &SimulationControl::ChangeStateOfAll,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
}

/// @brief Lists all the existing Lifecycle Nodes and tries to change their
/// state. It also pauses the simulation.
/// @param request
/// @param response
void SimulationControl::ChangeStateOfAll(
    const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> request,
    std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response> response)
{
    // Pause the simulation
    auto pause_request = std::make_shared<std_srvs::srv::SetBool::Request>();
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

    // Change state of all lifecycle nodes agents
    int i = 1;
    int size = agent_names.size();
    int success = 0;
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

    // It's a success if all ChangeState calls returned true
    response->success = success == size;
}

/// @brief Change the state of one Lifecycle Node based on its name
/// @param agent_name
/// @param request
/// @return
bool SimulationControl::ChangeState(
    std::string agent_name,
    const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> request)
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

    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_INFO(
            this->get_logger(),
            "%s service not available, waiting again...",
            agent_name.c_str());
    }

    auto res = client->async_send_request(request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(change_state_client_node, res) ==
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

/// @brief Execute a system command and return the output
/// @param cmd 
/// @return 
std::string SimulationControl::exec_command(const char *cmd)
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

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(SimulationControl)