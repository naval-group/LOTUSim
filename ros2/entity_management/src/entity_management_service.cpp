#include "liquidai_msgs/srv/add_entity.hpp"
#include "liquidai_msgs/srv/remove_entity.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdlib>
#include <iostream>
#include <rclcpp/client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <stdexcept>
#include <string>
#include <thread>

namespace entity_management_service {
std::string SERVICE_NODE_NAME = "add_entity_service_node";
std::string ADD_SERVICE_NAME = "add_entity";
std::string REMOVE_SERVICE_NAME = "remove_entity";
std::string ENTITY_EVENTS_TOPIC = "/entity_events";

class EntityManagementService : public rclcpp::Node {
public:
    EntityManagementService(std::string name)
        : rclcpp::Node(name)
    {
        this->ros_entity_events_publisher_ =
            this->create_publisher<std_msgs::msg::Int32>(
                ENTITY_EVENTS_TOPIC, 10);

        this->add_service = this->create_service<liquidai_msgs::srv::AddEntity>(
            ADD_SERVICE_NAME,
            std::bind(
                &EntityManagementService::add_entity,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

        this->remove_service =
            this->create_service<liquidai_msgs::srv::RemoveEntity>(
                REMOVE_SERVICE_NAME,
                std::bind(
                    &EntityManagementService::remove_entity,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

        this->step_control_client_node_ =
            rclcpp::Node::make_shared("step_control_client_node");

        this->step_control_client_ =
            step_control_client_node_->create_client<std_srvs::srv::SetBool>(
                "step_control_enable");
    }

    void add_entity(
        const std::shared_ptr<liquidai_msgs::srv::AddEntity::Request> request,
        std::shared_ptr<liquidai_msgs::srv::AddEntity::Response> response)
    {
        std::string full_model_filepath =
            ament_index_cpp::get_package_share_directory("assets") + "/" +
            request->model_filepath;

        auto message = std_msgs::msg::Int32();
        message.data = 0;
        ros_entity_events_publisher_->publish(message);

        // Pause the simulation
        auto pause_request =
            std::make_shared<std_srvs::srv::SetBool::Request>();
        pause_request->data = true;

        // Wait for the service to be activated
        while (
            !step_control_client_->wait_for_service(std::chrono::seconds(1))) {
            // If ROS is shutdown before the service is activated, show this
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(
                    rclcpp::get_logger("rclcpp"),
                    "Interrupted while waiting for the service. Exiting.");
                return;
            }

            RCLCPP_INFO(
                rclcpp::get_logger("rclcpp"),
                "Service not available, waiting again...");
        }

        auto result = step_control_client_->async_send_request(pause_request);

        if (rclcpp::spin_until_future_complete(
                step_control_client_node_, result) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(
                rclcpp::get_logger("rclcpp"),
                "Failed to call service 'step_control_enable'");
            return;
        }

        createBridge(
            request->name + "/sched",
            "std_msgs/msg/Int32",
            "gz.msgs.Int32",
            "[");

        // TODO: Implement the service call through ros_gz_bridge once it is
        // implemented

        // Construct the spawn command
        std::ostringstream command_stream;
        command_stream << "gz service -s /world/demo/create"
                       << " --reqtype gz.msgs.EntityFactory"
                       << " --reptype gz.msgs.Boolean"
                       << " --timeout 5000"
                       << " --req"
                       << " 'name: \"" << request->name.c_str()
                       << "\"; sdf_filename: \"" << full_model_filepath.c_str()
                       << "\"; pose: {position: {x: " << request->location.x
                       << ", y: " << request->location.y
                       << ", z: " << request->location.z
                       << "}, orientation: {x: " << request->rotation.x
                       << ", y: " << request->rotation.y
                       << ", z: " << request->rotation.z << "}}'";

        std::string res = this->exec_command(command_stream.str().data());

        RCLCPP_INFO(
            this->get_logger(), "/world/demo/create output is %s", res.c_str());
        response->result = res.find("true") != std::string::npos;
        // response->result = false;

        // UnPause the simulation
        pause_request->data = false;

        auto result2 = step_control_client_->async_send_request(pause_request);

        if (rclcpp::spin_until_future_complete(
                step_control_client_node_, result2) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(
                rclcpp::get_logger("rclcpp"),
                "Failed to call service 'step_control_enable'");
            return;
        }
    }

    void remove_entity(
        const std::shared_ptr<liquidai_msgs::srv::RemoveEntity::Request>
            request,
        std::shared_ptr<liquidai_msgs::srv::RemoveEntity::Response> response)
    {

        auto message = std_msgs::msg::Int32();
        message.data = 0;
        ros_entity_events_publisher_->publish(message);

        // TODO: Implement the service call through ros_gz_bridge once it is
        // implemented

        // Construct the remove command
        std::ostringstream command_stream;
        command_stream << "gz service -s /world/demo/remove"
                       << " --reqtype gz.msgs.Entity"
                       << " --reptype gz.msgs.Boolean"
                       << " --timeout 5000"
                       << " --req"
                       << " 'type: MODEL; name: \"" << request->name << "\"'";

        std::string res = this->exec_command(command_stream.str().data());

        RCLCPP_INFO(
            this->get_logger(), "/world/demo/remove output is %s", res.c_str());
        response->result = res.find("true") != std::string::npos;
        // response->result = false;
    }

    void createBridge(
        std::string topic_name,
        std::string ros_type_name,
        std::string gz_type_name,
        std::string direction = "@")
    {
        std::ostringstream command_stream;
        command_stream << "ros2 run ros_gz_bridge parameter_bridge "
                       << topic_name << "@" << ros_type_name << direction
                       << gz_type_name;
        std::string command = command_stream.str();
        std::thread cmdThread(
            [this, command]() { runCommand(command.data()); });
        cmdThread.detach(); // Detach the thread to run it independently
    }

private:
    rclcpp::Service<liquidai_msgs::srv::AddEntity>::SharedPtr add_service;
    rclcpp::Service<liquidai_msgs::srv::RemoveEntity>::SharedPtr remove_service;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr step_control_client_;

    rclcpp::Node::SharedPtr step_control_client_node_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr
        ros_entity_events_publisher_;

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

    int runCommand(const char *command)
    {
        // Run the command
        int result = system(command);

        return result;
    }
};
} // namespace entity_management_service

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<entity_management_service::EntityManagementService>(
            entity_management_service::SERVICE_NODE_NAME);

    rclcpp::spin(node);
    rclcpp::shutdown();
}
