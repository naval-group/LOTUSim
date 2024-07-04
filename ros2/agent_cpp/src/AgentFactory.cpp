#include "Scheduler.hpp"
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_packages_with_prefixes.hpp>
#include <ament_index_cpp/get_resource.hpp>
#include <ament_index_cpp/has_resource.hpp>
#include <chrono>
#include <composition_interfaces/srv/load_node.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>
#include <string>
#include <sys/resource.h>
#include <vector>

namespace fs = std::filesystem;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    for (int i = 0; i < argc - 1; i++) {
        std::cerr << argv[i] << std::endl;
    }

    int ns_index = 1;

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_json_file>"
                  << std::endl;
        return 1;
    }

    std::string json_file_name = argv[1];
    std::string json_file_path =
        (fs::path(ament_index_cpp::get_package_share_directory("bringup")) /
         "config" / json_file_name)
            .string();

    std::ifstream json_file(json_file_path);
    if (!json_file.is_open()) {
        std::cerr << "Error opening file: " << json_file_path << std::endl;
        return 1;
    }

    nlohmann::json json_data;
    try {
        json_file >> json_data;
    }
    catch (nlohmann::json::parse_error &e) {
        std::cerr << "JSON parsing error: " << e.what() << std::endl;
        return 1;
    }

    SchedulerSingleton *sched = SchedulerSingleton::GetInstance();
    auto node = std::make_shared<rclcpp::Node>("agent_fact_node");

    for (auto it = json_data.begin(); it != json_data.end(); ++it) {
        std::cout << "Found type " << it.key()
                  << " in the json config. Iterating through..." << std::endl;
        std::string sdf_filename =
            (fs::path("models") / it.key() / "model.sdf").string();

        int i = 1;
        for (nlohmann::json agent_data : it.value()) {
            std::cerr << "Found agent data for this type: " << agent_data.dump()
                      << std::endl;

            std::string namespace_str = "ns_main" + std::to_string(ns_index);

            // TODO: Find a way to generate the xacro here

            std::string sdf_file = "";
            // sched->AddExecutor(executor);

            // Based on
            // https://liusongran.github.io/Blog/ROS2/Slides_refs/6.%20ROSWorld2021-Executor-Ralph%20Lange.pdf
            auto executor_thread = std::thread([&]() {
                auto executor = std::make_shared<
                    rclcpp::executors::MultiThreadedExecutor>();
                auto component_manager =
                    std::make_shared<rclcpp_components::ComponentManager>(
                        executor, namespace_str + "_manager");
                executor->add_node(component_manager);
                int nice = -5; // -20 to 19
                setpriority(PRIO_PROCESS, gettid(), nice);
                executor->spin();
            });

            executor_thread.detach();

            auto load_node_client =
                node->create_client<composition_interfaces::srv::LoadNode>(
                    // "/Test" + std::to_string(i) + "/_container/load_node");
                    namespace_str + "_manager" + "/_container/load_node");

            i++;

            // Wait for the service to be available
            while (
                !load_node_client->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(
                        node->get_logger(),
                        "Interrupted while waiting for the service. Exiting.");
                    return 1;
                }
                RCLCPP_INFO(
                    node->get_logger(),
                    "Service not available, waiting again...");
            }

            auto load_node_req =
                composition_interfaces::srv::LoadNode::Request();
            load_node_req.set__node_name(agent_data["name"]);
            load_node_req.set__node_namespace("/" + namespace_str);
            load_node_req.set__package_name("agent_cpp");
            load_node_req.set__plugin_name("AgentEntity");

            load_node_req.parameters.push_back(
                rclcpp::Parameter("use_sim_time", true).to_parameter_msg());
            load_node_req.parameters.push_back(
                rclcpp::Parameter("sdf_file", "").to_parameter_msg());
            load_node_req.parameters.push_back(
                rclcpp::Parameter("sdf_filename", sdf_filename)
                    .to_parameter_msg());
            load_node_req.parameters.push_back(
                rclcpp::Parameter("pose", std::string(agent_data["pose"]))
                    .to_parameter_msg());
            if (agent_data.contains("configure_on_startup")) {
                load_node_req.parameters.push_back(
                    rclcpp::Parameter(
                        "configure_on_startup",
                        bool(agent_data["configure_on_startup"]))
                        .to_parameter_msg());
            }
            else {
                load_node_req.parameters.push_back(
                    rclcpp::Parameter("configure_on_startup", true)
                        .to_parameter_msg());
            }

            // Set extra arguments
            load_node_req.extra_arguments.push_back(
                rcl_interfaces::msg::Parameter());
            load_node_req.extra_arguments.back().name =
                "use_intra_process_comms";
            load_node_req.extra_arguments.back().value.type =
                rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
            load_node_req.extra_arguments.back().value.bool_value = true;

            load_node_req.extra_arguments.push_back(
                rcl_interfaces::msg::Parameter());
            load_node_req.extra_arguments.back().name = "use_sim_time";
            load_node_req.extra_arguments.back().value.type =
                rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
            load_node_req.extra_arguments.back().value.bool_value = true;

            auto req = load_node_client->async_send_request(
                std::make_shared<
                    composition_interfaces::srv::LoadNode::Request>(
                    load_node_req));

            ns_index++;
        }
    }

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
