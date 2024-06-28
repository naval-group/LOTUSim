#include "Scheduler.hpp"

// Function to convert a map to a string
template <typename K, typename V>
std::string mapToString(const std::map<K, V> &m)
{
    std::ostringstream oss;
    oss << "{";
    for (auto it = m.begin(); it != m.end(); ++it) {
        if (it != m.begin())
            oss << ", ";
        oss << it->first << ": " << it->second;
    }
    oss << "}";
    return oss.str();
}

std::string
mapToString2(const std::map<std::string, std::vector<std::string>> &map)
{
    std::ostringstream oss;
    oss << "{";
    for (auto it = map.begin(); it != map.end(); ++it) {
        if (it != map.begin())
            oss << ", ";
        oss << "\"" << it->first << "\": [";
        for (size_t i = 0; i < it->second.size(); ++i) {
            if (i > 0)
                oss << ", ";
            oss << "\"" << it->second[i] << "\"";
        }
        oss << "]";
    }
    oss << "}";
    return oss.str();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // auto factory = std::make_shared<AgentFactory>();
    // AgentEntity::UniquePtr a =  factory->createAgent();

    auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto manager = std::make_shared<rclcpp_components::ComponentManager>(exec);
    auto sub_manager = std::make_shared<rclcpp_components::ComponentManager>(exec);

    exec->add_node(manager);
    exec->add_node(sub_manager);

    auto service_list = manager->get_service_names_and_types();

    RCLCPP_INFO(manager->get_logger(), mapToString2(service_list).c_str());

    auto LoadNodeClient =
        manager->create_client<composition_interfaces::srv::LoadNode>(
            "/scheduler_node/_container/load_node");

    // Wait for the service to be available
    while (!LoadNodeClient->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(
                manager->get_logger(),
                "Interrupted while waiting for the service. Exiting.");
            return 1;
        }
        RCLCPP_INFO(
            manager->get_logger(), "Service not available, waiting again...");
    }

    auto LoadNodeReq = composition_interfaces::srv::LoadNode::Request();
    LoadNodeReq.set__node_name("my_dynamic_component_node");
    LoadNodeReq.set__package_name("agent_cpp");
    LoadNodeReq.set__plugin_name("AgentEntity");

    LoadNodeReq.parameters.push_back(
        rclcpp::Parameter("use_sim_time", true).to_parameter_msg());
    LoadNodeReq.parameters.push_back(
        rclcpp::Parameter("sdf_file", "").to_parameter_msg());
    LoadNodeReq.parameters.push_back(
        rclcpp::Parameter("sdf_filename", "models/test_ship/model.sdf").to_parameter_msg());
    LoadNodeReq.parameters.push_back(
        rclcpp::Parameter("pose", "15 0 0 0 0 0").to_parameter_msg());
    LoadNodeReq.parameters.push_back(
        rclcpp::Parameter("configure_on_startup", true).to_parameter_msg());

    // Set extra arguments
    LoadNodeReq.extra_arguments.push_back(rcl_interfaces::msg::Parameter());
    LoadNodeReq.extra_arguments.back().name = "use_intra_process_comms";
    LoadNodeReq.extra_arguments.back().value.type =
        rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    LoadNodeReq.extra_arguments.back().value.bool_value = true;

    auto req = LoadNodeClient->async_send_request(
        std::make_shared<composition_interfaces::srv::LoadNode::Request>(
            LoadNodeReq));

    exec->spin();
    rclcpp::shutdown();
    return 0;
}