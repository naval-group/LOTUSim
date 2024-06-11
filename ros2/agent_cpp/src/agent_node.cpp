#include "agent.hpp"
#include "agent_entity.hpp"

shared_ptr<AgentEntity> agent_node;

// int main(int argc, char *argv[])
// {
//     // force flush of the stdout buffer.
//     // this ensures a correct sync of all prints
//     // even when executed simultaneously within the launch file.
//     setvbuf(stdout, NULL, _IONBF, BUFSIZ);

//     rclcpp::init(argc, argv);

//     rclcpp::NodeOptions options;

//     agent_node =
//         std::make_shared<AgentEntity>("agent_node", options);
    
//     rclcpp::executors::SingleThreadedExecutor exe;
//     exe.add_node(agent_node->get_node_base_interface());
//     exe.spin();

//     rclcpp::shutdown();

//     return 0;
// }

RCLCPP_COMPONENTS_REGISTER_NODE(AgentEntity)

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<rclcpp::Node>("my_composition");

//     // Create a node factory for the component
//     auto node_factory = std::make_shared<rclcpp_components::NodeFactory>(
//         node->get_node_base_interface(), node->get_node_topics_interface(),
//         node->get_node_graph_interface(), node->get_node_services_interface(),
//         node->get_node_timers_interface(), node->get_node_parameters_interface());

//     rclcpp::NodeOptions options;

//     // Create an instance of the component node
//     auto my_component = node_factory->create_node_instance(options);

//     rclcpp::spin(node);

//     rclcpp::shutdown();
//     return 0;
// }