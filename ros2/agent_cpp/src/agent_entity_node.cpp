#include "agent.hpp"
#include "agent_entity.hpp"

shared_ptr<AgentEntity> agent_node;

int main(int argc, char *argv[])
{
    // force flush of the stdout buffer.
    // this ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    agent_node = std::make_shared<AgentEntity>(options);

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(agent_node->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();

    return 0;
}