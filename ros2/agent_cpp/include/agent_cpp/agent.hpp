#ifndef AGENT_HH_
#define AGENT_HH_

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "rclcpp_components/component_manager.hpp"
#include "rclcpp_components/node_factory.hpp"

#include "liquidai_msgs/srv/add_entity_srv.hpp"
#include "liquidai_msgs/srv/add_entity_srv_array.hpp"
#include "liquidai_msgs/srv/remove_entity.hpp"

#include "DespawnInterface.hpp"
#include "DespawnOnGazebo.hpp"
#include "SpawnInterface.hpp"
#include "SpawnOnGazebo.hpp"

using namespace std;

/// @brief Every C++ agent should derive from this class. It is a LifecycleNode.
class Agent : public rclcpp_lifecycle::LifecycleNode {
public:
    Agent(const string &node_name, const rclcpp::NodeOptions &options);

    virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn
        on_configure(const rclcpp_lifecycle::State &previous_state) = 0;

    virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn
        on_activate(const rclcpp_lifecycle::State &previous_state) = 0;

    virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &previous_state) = 0;

    virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &previous_state) = 0;

    virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State &previous_state);

    void set_spawn(std::shared_ptr<SpawnInterface> spawn)
    {
        spawnInterface = spawn;
    }

    void set_despawn(std::shared_ptr<DespawnInterface> despawn)
    {
        despawnInterface = despawn;
    }

    bool perform_spawn() { return spawnInterface->spawn(); }
    bool perform_despawn() { return despawnInterface->despawn(); }

    string exec_command(const char *cmd);

protected:
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr
        param_change_callback_handle_;

    std::shared_ptr<SpawnInterface> spawnInterface;
    std::shared_ptr<DespawnInterface> despawnInterface;
};

#endif