#ifndef AGENT_HH_
#define AGENT_HH_

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdexcept>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_components/node_factory.hpp"

#include "custom_gz_interfaces/srv/add_entity.hpp"
#include "custom_gz_interfaces/srv/remove_entity.hpp"

using namespace std;

class Agent : public rclcpp_lifecycle::LifecycleNode
{
public:
    Agent(const string &node_name, const rclcpp::NodeOptions &options)
        : rclcpp_lifecycle::LifecycleNode(node_name, options)
    {
        // We need to declare all parameters here
        declare_parameter("sdf_file", "");
        declare_parameter("pose", "");

        auto param_change_callback =
            [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
        {
            auto result = rcl_interfaces::msg::SetParametersResult();
            result.successful = true;
            for (auto parameter : parameters)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "parameter '%s' is now: %s",
                    parameter.get_name().c_str(),
                    parameter.value_to_string().c_str());
            }
            return result;
        };

        param_change_callback_handle_ = this->add_on_set_parameters_callback(param_change_callback);
    }

    virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &previous_state) = 0;

    virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &previous_state) = 0;

    virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &previous_state) = 0;

    virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &previous_state) = 0;

    virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &previous_state);

    virtual bool spawn() = 0;

    virtual bool despawn() = 0;

    string exec_command(const char *cmd);

protected:
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_change_callback_handle_;
};

#endif