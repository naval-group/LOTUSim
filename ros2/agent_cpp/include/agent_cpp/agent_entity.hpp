#ifndef AGENT_ENTITY_HH_
#define AGENT_ENTITY_HH_

#include "agent.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sdf/sdf.hh>

// An agent that is an entity in Gazebo
class AgentEntity : public Agent {
private:
    string sdf_file_;
    string sdf_filename_;
    string pose_str;
    string spawn_on_startup_;
    rclcpp::Node::SharedPtr entity_management_client_node;
    // rclcpp::Client<liquidai_msgs::srv::AddEntitySrvArray>::SharedPtr
    //     add_entity_client;

public:
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

    AgentEntity(const rclcpp::NodeOptions &options)
        : Agent("agent_entity", options)
    {
        get_parameter("sdf_file", sdf_file_);
        get_parameter("sdf_filename", sdf_filename_);
        get_parameter("pose", pose_str);

        entity_management_client_node =
            rclcpp::Node::make_shared("entity_management_client_node");

        // Parse the pose string
        float pose_components[6];
        int i = 0;
        stringstream ssin(pose_str);
        while (ssin.good() && i < 6) {
            ssin >> pose_components[i];
            ++i;
        }

        geometry_msgs::msg::Point p;
        p.set__x(pose_components[0]);
        p.set__y(pose_components[1]);
        p.set__z(pose_components[2]);

        geometry_msgs::msg::Vector3 r;
        r.set__x(pose_components[3]);
        r.set__y(pose_components[4]);
        r.set__z(pose_components[5]);

        auto spawn_request = liquidai_msgs::srv::AddEntitySrv::Request();

        spawn_request.data.name = this->get_name();
        spawn_request.data.model_filepath = this->sdf_filename_;
        spawn_request.data.model_file = this->sdf_file_;
        spawn_request.data.location = p;
        spawn_request.data.rotation = r;

        auto spawnBehavior = std::make_shared<SpawnOnGazebo>(
            spawn_request, entity_management_client_node);
        this->set_spawn(spawnBehavior);

        auto despawn_request = liquidai_msgs::srv::RemoveEntity::Request();

        despawn_request.name = this->get_name();

        auto despawnBehavior = std::make_shared<DespawnOnGazebo>(
            despawn_request, entity_management_client_node);
        this->set_despawn(despawnBehavior);

        // // Spawn entity on Gazebo at startup, may be unstable
        // spawn();
    };

    bool GetSensors();
};

#endif