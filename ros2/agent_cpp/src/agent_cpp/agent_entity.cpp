#include "agent_entity.hpp"

AgentEntity::AgentEntity(const rclcpp::NodeOptions &options)
    : Agent("agent_entity", options)
{
    get_parameter("sdf_file", sdf_file_);
    get_parameter("sdf_filename", sdf_filename_);
    get_parameter("pose", pose_str);
    get_parameter<bool>("configure_on_startup", configure_on_startup);

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

    const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions();
    // this->get_node_base_interface()
    // auto sensor = std::shared_ptr<MyIMUSensor>(new
    // MyIMUSensor(options_sensor)); this->add_sensor(sensor);

    // Load itself into the scheduler ?

    if (configure_on_startup) {
        this->on_configure(this->get_current_state());
    }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AgentEntity::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(get_logger(), "on_configure() is called.");

    bool isFine = true;

    isFine = isFine && this->perform_spawn();

    if (isFine) {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
            CallbackReturn::SUCCESS;
    }
    else {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
            CallbackReturn::ERROR;
    }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AgentEntity::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AgentEntity::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AgentEntity::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(get_logger(), "on_cleanup() is called.");

    bool isFine = true;

    isFine = isFine && this->perform_despawn();

    if (isFine) {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
            CallbackReturn::SUCCESS;
    }
    else {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
            CallbackReturn::ERROR;
    }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AgentEntity::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(get_logger(), "on_shutdown() is called.");

    bool isFine = true;

    isFine = isFine && this->perform_despawn();

    if (isFine) {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
            CallbackReturn::SUCCESS;
    }
    else {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
            CallbackReturn::ERROR;
    }
}

bool AgentEntity::GetSensors() {}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(AgentEntity)