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

    get_id_by_name_client_ =
        entity_management_client_node
            ->create_client<liquidai_msgs::srv::GetIdByName>(
                "/gz_get_id_by_name");

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

    timer_ = rclcpp::create_timer(
        this,
        this->get_clock(),
        std::chrono::milliseconds(500),
        std::bind(&AgentEntity::timer_callback, this));

    if (configure_on_startup) {
        this->on_configure(this->get_current_state());
    }
}

void AgentEntity::timer_callback()
{
    RCLCPP_INFO(get_logger(), "my id is %d", gazebo_id);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AgentEntity::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(get_logger(), "on_configure() is called.");

    if (!this->perform_spawn()) {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
            CallbackReturn::ERROR;
    }

    // Wait for the service to be activated
    while (!get_id_by_name_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Interrupted while waiting for the service. Exiting.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
                CallbackReturn::ERROR;
        }
        // Print in the screen some information so the user knows what is
        // happening
        RCLCPP_INFO(
            this->get_logger(), "Service not available, waiting again...");
    }

    auto request = std::make_shared<liquidai_msgs::srv::GetIdByName::Request>();
    std::string name = this->get_name();
    request->set__entity_name(name);

    // Client sends its asynchronous request
    auto res = get_id_by_name_client_->async_send_request(request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(
            entity_management_client_node, res) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        int id = res.get()->id;
        if (id != 0) {
            RCLCPP_INFO(this->get_logger(), "The checks were successful!");
            gazebo_id = id;
        }
        else {
            RCLCPP_WARN(
                this->get_logger(),
                "The checks were not successful: %s",
                "bruh");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
                CallbackReturn::ERROR;
        }
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service 'checks'");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
            CallbackReturn::ERROR;
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
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

    if (!this->perform_despawn()) {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
            CallbackReturn::SUCCESS;
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AgentEntity::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(get_logger(), "on_shutdown() is called.");

    if (!this->perform_despawn()) {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
            CallbackReturn::SUCCESS;
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
}

bool AgentEntity::GetSensors() {}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(AgentEntity)