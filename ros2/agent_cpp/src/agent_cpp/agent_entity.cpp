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

    // Subscription to the Gazebo pose
    gz_poses_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/model/" + name_ + "/pose",
        10,
        std::bind(&AgentEntity::gz_pose_callback, this, std::placeholders::_1));

    pose_pub_ = this->create_publisher<liquidai_msgs::msg::EntityPosition>(
        "/pose_effector", 10);

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

    timer_ = rclcpp::create_timer(
        this,
        this->get_clock(),
        std::chrono::milliseconds(1000),
        std::bind(&AgentEntity::timer_callback, this));

    if (configure_on_startup) {
        this->on_configure(this->get_current_state());
    }
}

void AgentEntity::timer_callback()
{
    if (gazebo_id == 0) {
        return;
    }

    // Print the current state for demo purposes
    if (!pose_pub_->is_activated()) {
        RCLCPP_DEBUG(
            get_logger(),
            "Lifecycle publisher is currently inactive. Messages are not "
            "published.");
    }
    else {
        RCLCPP_DEBUG(get_logger(), "Lifecycle publisher is active. Publishing...");

        auto message = liquidai_msgs::msg::EntityPosition();
        message.id = gazebo_id;
        message.position.set__x(pose_.position.x + 0.1);
        message.position.set__y(pose_.position.y);
        message.position.set__z(pose_.position.z);
        RCLCPP_INFO(
            this->get_logger(),
            "Publishing: '%s' on id %d",
            std::to_string(message.position.x).c_str(),
            gazebo_id);
        pose_pub_->publish(message);
    }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AgentEntity::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(get_logger(), "on_configure() is called.");

    if (!this->perform_spawn()) {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
            CallbackReturn::ERROR;
    }

    if (!this->get_gazebo_id()) {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
            CallbackReturn::ERROR;
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AgentEntity::on_activate(const rclcpp_lifecycle::State &previous_state)
{

    LifecycleNode::on_activate(previous_state);

    RCLCPP_INFO(get_logger(), "on_activate() is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AgentEntity::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    LifecycleNode::on_deactivate(previous_state);

    RCLCPP_INFO(get_logger(), "on_deactivate() is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AgentEntity::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(get_logger(), "on_cleanup() is called.");

    timer_.reset();
    pose_pub_.reset();
    gazebo_id = 0;

    if (!this->perform_despawn()) {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
            CallbackReturn::ERROR;
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AgentEntity::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(get_logger(), "on_shutdown() is called.");

    timer_.reset();
    pose_pub_.reset();
    gazebo_id = 0;

    if (!this->perform_despawn()) {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
            CallbackReturn::ERROR;
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
}

bool AgentEntity::get_gazebo_id()
{
    while (!get_id_by_name_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Interrupted while waiting for the service. Exiting.");
            exit(0);
        }
        RCLCPP_INFO(
            this->get_logger(), "Service not available, waiting again...");
    }

    auto request = std::make_shared<liquidai_msgs::srv::GetIdByName::Request>();
    request->set__entity_name(name_);
    auto res = get_id_by_name_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(
            entity_management_client_node, res) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        int id = res.get()->id;
        if (id != 0) {
            RCLCPP_INFO(
                this->get_logger(),
                "Successfully got and set the ID %d from Gazebo!",
                id);
            gazebo_id = id;
            return true;
        }
        else {
            RCLCPP_WARN(this->get_logger(), "The Gazebo Id I got is 0.");
            return false;
        }
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service");
        return false;
    }
}

/// @brief Get the pose from Gazebo. It is sent by the Odometry system
/// plugin.
/// @param msg
void AgentEntity::gz_pose_callback(
    const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    pose_ = msg->poses.back();
}

// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(AgentEntity)