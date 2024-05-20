#include "agent_entity.hpp"

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    AgentEntity::on_configure(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(get_logger(), "on_configure() is called.");

        bool isFine = true;

        entity_management_client_node = rclcpp::Node::make_shared("entity_management_client_node");
        isFine = isFine && this->spawn();

        if(isFine){
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }
        else{
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
    }

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    AgentEntity::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    AgentEntity::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    AgentEntity::on_cleanup(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(get_logger(), "on_cleanup() is called.");
        
        bool isFine = true;

        isFine = isFine && this->despawn();

        if(isFine){
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }
        else
        {
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
    }

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    AgentEntity::on_shutdown(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(get_logger(), "on_shutdown() is called.");

        bool isFine = true;

        isFine = isFine && this->despawn();

        if(isFine){
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }
        else{
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
    }


bool AgentEntity::spawn(){
    char full_name[100]; 
    strcpy(full_name,this->get_namespace());
    strcat(full_name,"/");
    strcat(full_name,this->get_name());
    RCLCPP_DEBUG(get_logger(), "Creating ros2 node of agent %s", full_name);

    // Parse the pose string
    float pose_components[6];
    int i = 0;
    stringstream ssin(pose_str);
    while (ssin.good() && i < 6){
        ssin >> pose_components[i];
        ++i;
    }

    rclcpp::Client<custom_gz_interfaces::srv::AddEntity>::SharedPtr client =
    entity_management_client_node->create_client<custom_gz_interfaces::srv::AddEntity>("/add_entity");

    auto request = std::make_shared<custom_gz_interfaces::srv::AddEntity::Request>();

    request->name = full_name;
    request->model_filepath = this->sdf_file_;

    geometry_msgs::msg::Point p;
    p.set__x(pose_components[0]);
    p.set__y(pose_components[1]);
    p.set__z(pose_components[2]);
    request->location = p;

    geometry_msgs::msg::Vector3 r;
    r.set__x(pose_components[3]);
    r.set__y(pose_components[4]);
    r.set__z(pose_components[5]);
    request->rotation = r;

    // Wait for the service to be activated
    while (!client->wait_for_service(1s)) {
        // If ROS is shutdown before the service is activated, show this error
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return false;
        }
        // Print in the screen some information so the user knows what is happening
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s service not available, waiting again...", full_name);
    }

    // Client sends its asynchronous request
    auto res = client->async_send_request(request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(entity_management_client_node, res) == rclcpp::FutureReturnCode::SUCCESS) {
        // Get the response's success field to see if all checks passed
        if (res.get()->result) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The checks were successful!");
            return true;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "The checks were not successful: %s", "");
            return false;
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service 'checks'");
        return false;
    }
}

bool AgentEntity::despawn(){
    char full_name[100]; 
    strcpy(full_name,this->get_namespace());
    strcat(full_name,"/");
    strcat(full_name,this->get_name());
    RCLCPP_DEBUG(get_logger(), "Destroying ros2 node of agent %s", full_name);
    
    rclcpp::Client<custom_gz_interfaces::srv::RemoveEntity>::SharedPtr client =
        entity_management_client_node->create_client<custom_gz_interfaces::srv::RemoveEntity>("/remove_entity");

    auto request = std::make_shared<custom_gz_interfaces::srv::RemoveEntity::Request>();

    request->name = full_name;

    // Wait for the service to be activated
    while (!client->wait_for_service(1s)) {
        // If ROS is shutdown before the service is activated, show this error
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return false;
        }
        // Print in the screen some information so the user knows what is happening
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s service not available, waiting again...", full_name);
    }

    // Client sends its asynchronous request
    auto res = client->async_send_request(request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(entity_management_client_node, res) == rclcpp::FutureReturnCode::SUCCESS) {
        // Get the response's success field to see if all checks passed
        if (res.get()->result) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The checks were successful!");
            return true;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "The checks were not successful: %s", "bruh");
            return false;
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service 'checks'");
    }
}