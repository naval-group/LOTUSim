#include "agent_entity.hpp"

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AgentEntity::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(get_logger(), "on_configure() is called.");

    bool isFine = true;

    this->add_entity_client =
        entity_management_client_node
            ->create_client<liquidai_msgs::srv::AddEntitySrvArray>(
                "/gz_add_entity_V");

    isFine = isFine && this->spawn();

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

    isFine = isFine && this->despawn();

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

    isFine = isFine && this->despawn();

    if (isFine) {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
            CallbackReturn::SUCCESS;
    }
    else {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
            CallbackReturn::ERROR;
    }
}

/// @brief Spawn command called on OnConfigure()
/// @return Returns true if successfully spawned entity
bool AgentEntity::spawn()
{
    char full_name[100];
    // strcpy(full_name, this->get_namespace());
    // strcat(full_name, "/");
    // strcat(full_name, this->get_name());
    strcpy(full_name, this->get_name());
    RCLCPP_DEBUG(get_logger(), "Creating ros2 node of agent %s", full_name);

    // Parse the pose string
    float pose_components[6];
    int i = 0;
    stringstream ssin(pose_str);
    while (ssin.good() && i < 6) {
        ssin >> pose_components[i];
        ++i;
    }

    // When send a create_multiple request through the gz_entity_management
    // package
    auto request_V =
        std::make_shared<liquidai_msgs::srv::AddEntitySrvArray::Request>();

    auto request =
        std::make_shared<liquidai_msgs::srv::AddEntitySrv::Request>();

    request->data.name = full_name;
    request->data.model_filepath = this->sdf_filename_;
    request->data.model_file = this->sdf_file_;

    geometry_msgs::msg::Point p;
    p.set__x(pose_components[0]);
    p.set__y(pose_components[1]);
    p.set__z(pose_components[2]);
    request->data.location = p;

    geometry_msgs::msg::Vector3 r;
    r.set__x(pose_components[3]);
    r.set__y(pose_components[4]);
    r.set__z(pose_components[5]);
    request->data.rotation = r;

    // Wait for the service to be activated
    while (!this->add_entity_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(
                rclcpp::get_logger("rclcpp"),
                "Interrupted while waiting for the service. Exiting.");
            return false;
        }

        RCLCPP_INFO(
            rclcpp::get_logger("rclcpp"),
            "%s service not available, waiting again...",
            full_name);
    }

    // Possibility of spawning multiple things here
    request_V->data.push_back(request->data);

    auto res = this->add_entity_client->async_send_request(request_V);

    if (rclcpp::spin_until_future_complete(
            entity_management_client_node, res) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        // Get the response's success field to see if all checks passed
        if (res.get()->result) {
            RCLCPP_INFO(
                rclcpp::get_logger("rclcpp"), "The checks were successful!");
            return true;
        }
        else {
            RCLCPP_WARN(
                rclcpp::get_logger("rclcpp"),
                "The checks were not successful: %s",
                "");
            return false;
        }
    }
    else {
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"), "Failed to call service 'checks'");
        return false;
    }
}

/// @brief Despawn command called on OnCleanup() or OnShutdown()
/// @return Returns true if successfully despawned
bool AgentEntity::despawn()
{
    char full_name[100];
    // strcpy(full_name, this->get_namespace());
    // strcat(full_name, "/");
    // strcat(full_name, this->get_name());
    strcpy(full_name, this->get_name());
    RCLCPP_DEBUG(get_logger(), "Destroying ros2 node of agent %s", full_name);

    rclcpp::Client<liquidai_msgs::srv::RemoveEntity>::SharedPtr client =
        entity_management_client_node
            ->create_client<liquidai_msgs::srv::RemoveEntity>(
                "/gz_remove_entity");

    auto request =
        std::make_shared<liquidai_msgs::srv::RemoveEntity::Request>();

    request->name = full_name;

    // Wait for the service to be activated
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(
                rclcpp::get_logger("rclcpp"),
                "Interrupted while waiting for the service. Exiting.");
            return false;
        }

        RCLCPP_INFO(
            rclcpp::get_logger("rclcpp"),
            "%s service not available, waiting again...",
            full_name);
    }

    auto res = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(
            entity_management_client_node, res) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        // Get the response's success field to see if all checks passed
        if (res.get()->result) {
            RCLCPP_INFO(
                rclcpp::get_logger("rclcpp"), "The checks were successful!");
            return true;
        }
        else {
            RCLCPP_WARN(
                rclcpp::get_logger("rclcpp"),
                "The checks were not successful: %s",
                "bruh");
            return false;
        }
    }
    else {
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"), "Failed to call service 'checks'");
    }
}

bool AgentEntity::GetSensors()
{
    string sdfPath = ament_index_cpp::get_package_share_directory("assets") +
                     "/" + sdf_file_;

    // load and check sdf file
    sdf::SDFPtr sdfElement(new sdf::SDF());
    sdf::init(sdfElement);
    if (!sdf::readFile(sdfPath, sdfElement)) {
        std::cerr << sdfPath << " is not a valid SDF file!" << std::endl;
        return -2;
    }

    // start parsing model
    const sdf::ElementPtr rootElement = sdfElement->Root();
    if (!rootElement->HasElement("model")) {
        std::cerr << sdfPath << " is not a model SDF file!" << std::endl;
        return -3;
    }
    const sdf::ElementPtr modelElement = rootElement->GetElement("model");
    const std::string modelName = modelElement->Get<std::string>("name");
    std::cout << "Found " << modelName << " model!" << std::endl;

    // parse model links
    sdf::ElementPtr linkElement = modelElement->GetElement("link");
    while (linkElement) {
        const std::string linkName = linkElement->Get<std::string>("name");
        std::cout << "Found " << linkName << " link in " << modelName
                  << " model!" << std::endl;
        linkElement = linkElement->GetNextElement("link");
    }

    // parse model joints
    sdf::ElementPtr jointElement = modelElement->GetElement("joint");
    while (jointElement) {
        const std::string jointName = jointElement->Get<std::string>("name");
        std::cout << "Found " << jointName << " joint in " << modelName
                  << " model!" << std::endl;

        const sdf::ElementPtr parentElement =
            jointElement->GetElement("parent");
        const std::string parentLinkName = parentElement->Get<std::string>();

        const sdf::ElementPtr childElement = jointElement->GetElement("child");
        const std::string childLinkName = childElement->Get<std::string>();

        std::cout << "Joint " << jointName << " connects " << parentLinkName
                  << " link to " << childLinkName << " link" << std::endl;

        jointElement = jointElement->GetNextElement("joint");
    }

    if (!modelElement->HasElement("sensors")) {
        std::cerr << sdfPath << " does not have sensors!" << std::endl;
        return -3;
    }

    // Get the sensors element
    sdf::ElementPtr sensorsElement = modelElement->GetElement("sensors");

    // Iterate through each sensor
    sdf::ElementPtr pluginElement =
        sensorsElement->GetFirstElement(); // Get the first child element

    while (pluginElement) {
        if (pluginElement->HasElement("plugin")) {
            // Process the sdf tag here
            // For example, you can retrieve attributes or nested elements
            // Assuming sdfElement is the sdf tag you want to work with
            // Example code to retrieve attributes:
            std::string sdfName = pluginElement->Get<std::string>("name");

            // Example code to retrieve nested elements
            sdf::ElementPtr nestedElement =
                pluginElement->GetElement("nested_element_name");

            // Continue processing as needed

            // Move to the next sdf element
        }
        // Move to the next element under sensors
        pluginElement = pluginElement->GetNextElement();
    }
}