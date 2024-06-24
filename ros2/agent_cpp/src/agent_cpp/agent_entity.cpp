#include "agent_entity.hpp"

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