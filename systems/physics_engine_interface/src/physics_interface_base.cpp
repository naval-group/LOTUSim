#include "physics_engine_interface/physics_interface_base.hpp"

#include "physics_engine_interface/ros2_interface.hpp"
#include "physics_engine_interface/xdyn_websocket.hpp"

namespace lotusim::gazebo {

std::shared_ptr<PhysicsInterfaceBase> PhysicsInterfaceBase::createInterface(
    const InterfaceType& protocol_type,
    std::shared_ptr<std::unordered_map<gz::sim::Entity, std::string>> cmd,
    std::shared_ptr<spdlog::logger> logger)
{
    try {
        std::shared_ptr<PhysicsInterfaceBase> client;
        switch (protocol_type) {
            // XDynWebSocket & ROS2Interface are both Singleton by design,
            // therefore created through Static method. For other design, please
            // pass on the arguments to interface as deemed fit
            case InterfaceType::XDynWebSocket: {
                client = XdynWebsocket::createInterface();
                break;
            }
            case InterfaceType::ROS2Interface: {
                client = ROS2Interface::createInterface();
                break;
            }
            default: {
                logger->warn(
                    "PhysicsInterfacePlugin::createInterface: failed to create connection.");
                return nullptr;
            }
        }
        if (!client) {
            logger->error(
                "PhysicsInterfacePlugin::createInterface: failed to create connection.");
            return nullptr;
        }
        client->setSharedCmd(cmd);
        client->setLogger(logger);
        return client;
    } catch (...) {
        logger->error(
            "PhysicsInterfacePlugin::createInterface: Failed to create connection.");
        return nullptr;
    }
}

}  // namespace lotusim::gazebo