#include "world_plugins/WaveRAOPlugin.h"

namespace liquidai {
namespace gazebo {

WaveRaoPlugin::WaveRaoPlugin() {}

void WaveRaoPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr)
{
    // Initiallizing vessel
    m_entity = _entity;
    auto name_opt = _ecm.Component<gz::sim::v7::components::Name>(_entity);
    if (name_opt) {
        m_vessel_name = name_opt->Data();
    }
    else {
        gzwarn << "No vessel name found. Commands topic will not be subscribed"
               << std::endl;
    }
    gz::math::Pose3d pose =
        _ecm.Component<gz::sim::v7::components::Pose>(m_entity)->Data();
    m_last_update_state_json = {
        {"t", 0},
        {"x", pose.X()},
        {"y", pose.Y()},
        {"z", pose.Z()},
        {"u", 0},
        {"v", 0},
        {"w", 0},
        {"p", 0},
        {"q", 0},
        {"r", 0},
        {"qr", pose.Rot().W()},
        {"qi", pose.Rot().X()},
        {"qj", pose.Rot().Y()},
        {"qk", pose.Rot().Z()}};

    // Getting websocket prarm
    std::string uri;
    auto sdfPtr = const_cast<sdf::Element *>(_sdf.get());
    if (sdfPtr->HasElement("uri")) {
        m_uri = sdfPtr->GetElement("uri")->Get<std::string>();
    }
    if (sdfPtr->HasElement("connection_type")) {
        std::string type =
            sdfPtr->GetElement("connection_type")->Get<std::string>();
        if (type == "XDynWebSocket") {

            gzmsg << "Creating XDynWebsocket." << std::endl;
            m_connection_type = ConnectionType::XDynWebSocket;
        }
        // else if (type == "XDynGRPC") {
        //     gzmsg << "Creating XDynGrpc." << std::endl;
        //     m_connection_type = ConnectionType::XDynGRPC;
        // }
        // else if (type == "Manual") {
        //     gzmsg << "Creating Manual." << std::endl;
        //     m_connection_type = ConnectionType::Manual;
        // }
        else {
            gzerr << "Attempting to a load an WaveRaoPlugin, but no known"
                     "connection_type received"
                  << std::endl;
        }
    }
    else {
        gzerr << "Attempting to a load an WaveRaoPlugin, but no "
                 "connection_type received"
              << std::endl;
        return;
    }
    createConnection();
}

void WaveRaoPlugin::Update(
    const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{
    auto time_now = std::chrono::system_clock::now();
    if (m_vessel_name.empty()) {
        return;
    }
    float target_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(_info.dt).count();
    std::optional<nlohmann::json> last_update_state_json_opt =
        m_client->getNewState(
            m_vessel_name, m_last_update_state_json, target_time);
    
    // update_state
    if (last_update_state_json_opt) {
        // TODO pay attention to rpy, get orientation right. What is Xdyn row
        // pitch yaw
        m_last_update_state_json = last_update_state_json_opt.value();
        // std::cout << m_last_update_state_json["z"] << std::endl;
        auto new_pose = gz::math::Pose3<double>(
            m_last_update_state_json["x"].back(),
            m_last_update_state_json["y"].back(),
            m_last_update_state_json["z"].back(),
            m_last_update_state_json["qr"].back(),
            m_last_update_state_json["qi"].back(),
            m_last_update_state_json["qj"].back(),
            m_last_update_state_json["qk"].back());
        bool res = _ecm.SetComponentData<gz::sim::components::Pose>(
            m_entity, new_pose);
    }
    else {
        std::cout << m_vessel_name << " update failed." << std::endl;
    }
    // std::cout << target_time << "\t"
    //           << std::chrono::duration_cast<std::chrono::milliseconds>(
    //                  std::chrono::system_clock::now() - time_now)
    //                  .count()
    //           << std::endl;
}

void WaveRaoPlugin::createConnection()
{
    // This can be reduced to configure. This not needed

    // switch (m_connection_type) {
    // case (ConnectionType::XDynWebSocket): {
    //     m_client = XdynWebsocket::getInstance();
    // }
    // // case (ConnectionType::XDynGRPC): {
    // //     m_client = XdynGrpc::getInstance();
    // // }
    // // case (ConnectionType::Manual): {
    // //     m_client = ManualRAO::getInstance();
    // // }
    // default: {
    //     std::cout << "WaveRaoPlugin::createConnection " << m_vessel_name
    //               << " uri: " << m_uri << " failed to create connection."
    //               << std::endl;
    // }
    // }

    m_client = XdynWebsocket::getInstance();
    m_client->createConnection(m_vessel_name, m_uri);
}

} // namespace gazebo
} // namespace liquidai
GZ_ADD_PLUGIN(
    liquidai::gazebo::WaveRaoPlugin,
    gz::sim::System,
    liquidai::gazebo::WaveRaoPlugin::ISystemConfigure,
    liquidai::gazebo::WaveRaoPlugin::ISystemUpdate)