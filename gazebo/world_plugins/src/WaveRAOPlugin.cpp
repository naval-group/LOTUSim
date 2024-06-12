#include "world_plugins/WaveRAOPlugin.h"

namespace liquidai {
namespace gazebo {

WaveRaoPlugin::WaveRaoPlugin()
    : m_debug(false)
{
}
WaveRaoPlugin::~WaveRaoPlugin() {}

void WaveRaoPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr)
{
    gzdbg << "WaveRaoPlugin initiated" << std::endl;
    auto sdfPtr = const_cast<sdf::Element *>(_sdf.get());
    if (sdfPtr->HasElement("debug")) {
        m_debug = sdfPtr->Get<bool>("debug");
    }
}

void WaveRaoPlugin::PreUpdate(
    const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &_ecm)
{
    _ecm.EachNew<gz::sim::components::ModelSdf>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::ModelSdf *_model) -> bool {
            gz::sim::Entity base_link;
            sdf::Model data = _model->Data();
            sdf::ElementPtr sdfptr = data.Element();
            auto name_opt = _ecm.Component<gz::sim::components::Name>(_entity);
            std::string vessel_name;
            if (name_opt) {
                vessel_name = name_opt->Data();
            }
            else {
                gzwarn << "WaveRaoPlugin::PreUpdate: No vessel name found. "
                          "Ignoring model."
                       << std::endl;
                return false;
            }

            if (sdfptr->HasElement("physics_server_interface")) {
                gzdbg << sdfptr->ToString("") << std::endl;
                // Get Base Link
                m_vessels_model_map[vessel_name] = _entity;
                m_vessels_name_map[_entity] = vessel_name;

                auto child_link = _ecm.ChildrenByComponents(
                    _entity, gz::sim::components::Link());
                for (auto &&link : child_link) {
                    auto name_opt =
                        _ecm.Component<gz::sim::components::Name>(link);
                    if (name_opt && name_opt->Data().find("base_link") !=
                                        std::string::npos) {
                        base_link = link;
                        m_vessels_base_link_map[vessel_name] = base_link;
                        gz::sim::Link _link(base_link);
                        _link.EnableVelocityChecks(_ecm);
                    }
                }

                // Init Interface
                sdf::ElementPtr render_sdf_ptr =
                    sdfptr->GetElement("physics_server_interface");
                gzmsg << "WaveRaoPlugin::PreUpdate: " << vessel_name
                      << " physics update request detected."
                      << "\n";

                // If vessel has aerial component
                if (render_sdf_ptr->HasElement("aerial")) {
                    sdf::ElementPtr aerial_sdf =
                        render_sdf_ptr->GetElement("aerial");

                    ConnectionType aerial_connection_type;

                    if (!aerial_sdf->HasElement("ConnectionType") ||
                        !aerial_sdf->HasElement("uri")) {
                        gzwarn << "WaveRaoPlugin::PreUpdate: " << vessel_name
                               << "missing aerial connectionType or uri. "
                                  "removing aerial calculation. \n";
                    }
                    aerial_connection_type =
                        ConnectionTypeMap[aerial_sdf->Get<std::string>(
                            "ConnectionType")];
                    m_aerial_interface[_entity] = createConnection(
                        _entity,
                        vessel_name,
                        aerial_connection_type,
                        aerial_sdf);
                }

                // If vessel has surface component
                if (render_sdf_ptr->HasElement("surface")) {
                    sdf::ElementPtr surface_sdf =
                        render_sdf_ptr->GetElement("surface");

                    ConnectionType surface_connection_type;
                    if (!surface_sdf->HasElement("ConnectionType") ||
                        !surface_sdf->HasElement("uri")) {
                        gzwarn << "WaveRaoPlugin::PreUpdate: " << vessel_name
                               << "missing surface connectionType or uri. "
                                  "removing surface calculation. \n";
                    }
                    surface_connection_type =
                        ConnectionTypeMap[surface_sdf->Get<std::string>(
                            "ConnectionType")];
                    m_surface_interface[_entity] = createConnection(
                        _entity,
                        vessel_name,
                        surface_connection_type,
                        surface_sdf);
                }

                // If vessel has underwater component
                if (render_sdf_ptr->HasElement("underwater")) {
                    sdf::ElementPtr underwater_sdf =
                        render_sdf_ptr->GetElement("underwater");

                    ConnectionType underwater_connection_type;

                    if (!underwater_sdf->HasElement("ConnectionType") ||
                        !underwater_sdf->HasElement("uri")) {
                        gzwarn << "WaveRaoPlugin::PreUpdate: " << vessel_name
                               << "missing underwater connectionType or uri. "
                                  "removing underwater calculation. \n";
                    }
                    underwater_connection_type =
                        ConnectionTypeMap[underwater_sdf->Get<std::string>(
                            "ConnectionType")];
                    m_underwater_interface[_entity] = createConnection(
                        _entity,
                        vessel_name,
                        underwater_connection_type,
                        underwater_sdf);
                }
                // Add warning that the init is not found and find the
                // possible state
                if (render_sdf_ptr->HasElement("init_state")) {
                    DomainType init_domain =
                        DomainTypeMap[render_sdf_ptr->Get<std::string>(
                            "init_state")];

                    switch (init_domain) {
                    case (DomainType::Aerial): {
                        if (m_aerial_interface.find(_entity) !=
                            m_aerial_interface.end()) {
                            m_current_vessel_interface[_entity] =
                                m_aerial_interface[_entity];
                            m_vehicle_current_mode[_entity] =
                                DomainType::Aerial;
                        }
                        else {
                            gzerr << "WaveRaoPlugin::PreUpdate: " << vessel_name
                                  << "init state aerial. No aerial set. "
                                     "Failed "
                                     "\n";
                        }
                        break;
                    }
                    case (DomainType::Surface): {
                        if (m_surface_interface.find(_entity) !=
                            m_surface_interface.end()) {
                            m_current_vessel_interface[_entity] =
                                m_surface_interface[_entity];
                            m_vehicle_current_mode[_entity] =
                                DomainType::Surface;
                        }
                        else {
                            gzerr << "WaveRaoPlugin::PreUpdate: " << vessel_name
                                  << "init state surface. No surface set. "
                                     "Failed "
                                     "\n";
                        }
                        break;
                    }
                    case (DomainType::Underwater): {
                        if (m_underwater_interface.find(_entity) !=
                            m_underwater_interface.end()) {
                            m_current_vessel_interface[_entity] =
                                m_underwater_interface[_entity];
                            m_vehicle_current_mode[_entity] =
                                DomainType::Underwater;
                        }
                        else {
                            gzerr << "WaveRaoPlugin::PreUpdate: " << vessel_name
                                  << "init state underwater. No underwater "
                                     "set. Failed "
                                     "\n";
                        }
                        break;
                    }
                    default: {
                        gzerr << "WaveRaoPlugin::PreUpdate: " << vessel_name
                              << "init state no state found. Failed "
                                 "\n";
                        return false;
                    }
                    }
                    m_current_vessel_interface[_entity]->activateConnection(
                        _entity);
                }
            }
            else {
                gzmsg << "WaveRaoPlugin::PreUpdate: Not calculating phyics for "
                      << vessel_name << "\n";
            }

            // Add the uri and domain info
            gzmsg << "WaveRaoPlugin::PreUpdate: " << vessel_name
                  << " physics successfully created."
                  << "\n";
            return true;
        });
}

void WaveRaoPlugin::Update(
    const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{
    if (_info.dt.count() == 0) {
        return;
    }

    float target_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(_info.dt).count();

    for (auto &&vessel : m_vessels_name_map) {
        if (m_current_vessel_interface.find(vessel.first) ==
            m_current_vessel_interface.end()) {
            gzwarn << "WaveRaoPlugin::Update " << vessel.second
                   << " update failed. Unable for find "
                      "interface\n";
        }
        std::chrono::_V2::system_clock::time_point time_now;
        time_now = std::chrono::system_clock::now();
        gz::math::Pose3d pose =
            _ecm.Component<gz::sim::components::Pose>(vessel.first)->Data();

        gz::sim::Link _link(m_vessels_base_link_map[vessel.second]);
        auto lin_vel = _link.WorldLinearVelocity(_ecm).value();
        auto ang_vel = _link.WorldAngularVelocity(_ecm).value();
        json vessel_info = {
            {"t",
             std::chrono::duration_cast<std::chrono::milliseconds>(
                 _info.simTime)
                     .count() /
                 1000.0},
            {"x", pose.X()},
            {"y", pose.Y()},
            {"z", pose.Z()},
            {"u", lin_vel.X()},
            {"v", lin_vel.Y()},
            {"w", lin_vel.Z()},
            {"p", ang_vel.X()},
            {"q", ang_vel.Y()},
            {"r", ang_vel.Z()},
            {"qr", pose.Rot().W()},
            {"qi", pose.Rot().X()},
            {"qj", pose.Rot().Y()},
            {"qk", pose.Rot().Z()}};

        std::optional<std::tuple<nlohmann::json, DomainType>> update_opt =
            m_current_vessel_interface[vessel.first]->getNewState(
                vessel.first, vessel_info, target_time);

        if (update_opt) {
            // if (!vesselTransition(
            //         vessel.first, std::get<1>(update_opt.value()))) {
            //     gzerr << "WaveRaoPlugin::Update vessel " << vessel.second
            //           << "transition failed" << std::endl;
            // }

            json update = std::move(std::get<0>(update_opt.value()));
            pose = gz::math::Pose3<double>(
                update["x"].back(),
                update["y"].back(),
                update["z"].back(),
                update["qr"].back(),
                update["qi"].back(),
                update["qj"].back(),
                update["qk"].back());

            auto lin_vel = gz::math::Vector3d{
                update["u"].back(), update["v"].back(), update["w"].back()};

            auto ang_vel = gz::math::Vector3d{
                update["p"].back(), update["q"].back(), update["r"].back()};

            bool res = _ecm.SetComponentData<gz::sim::components::Pose>(
                vessel.first, pose);
            if (!res)
                gzwarn << "WaveRaoPlugin::Update " << vessel.second
                       << " Change Position failed." << std::endl;
            res =
                _ecm.SetComponentData<gz::sim::components::WorldLinearVelocity>(
                    m_vessels_base_link_map[vessel.second], lin_vel);
            if (!res)
                gzwarn << "WaveRaoPlugin::Update " << vessel.second
                       << " Change linear velocity failed." << std::endl;
            const auto angularVel =
                _ecm.Component<gz::sim::components::WorldAngularVelocity>(
                    m_vessels_base_link_map[vessel.second]);
            *angularVel = gz::sim::components::WorldAngularVelocity(ang_vel);
            if (!res) {
                gzwarn << "WaveRaoPlugin::Update " << vessel.second
                       << " Change position failed." << std::endl;
            }
        }
        else {
            gzwarn << vessel.first << " update failed." << std::endl;
        }
    }
}

std::shared_ptr<WaveRaoInterface> WaveRaoPlugin::createConnection(
    const gz::sim::Entity &entity,
    const std::string &name,
    const ConnectionType &protocol_type,
    const sdf::ElementPtr sdf)
{
    std::shared_ptr<WaveRaoInterface> client;

    switch (protocol_type) {
    case (ConnectionType::XDynWebSocket): {
        client = XdynWebsocket::getInstance(entity, name);
        break;
    }
    // case (ConnectionType::XDynGRPC): {
    //     client = XdynGrpc();
    // break;
    // }
    // case (ConnectionType::Manual): {
    //     client = ManualRAO();
    // break;

    // }
    default: {
        gzwarn << "WaveRaoPlugin::createConnection " << name
               << " failed to create connection." << std::endl;
        return nullptr;
    }
    }
    client->createConnection(entity, name, sdf);
    return client;
}

bool WaveRaoPlugin::vesselTransition(
    gz::sim::Entity _vessel, DomainType _new_mode)
{
    if (m_vehicle_current_mode[_vessel] == _new_mode) {
        return true;
    }
    else if (_new_mode == DomainType::Aerial) {
        if (m_aerial_interface.find(_vessel) != m_aerial_interface.end()) {
            m_vehicle_current_mode[_vessel] = _new_mode;
            m_current_vessel_interface[_vessel]->deactivateConnection(_vessel);
            m_current_vessel_interface[_vessel] = m_aerial_interface[_vessel];
            m_current_vessel_interface[_vessel]->activateConnection(_vessel);
        }
        else {
            gzwarn << m_vessels_name_map[_vessel]
                   << "requrested to switch domain to "
                      "aerial but no "
                      "connection "
                      "is set. Ignoring transition.\n";
            return false;
        }
    }
    else if (_new_mode == DomainType::Surface) {
        if (m_surface_interface.find(_vessel) != m_surface_interface.end()) {
            m_vehicle_current_mode[_vessel] = _new_mode;
            m_current_vessel_interface[_vessel]->deactivateConnection(_vessel);
            m_current_vessel_interface[_vessel] = m_surface_interface[_vessel];
            m_current_vessel_interface[_vessel]->activateConnection(_vessel);
        }
        else {
            gzwarn << m_vessels_name_map[_vessel]
                   << "requrested to switch domain to "
                      "surface but no "
                      "connection "
                      "is set. Ignoring transition.\n";
            return false;
        }
    }
    else if (_new_mode == DomainType::Underwater) {
        if (m_underwater_interface.find(_vessel) !=
            m_underwater_interface.end()) {
            m_vehicle_current_mode[_vessel] = _new_mode;
            m_current_vessel_interface[_vessel]->deactivateConnection(_vessel);
            m_current_vessel_interface[_vessel] =
                m_underwater_interface[_vessel];
            m_current_vessel_interface[_vessel]->activateConnection(_vessel);
        }
        else {
            gzwarn << m_vessels_name_map[_vessel]
                   << "requrested to switch domain to "
                      "surface but no "
                      "connection "
                      "is set. Ignoring transition.\n";
            return false;
        }
    }
    return true;
}
} // namespace gazebo
} // namespace liquidai
GZ_ADD_PLUGIN(
    liquidai::gazebo::WaveRaoPlugin,
    gz::sim::System,
    liquidai::gazebo::WaveRaoPlugin::ISystemConfigure,
    liquidai::gazebo::WaveRaoPlugin::ISystemPreUpdate,
    liquidai::gazebo::WaveRaoPlugin::ISystemUpdate)