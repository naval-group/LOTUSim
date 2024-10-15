#include "PhysicsInterfacePlugin.h"

namespace lotusim::gazebo {
using namespace std::placeholders;

bool pose3Eql(const gz::math::Pose3d &_a, const gz::math::Pose3d &_b)
{
    return _a.Pos().Equal(_b.Pos(), 1e-6) &&
           gz::math::equal(_a.Rot().X(), _b.Rot().X(), 1e-6) &&
           gz::math::equal(_a.Rot().Y(), _b.Rot().Y(), 1e-6) &&
           gz::math::equal(_a.Rot().Z(), _b.Rot().Z(), 1e-6) &&
           gz::math::equal(_a.Rot().W(), _b.Rot().W(), 1e-6);
}

void shuffleOrder(std::vector<uint64_t> &_entities, RandomisedType _type)
{
    switch (_type) {
        case (RandomisedType::RANDOM): {
            std::random_device rd;
            std::default_random_engine rng(rd());
            std::shuffle(_entities.begin(), _entities.end(), rng);
        }
        default: {
        }
    }
}

PhysicsInterfacePlugin::PhysicsInterfacePlugin()
{
    m_gz_node = std::make_shared<gz::transport::Node>();

    m_logger = logger::createConsoleAndFileLogger(
        "physics_interface_plugin",
        "physics_interface_plugin.txt");
}

PhysicsInterfacePlugin::~PhysicsInterfacePlugin() {}

void PhysicsInterfacePlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr)
{
    m_entity = _entity;
    m_logger->debug("PhysicsInterfacePlugin debug initiated");
    auto sdfPtr = const_cast<sdf::Element *>(_sdf.get());
}

void PhysicsInterfacePlugin::Update(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
    _ecm.EachNew<gz::sim::components::ModelSdf>(
        std::bind(&PhysicsInterfacePlugin::LoadVessel, this, _1, _2, &_ecm));

    _ecm.EachRemoved<gz::sim::components::ModelSdf>(
        std::bind(&PhysicsInterfacePlugin::DeleteVessel, this, _1, _2, &_ecm));

    if (_info.dt.count() == 0) {
        return;
    }

    float target_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(_info.dt).count();

    // TODO: hardcoded for now. To change to a variable in sdf
    shuffleOrder(m_vessels_entities, RandomisedType::RANDOM);

    for (auto &&vessel_entity : m_vessels_entities) {
        std::string vessel_name = m_vessels_name_map[vessel_entity];
        if (m_current_vessel_interface.find(vessel_entity) ==
            m_current_vessel_interface.end()) {
            m_logger->warn(
                "PhysicsInterfacePlugin::Update: {} update failed. Unable for find interface",
                vessel_name);
        }

        std::chrono::_V2::system_clock::time_point start_time =
            std::chrono::system_clock::now();
        gz::math::Pose3d pose =
            _ecm.Component<gz::sim::components::Pose>(vessel_entity)->Data();
        gz::sim::Link _link(m_vessels_base_link_map[vessel_name]);
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
            m_current_vessel_interface[vessel_entity]->getNewState(
                vessel_entity,
                vessel_info,
                target_time);

        // Print physics engine response time if compiled in debug mode
        if (m_logger->level() < spdlog::level::info) {
            m_logger->debug(
                "PhysicsInterfacePlugin::Update: Phyiscs update time: {}",
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now() - start_time)
                    .count());
        }

        if (update_opt) {
            if (!vesselTransition(
                    vessel_entity,
                    std::get<1>(update_opt.value()))) {
                m_logger->error(
                    "PhysicsInterfacePlugin::Update: Vessel {} transition failed",
                    vessel_name);
            }

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
                update["u"].back(),
                update["v"].back(),
                update["w"].back()};

            auto ang_vel = gz::math::Vector3d{
                update["p"].back(),
                update["q"].back(),
                update["r"].back()};

            auto poseCmdComp =
                _ecm.Component<gz::sim::components::WorldPoseCmd>(
                    vessel_entity);

            if (!poseCmdComp) {
                _ecm.CreateComponent(
                    vessel_entity,
                    gz::sim::components::WorldPoseCmd(pose));
            } else {
                auto state = poseCmdComp->SetData(pose, pose3Eql)
                                 ? gz::sim::ComponentState::OneTimeChange
                                 : gz::sim::ComponentState::NoChange;
                _ecm.SetChanged(
                    vessel_entity,
                    gz::sim::components::WorldPoseCmd::typeId,
                    state);
            }

            _ecm.SetChanged(
                vessel_entity,
                gz::sim::components::Pose::typeId,
                gz::sim::ComponentState::OneTimeChange);

            _ecm.SetComponentData<gz::sim::components::WorldLinearVelocity>(
                m_vessels_base_link_map[vessel_name],
                lin_vel);
            const auto angularVel =
                _ecm.Component<gz::sim::components::WorldAngularVelocity>(
                    m_vessels_base_link_map[vessel_name]);
            *angularVel = gz::sim::components::WorldAngularVelocity(ang_vel);
        } else {
            m_logger->warn(
                "PhysicsInterfacePlugin::Update: {} update failed.",
                vessel_entity);
        }
    }
}

std::shared_ptr<PhysicsInterfaceBase> PhysicsInterfacePlugin::createConnection(
    const gz::sim::Entity &entity,
    const std::string &name,
    const ConnectionType &protocol_type,
    const sdf::ElementPtr sdf)
{
    std::shared_ptr<PhysicsInterfaceBase> client;

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
            m_logger->warn(
                "PhysicsInterfacePlugin::createConnection: {} failed to create connection.",
                name);
            return nullptr;
        }
    }
    client->createConnection(entity, name, sdf);
    client->setLogger(m_logger);
    return client;
}

bool PhysicsInterfacePlugin::vesselTransition(
    gz::sim::Entity _vessel,
    DomainType _new_mode)
{
    if (m_vehicle_current_mode[_vessel] == _new_mode) {
        return true;
    } else if (_new_mode == DomainType::Aerial) {
        if (m_aerial_interface.find(_vessel) != m_aerial_interface.end()) {
            m_vehicle_current_mode[_vessel] = _new_mode;
            m_current_vessel_interface[_vessel]->deactivateConnection(_vessel);
            m_current_vessel_interface[_vessel] = m_aerial_interface[_vessel];
            m_current_vessel_interface[_vessel]->activateConnection(_vessel);
        } else {
            m_logger->warn(
                "PhysicsInterfacePlugin::vesselTransition: {} requested to switch domain to aerial but no connection is set. Ignoring transition.",
                m_vessels_name_map[_vessel]);
            return false;
        }
    } else if (_new_mode == DomainType::Surface) {
        if (m_surface_interface.find(_vessel) != m_surface_interface.end()) {
            m_vehicle_current_mode[_vessel] = _new_mode;
            m_current_vessel_interface[_vessel]->deactivateConnection(_vessel);
            m_current_vessel_interface[_vessel] = m_surface_interface[_vessel];
            m_current_vessel_interface[_vessel]->activateConnection(_vessel);
        } else {
            m_logger->warn(
                "PhysicsInterfacePlugin::vesselTransition: {} requested to switch domain to surface but no connection is set. Ignoring transition.",
                m_vessels_name_map[_vessel]);
            return false;
        }
    } else if (_new_mode == DomainType::Underwater) {
        if (m_underwater_interface.find(_vessel) !=
            m_underwater_interface.end()) {
            m_vehicle_current_mode[_vessel] = _new_mode;
            m_current_vessel_interface[_vessel]->deactivateConnection(_vessel);
            m_current_vessel_interface[_vessel] =
                m_underwater_interface[_vessel];
            m_current_vessel_interface[_vessel]->activateConnection(_vessel);
        } else {
            m_logger->warn(
                "PhysicsInterfacePlugin::vesselTransition: {} requested to switch domain to surface but no connection is set. Ignoring transition.",
                m_vessels_name_map[_vessel]);
            return false;
        }
    }
    return true;
}

bool PhysicsInterfacePlugin::LoadVessel(
    const gz::sim::Entity &_entity,
    const gz::sim::components::ModelSdf *_model,
    gz::sim::EntityComponentManager *_ecm)
{
    gz::sim::Entity base_link;
    sdf::Model data = _model->Data();
    sdf::ElementPtr sdfptr = data.Element();
    auto name_opt = _ecm->Component<gz::sim::components::Name>(_entity);
    std::string vessel_name;
    if (name_opt) {
        vessel_name = name_opt->Data();
    } else {
        m_logger->warn(
            "PhysicsInterfacePlugin::LoadVessel: No vessel name found. Ignoring model.");
        return true;
    }

    if (sdfptr->HasElement("physics_server_interface")) {
        m_logger->debug(sdfptr->ToString(""));
        m_vessels_entities.push_back(_entity);
        // Get Base Link. Currently a fix as only base_link can get
        // model speed
        m_vessels_model_map[vessel_name] = _entity;
        m_vessels_name_map[_entity] = vessel_name;

        auto child_link =
            _ecm->ChildrenByComponents(_entity, gz::sim::components::Link());
        for (auto &&link : child_link) {
            auto name_opt = _ecm->Component<gz::sim::components::Name>(link);
            if (name_opt &&
                name_opt->Data().find("base_link") != std::string::npos) {
                base_link = link;
                m_vessels_base_link_map[vessel_name] = base_link;
                gz::sim::Link _link(base_link);
                _link.EnableVelocityChecks(*_ecm);
            }
        }

        // Init Interface
        sdf::ElementPtr physics_sdf_ptr =
            sdfptr->GetElement("physics_server_interface");
        m_logger->info(
            "PhysicsInterfacePlugin::LoadVessel: {} physics update request detected.",
            vessel_name);

        // If vessel has aerial component
        if (physics_sdf_ptr->HasElement("aerial")) {
            sdf::ElementPtr aerial_sdf = physics_sdf_ptr->GetElement("aerial");

            ConnectionType aerial_connection_type;

            if (!aerial_sdf->HasElement("ConnectionType") ||
                !aerial_sdf->HasElement("uri")) {
                m_logger->warn(
                    "PhysicsInterfacePlugin::LoadVessel: {} missing aerial connectionType or uri. Removing aerial calculation.",
                    vessel_name);
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
        if (physics_sdf_ptr->HasElement("surface")) {
            sdf::ElementPtr surface_sdf =
                physics_sdf_ptr->GetElement("surface");

            ConnectionType surface_connection_type;
            if (!surface_sdf->HasElement("ConnectionType") ||
                !surface_sdf->HasElement("uri")) {
                m_logger->warn(
                    "PhysicsInterfacePlugin::LoadVessel: {} missing surface connectionType or uri. Removing surface calculation.",
                    vessel_name);
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
        if (physics_sdf_ptr->HasElement("underwater")) {
            sdf::ElementPtr underwater_sdf =
                physics_sdf_ptr->GetElement("underwater");

            ConnectionType underwater_connection_type;

            if (!underwater_sdf->HasElement("ConnectionType") ||
                !underwater_sdf->HasElement("uri")) {
                m_logger->warn(
                    "PhysicsInterfacePlugin::LoadVessel: {} missing surface connectionType or uri. Removing surface calculation.",
                    vessel_name);
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
        if (physics_sdf_ptr->HasElement("init_state")) {
            DomainType init_domain =
                DomainTypeMap[physics_sdf_ptr->Get<std::string>("init_state")];

            switch (init_domain) {
                case (DomainType::Aerial): {
                    if (m_aerial_interface.find(_entity) !=
                        m_aerial_interface.end()) {
                        m_current_vessel_interface[_entity] =
                            m_aerial_interface[_entity];
                        m_vehicle_current_mode[_entity] = DomainType::Aerial;
                    } else {
                        // Handle the case where state of vessel has no engine
                        // defined
                        m_logger->error(
                            "PhysicsInterfacePlugin::LoadVessel: {} init state aerial. No aerial set. Failed.",
                            vessel_name);
                    }
                    break;
                }
                case (DomainType::Surface): {
                    if (m_surface_interface.find(_entity) !=
                        m_surface_interface.end()) {
                        m_current_vessel_interface[_entity] =
                            m_surface_interface[_entity];
                        m_vehicle_current_mode[_entity] = DomainType::Surface;
                    } else {
                        m_logger->error(
                            "PhysicsInterfacePlugin::LoadVessel: {} init state aerial. No surface set. Failed.",
                            vessel_name);
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
                    } else {
                        m_logger->error(
                            "PhysicsInterfacePlugin::LoadVessel: {} init state aerial. No underwater set. Failed.",
                            vessel_name);
                    }
                    break;
                }
                default: {
                    m_logger->error(
                        "PhysicsInterfacePlugin::LoadVessel: {} init state not found.",
                        vessel_name);
                    return true;
                }
            }
            m_current_vessel_interface[_entity]->activateConnection(_entity);
        }
    } else {
        m_logger->warn(
            "PhysicsInterfacePlugin::LoadVessel: Not calculating phyics for {}",
            vessel_name);
    }

    // Add the uri and domain info
    m_logger->info(
        "PhysicsInterfacePlugin::LoadVessel: {} physics successfully created.",
        vessel_name);
    return true;
}

bool PhysicsInterfacePlugin::DeleteVessel(
    const gz::sim::Entity &_entity,
    const gz::sim::components::ModelSdf *_model,
    gz::sim::EntityComponentManager *_ecm)
{
    try {  // Deactivate connection
        if (m_vessels_name_map.find(_entity) != m_vessels_name_map.end()) {
            m_logger->info(
                "PhysicsInterfacePlugin::DeleteVessel: Removing vessel {}",
                m_vessels_name_map[_entity]);

            if (m_current_vessel_interface.find(_entity) !=
                m_current_vessel_interface.end()) {
                m_current_vessel_interface[_entity]->deactivateConnection(
                    _entity);
                m_logger->info(
                    "PhysicsInterfacePlugin::DeleteVessel: Deactivated vessel {} physics engine connection",
                    m_vessels_name_map[_entity]);
            } else {
                m_logger->warn(
                    "PhysicsInterfacePlugin::DeleteVessel: Vessel {} has no physics engine connection",
                    m_vessels_name_map[_entity]);
            }
        } else {
            m_logger->warn(
                "PhysicsInterfacePlugin::DeleteVessel: Removing vessel {}. No vessel name found, vessel not part of the system.",
                _entity);
        }

        // Removing entity
        auto it = std::find(
            m_vessels_entities.begin(),
            m_vessels_entities.end(),
            _entity);
        if (it != m_vessels_entities.end()) {
            m_vessels_entities.erase(it);
        }
        m_vessels_base_link_map.erase(m_vessels_name_map[_entity]);
        m_vessels_model_map.erase(m_vessels_name_map[_entity]);
        m_vessels_name_map.erase(_entity);
        m_current_vessel_interface.erase(_entity);
        m_vehicle_current_mode.erase(_entity);
        m_aerial_interface.erase(_entity);
        m_surface_interface.erase(_entity);
        m_underwater_interface.erase(_entity);
        return true;
    } catch (const std::runtime_error &e) {  // Catch the specific exception
        m_logger->error(
            "PhysicsInterfacePlugin::DeleteVessel: Failed for vessel {}. \nReason:{}",
            _entity,
            e.what());
    } catch (...) {
        m_logger->error(
            "PhysicsInterfacePlugin::DeleteVessel: Failed for vessel {} for unknown reason.",
            _entity);
    }
}

}  // namespace lotusim::gazebo
GZ_ADD_PLUGIN(
    lotusim::gazebo::PhysicsInterfacePlugin,
    gz::sim::System,
    lotusim::gazebo::PhysicsInterfacePlugin::ISystemConfigure,
    lotusim::gazebo::PhysicsInterfacePlugin::ISystemUpdate)