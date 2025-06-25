#include "PhysicsInterfacePlugin.h"

namespace lotusim::gazebo {
using namespace std::placeholders;

PhysicsInterfacePlugin::PhysicsInterfacePlugin()
{
    m_vessels_cmd_map_ptr =
        std::make_shared<std::unordered_map<gz::sim::Entity, std::string>>();

    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
}

PhysicsInterfacePlugin::~PhysicsInterfacePlugin()
{
    m_logger->info(
        "PhysicsInterfacePlugin::~PhysicsInterfacePlugin: PhysicsInterfacePlugin successfully shutdown.");
}

void PhysicsInterfacePlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr)
{
    m_entity = _entity;
    auto sdfPtr = const_cast<sdf::Element *>(_sdf.get());

    m_world_name = lotusim::common::getWorldName(_ecm);
    m_logger = logger::createConsoleAndFileLogger(
        "physics_interface_plugin",
        m_world_name + "_physics_interface_plugin.txt");

    m_ros_node = rclcpp::Node::make_shared("physics_plugin", m_world_name);
    m_cmd_array_sub = m_ros_node->create_subscription<
        lotusim_msgs::msg::VesselCmdArray>(
        "lotusim_vessel_array_cmd",
        10,
        [this](lotusim_msgs::msg::VesselCmdArray::ConstSharedPtr msgs) -> void {
            for (auto &&msg : msgs->cmds) {
                int entity;
                if (msg.vessel_name.empty() && msg.entity) {
                    entity = msg.entity;
                } else if (
                    m_vessels_model_map.find(msg.vessel_name) !=
                    m_vessels_model_map.end()) {
                    entity = m_vessels_model_map[msg.vessel_name];
                } else {
                    m_logger->error(
                        "PhysicsInterfacePlugin::Topic lotusim_vessel_cmd callback failed. No known entity: {}, {}",
                        msg.entity,
                        msg.vessel_name);
                }
                (*m_vessels_cmd_map_ptr)[entity] = std::move(msg.cmd_string);
            }
        });

    m_logger->debug(
        "PhysicsInterfacePlugin debug initiated\n {}",
        _sdf->ToString(""));
}

void PhysicsInterfacePlugin::Update(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
    _ecm.EachNew<gz::sim::components::ModelSdf>(
        std::bind(&PhysicsInterfacePlugin::LoadVessel, this, _1, _2, &_ecm));

    _ecm.EachRemoved<gz::sim::components::ModelSdf>(
        std::bind(&PhysicsInterfacePlugin::DeleteVessel, this, _1, _2, &_ecm));

    if (rclcpp::ok()) {
        rclcpp::executors::MultiThreadedExecutor m_executor;
        m_executor.add_node(m_ros_node);
        m_executor.spin_some();
    } else {
        m_logger->error(
            "PhysicsInterfacePlugin::Update: RCLCPP context shutdown.");
    }

    if (_info.dt.count() == 0) {
        return;
    }

    float target_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(_info.dt).count();

    // TODO: hardcoded for now. To change to a variable in sdf
    lotusim::common::shuffleOrder<gz::sim::Entity>(
        m_vessels_entities,
        lotusim::common::RandomisedType::RANDOM);

    for (auto &&vessel_entity : m_vessels_entities) {
        // Bound checking for all map used
        if (m_current_vessel_interface.find(vessel_entity) ==
                m_current_vessel_interface.end() &&
            m_vessels_name_map.find(vessel_entity) ==
                m_vessels_name_map.end() &&
            m_vessels_base_link_map.find(m_vessels_name_map[vessel_entity]) ==
                m_vessels_base_link_map.end()) {
            m_logger->warn(
                "PhysicsInterfacePlugin::Update: Entity {} update failed. Unable for find mappings",
                vessel_entity);
        }
        std::string vessel_name = m_vessels_name_map[vessel_entity];
        try {
            std::chrono::_V2::system_clock::time_point start_time =
                std::chrono::system_clock::now();

            // TODO: Temp fix measure. Test if the worldPose component matters
            // and if the rest of the child / sensors links follow along and
            // update to give good sensor reading
            gz::math::Pose3d pose;
            auto pose_comp =
                _ecm.Component<gz::sim::components::Pose>(vessel_entity);
            if (pose_comp) {
                pose = pose_comp->Data();
            } else {
                m_logger->error(
                    "PhysicsInterfacePlugin::Update: unable to get pose component");
            }

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

                // TODO: Figure out worldposecmd
                bool res = _ecm.SetComponentData<gz::sim::components::Pose>(
                    vessel_entity,
                    pose);
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
                *angularVel =
                    gz::sim::components::WorldAngularVelocity(ang_vel);
            } else {
                m_logger->warn(
                    "PhysicsInterfacePlugin::Update: {} update failed.",
                    vessel_entity);
            }
        } catch (const std::exception &e) {
            m_logger->warn(
                "PhysicsInterfacePlugin::Update: Error update for {}\n{}.\n.",
                vessel_name,
                e.what());
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
    client->setSharedCmd(m_vessels_cmd_map_ptr);
    client->createConnection(entity, name, sdf);
    client->setLogger(m_logger);
    return client;
}

bool PhysicsInterfacePlugin::vesselTransition(
    gz::sim::Entity _vessel,
    DomainType _new_mode)
{
    std::shared_ptr<PhysicsInterfaceBase> new_interface;
    try {
        if (m_vehicle_current_mode[_vessel] == _new_mode) {
            return true;
        } else if (_new_mode == DomainType::Aerial) {
            if (m_aerial_interface.find(_vessel) != m_aerial_interface.end()) {
                new_interface = m_aerial_interface[_vessel];
                new_interface->activateConnection(_vessel);
            } else {
                throw std::runtime_error(
                    fmt::format(
                        "{} requested to switch domain to aerial but no defined connection in that domain. Ignoring transition.",
                        m_vessels_name_map[_vessel]));
            }
        } else if (_new_mode == DomainType::Surface) {
            if (m_surface_interface.find(_vessel) !=
                m_surface_interface.end()) {
                new_interface = m_surface_interface[_vessel];
                new_interface->activateConnection(_vessel);
            } else {
                throw std::runtime_error(
                    fmt::format(
                        "{} requested to switch domain to Surface but no defined connection in that domain. Ignoring transition.",
                        m_vessels_name_map[_vessel]));
            }
        } else if (_new_mode == DomainType::Underwater) {
            if (m_underwater_interface.find(_vessel) !=
                m_underwater_interface.end()) {
                new_interface = m_underwater_interface[_vessel];
                new_interface->activateConnection(_vessel);
            } else {
                throw std::runtime_error(
                    fmt::format(
                        "{} requested to switch domain to Underwater but no defined connection in that domain. Ignoring transition.",
                        m_vessels_name_map[_vessel]));
            }
        }
    } catch (const std::exception &e) {
        m_logger->warn(
            "PhysicsInterfacePlugin::vesselTransition: {}.\nReverting to previous physics engine.",
            e.what());
        return false;
    }
    m_vehicle_current_mode[_vessel] = _new_mode;
    m_current_vessel_interface[_vessel]->deactivateConnection(_vessel);
    m_current_vessel_interface[_vessel] = new_interface;
    return true;
}

bool PhysicsInterfacePlugin::LoadVessel(
    const gz::sim::Entity &_entity,
    const gz::sim::components::ModelSdf *_model,
    gz::sim::EntityComponentManager *_ecm)
{
    try {
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

        auto includeptr = sdfptr->GetIncludeElement();
        // The lotus param will either be include statement or part of the model
        if (!includeptr) {
            includeptr = sdfptr;
        }

        if (includeptr->HasElement("lotus_param") &&
            includeptr->GetElement("lotus_param")
                ->HasElement("physics_engine_interface")) {
            m_logger->debug(includeptr->ToString(""));
            m_vessels_entities.push_back(_entity);
            // Get Base Link. Currently a fix as only base_link can get
            // model speed
            m_vessels_model_map[vessel_name] = _entity;
            m_vessels_name_map[_entity] = vessel_name;

            auto child_link = _ecm->ChildrenByComponents(
                _entity,
                gz::sim::components::Link());
            for (auto &&link : child_link) {
                auto name_opt =
                    _ecm->Component<gz::sim::components::Name>(link);
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
                includeptr->GetElement("lotus_param")
                    ->GetElement("physics_engine_interface");
            m_logger->info(
                "PhysicsInterfacePlugin::LoadVessel: {} physics update request detected.",
                vessel_name);

            // If vessel has aerial component
            if (physics_sdf_ptr->HasElement("aerial")) {
                sdf::ElementPtr aerial_sdf =
                    physics_sdf_ptr->GetElement("aerial");

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
                    DomainTypeMap[physics_sdf_ptr->Get<std::string>(
                        "init_state")];

                switch (init_domain) {
                    case (DomainType::Aerial): {
                        if (m_aerial_interface.find(_entity) !=
                            m_aerial_interface.end()) {
                            m_current_vessel_interface[_entity] =
                                m_aerial_interface[_entity];
                            m_vehicle_current_mode[_entity] =
                                DomainType::Aerial;
                        } else {
                            throw std::runtime_error(
                                fmt::format(
                                    "{} Init state aerial. No aerial physics engine set.",
                                    vessel_name));
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
                        } else {
                            throw std::runtime_error(
                                fmt::format(
                                    "{} Init state surface. No surface physics engine set.",
                                    vessel_name));
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
                            throw std::runtime_error(
                                fmt::format(
                                    "{} Init state underwater. No underwater physics engine set.",
                                    vessel_name));
                        }
                        break;
                    }
                    default: {
                        throw std::runtime_error(
                            fmt::format(
                                "Failed to find {} init state",
                                vessel_name));
                    }
                }
                m_current_vessel_interface[_entity]->activateConnection(
                    _entity);
            }
            m_logger->info(
                "PhysicsInterfacePlugin::LoadVessel: {} physics successfully created.",
                vessel_name);
        } else {
            m_logger->warn(
                "PhysicsInterfacePlugin::LoadVessel: Not calculating phyics for {}",
                vessel_name);
        }
        return true;
    } catch (const std::exception &e) {
        m_logger->error(
            "PhysicsInterfacePlugin::LoadVessel: {}. Loading failed.",
            e.what());
    } catch (...) {
        m_logger->error(
            "PhysicsInterfacePlugin::LoadVessel: Unknown error. Failed.");
    }
    DeleteVessel(_entity, _model, _ecm);  // cleanup
    return false;
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