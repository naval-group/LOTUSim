#include "physics_engine_interface/physics_interface_plugin.hpp"

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
    const gz::sim::Entity& _entity,
    const std::shared_ptr<const sdf::Element>& _sdf,
    gz::sim::EntityComponentManager& _ecm,
    gz::sim::EventManager&)
{
    m_entity = _entity;

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
            for (auto&& msg : msgs->cmds) {
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
                    continue;
                }
                (*m_vessels_cmd_map_ptr)[entity] = std::move(msg.cmd_string);
            }
        });

    m_logger->debug(
        "PhysicsInterfacePlugin debug initiated\n {}",
        _sdf->ToString(""));
}

void PhysicsInterfacePlugin::Update(
    const gz::sim::UpdateInfo& _info,
    gz::sim::EntityComponentManager& _ecm)
{
    _ecm.EachNew<gz::sim::components::ModelSdf>(
        std::bind(&PhysicsInterfacePlugin::loadVessel, this, _1, _2, &_ecm));

    _ecm.EachRemoved<gz::sim::components::ModelSdf>(
        std::bind(&PhysicsInterfacePlugin::deleteVessel, this, _1, _2, &_ecm));

    if (rclcpp::ok()) {
        rclcpp::spin_some(m_ros_node);
    } else {
        m_logger->error(
            "PhysicsInterfacePlugin::Update: RCLCPP context shutdown.");
    }

    if (_info.dt.count() == 0) {
        return;
    }

    // Randomise the order of vessel update to avoid bias
    lotusim::common::shuffleOrder<gz::sim::Entity>(
        m_vessels_entities,
        lotusim::common::RandomisedType::RANDOM);
    std::vector<std::future<void>> futures;

    std::chrono::_V2::system_clock::time_point start_time =
        std::chrono::system_clock::now();

    for (auto&& vessel_entity : m_vessels_entities) {
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
        futures.push_back(
            std::async(
            std::launch::async,
            &PhysicsInterfacePlugin::updateVesselState,
            this,
            vessel_entity,
            _info,
            std::ref(_ecm)));
    }
    for (auto& fut : futures) {
        fut.get();
    }
    if (m_logger->level() < spdlog::level::info) {
        m_logger->debug(
            "PhysicsInterfacePlugin::Update: Overall Phyiscs update time: {}",
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now() - start_time)
                .count());
    }
}

void PhysicsInterfacePlugin::updateVesselState(
    const gz::sim::Entity& vessel_entity,
    const gz::sim::UpdateInfo& _info,
    gz::sim::EntityComponentManager& _ecm)
{
    std::string vessel_name = m_vessels_name_map[vessel_entity];
    float target_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(_info.dt).count();
    try {
        std::chrono::_V2::system_clock::time_point start_time =
            std::chrono::system_clock::now();

        // TODO: Temp fix measure. Test if the worldPose component matters
        // and if the rest of the child / sensors links follow along and
        // update to give good sensor reading
        gz::math::Pose3d pose;
        gz::math::Vector3d lin_vel;
        gz::math::Vector3d ang_vel;
        auto pose_comp =
            _ecm.Component<gz::sim::components::Pose>(vessel_entity);
        if (pose_comp) {
            pose = pose_comp->Data();
        } else {
            m_logger->error(
                "PhysicsInterfacePlugin::Update: unable to get pose component for {}",
                vessel_name);
            return;
        }

        gz::sim::Link _link(m_vessels_base_link_map[vessel_name]);

        // Get the linear and angular velocity in world frame, ENU.
        auto lin_vel_opt = _link.WorldLinearVelocity(_ecm);
        auto ang_vel_opt = _link.WorldAngularVelocity(_ecm);

        VesselInformation vessel_info;
        vessel_info.time =
            std::chrono::duration_cast<std::chrono::milliseconds>(_info.simTime)
                .count() /
            1000.0;
        if (lin_vel_opt) {
            vessel_info.lin_vel = lin_vel_opt.value();
        }
        if (ang_vel_opt) {
            vessel_info.ang_vel = ang_vel_opt.value();
        }
        vessel_info.entity = vessel_entity;
        vessel_info.pose = pose;

        std::optional<std::tuple<VesselInformation, DomainType>> update_opt =
            m_current_vessel_interface[vessel_entity]->getNewState(
                vessel_entity,
                vessel_info,
                target_time);

        // Print physics engine response time if compiled in debug mode
        if (m_logger->level() < spdlog::level::info) {
            m_logger->debug(
                "PhysicsInterfacePlugin::Update: {} Phyiscs update time: {}",
                vessel_name,
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

            VesselInformation new_state =
                std::move(std::get<0>(update_opt.value()));

            pose = new_state.pose;
            lin_vel = new_state.lin_vel;
            ang_vel = new_state.ang_vel;

            _ecm.SetComponentData<gz::sim::components::Pose>(
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
            *angularVel = gz::sim::components::WorldAngularVelocity(ang_vel);
        } else {
            m_logger->warn(
                "PhysicsInterfacePlugin::Update: {} update failed.",
                vessel_name);
        }
        return;
    } catch (const std::exception& e) {
        m_logger->warn(
            "PhysicsInterfacePlugin::Update: Error update for entity {}\n{}.\n.",
            vessel_entity,
            e.what());
    }
}

std::shared_ptr<PhysicsInterfaceBase> PhysicsInterfacePlugin::createConnection(
    const gz::sim::Entity& entity,
    const std::string& name,
    const ConnectionType& protocol_type,
    const sdf::ElementPtr sdf)
{
    std::shared_ptr<PhysicsInterfaceBase> client;

    switch (protocol_type) {
        case (ConnectionType::XDynWebSocket): {
            client = XdynWebsocket::getInstance(entity, name);
            break;
        }
        case (ConnectionType::ROS2Interface): {
            client = ROS2Interface::getInstance(sdf);
            break;
        }
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
    } catch (const std::exception& e) {
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

bool PhysicsInterfacePlugin::loadVessel(
    const gz::sim::Entity& _entity,
    const gz::sim::components::ModelSdf* _model,
    gz::sim::EntityComponentManager* _ecm)
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
                "PhysicsInterfacePlugin::loadVessel: No vessel name found. Ignoring model.");
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
            for (auto&& link : child_link) {
                auto name_opt =
                    _ecm->Component<gz::sim::components::Name>(link);
                if (name_opt &&
                    name_opt->Data().find("base_link") != std::string::npos) {
                    base_link = link;
                    m_vessels_base_link_map[vessel_name] = base_link;
                    gz::sim::Link _link(base_link);
                    _link.EnableVelocityChecks(*_ecm);
                    break;
                }
            }

            // Init Interface
            sdf::ElementPtr physics_sdf_ptr =
                includeptr->GetElement("lotus_param")
                    ->GetElement("physics_engine_interface");
            m_logger->info(
                "PhysicsInterfacePlugin::loadVessel: {} physics update request detected.",
                vessel_name);

            // If vessel has aerial component
            sdf::ElementPtr aerial_sdf =
                lotusim::common::getElementCaseInsensitive(
                    physics_sdf_ptr,
                    "aerial");
            if (aerial_sdf) {
                ConnectionType aerial_connection_type;
                if (!aerial_sdf->HasElement("connection_type") ||
                    !aerial_sdf->HasElement("uri")) {
                    m_logger->warn(
                        "PhysicsInterfacePlugin::loadVessel: {} missing aerial connectionType or uri. Removing aerial calculation.",
                        vessel_name);
                }
                try {
                    aerial_connection_type =
                        ConnectionTypeMap.at(lotusim::common::toUpper(
                            aerial_sdf->Get<std::string>("connection_type")));
                    m_aerial_interface[_entity] = createConnection(
                        _entity,
                        vessel_name,
                        aerial_connection_type,
                        aerial_sdf);
                } catch (const std::out_of_range&) {
                    aerial_connection_type = ConnectionType::Unknown;
                    m_logger->warn(
                        "PhysicsInterfacePlugin::loadVessel: {} missing aerial connection_type. Removing aerial calculation.",
                        vessel_name);
                }
            }

            // If vessel has surface component
            sdf::ElementPtr surface_sdf =
                lotusim::common::getElementCaseInsensitive(
                    physics_sdf_ptr,
                    "surface");
            if (surface_sdf) {
                ConnectionType surface_connection_type;
                if (!surface_sdf->HasElement("connection_type") ||
                    !surface_sdf->HasElement("uri")) {
                    m_logger->warn(
                        "PhysicsInterfacePlugin::loadVessel: {} missing surface connection_type or uri. Removing surface calculation.",
                        vessel_name);
                }
                try {
                    surface_connection_type =
                        ConnectionTypeMap.at(lotusim::common::toUpper(
                            surface_sdf->Get<std::string>("connection_type")));
                    m_surface_interface[_entity] = createConnection(
                        _entity,
                        vessel_name,
                        surface_connection_type,
                        surface_sdf);
                } catch (const std::out_of_range&) {
                    surface_connection_type = ConnectionType::Unknown;
                    m_logger->warn(
                        "PhysicsInterfacePlugin::loadVessel: {} missing surface connection_type. Removing surface calculation.",
                        vessel_name);
                }
            }

            // If vessel has underwater component
            sdf::ElementPtr underwater_sdf =
                lotusim::common::getElementCaseInsensitive(
                    physics_sdf_ptr,
                    "underwater");

            if (underwater_sdf) {
                ConnectionType underwater_connection_type;
                if (!underwater_sdf->HasElement("connection_type") ||
                    !underwater_sdf->HasElement("uri")) {
                    m_logger->warn(
                        "PhysicsInterfacePlugin::loadVessel: {} missing underwater connectionType or uri. Removing underwater calculation.",
                        vessel_name);
                }
                try {
                    underwater_connection_type =
                        ConnectionTypeMap.at(lotusim::common::toUpper(
                            underwater_sdf->Get<std::string>(
                                "connection_type")));
                    m_underwater_interface[_entity] = createConnection(
                        _entity,
                        vessel_name,
                        underwater_connection_type,
                        underwater_sdf);
                } catch (const std::out_of_range&) {
                    underwater_connection_type = ConnectionType::Unknown;
                    m_logger->warn(
                        "PhysicsInterfacePlugin::loadVessel: {} missing underwater connection_type. Removing underwater calculation.",
                        vessel_name);
                }
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
                "PhysicsInterfacePlugin::loadVessel: {} physics successfully created.",
                vessel_name);
        } else {
            m_logger->warn(
                "PhysicsInterfacePlugin::loadVessel: Not calculating phyics for {}",
                vessel_name);
        }
        return true;
    } catch (const std::exception& e) {
        m_logger->error(
            "PhysicsInterfacePlugin::loadVessel: {}. Loading failed.",
            e.what());
    } catch (...) {
        m_logger->error(
            "PhysicsInterfacePlugin::loadVessel: Unknown error. Failed.");
    }
    deleteVessel(_entity, _model, _ecm);
    return false;
}

bool PhysicsInterfacePlugin::deleteVessel(
    const gz::sim::Entity& _entity,
    const gz::sim::components::ModelSdf*,
    gz::sim::EntityComponentManager*)
{
    try {  // Deactivate connection
        if (m_vessels_name_map.find(_entity) != m_vessels_name_map.end()) {
            m_logger->info(
                "PhysicsInterfacePlugin::deleteVessel: Removing vessel {}",
                m_vessels_name_map[_entity]);

            if (m_current_vessel_interface.find(_entity) !=
                m_current_vessel_interface.end()) {
                m_current_vessel_interface[_entity]->deactivateConnection(
                    _entity);
                m_logger->info(
                    "PhysicsInterfacePlugin::deleteVessel: Deactivated vessel {} physics engine connection",
                    m_vessels_name_map[_entity]);
            } else {
                m_logger->warn(
                    "PhysicsInterfacePlugin::deleteVessel: Vessel {} has no physics engine connection",
                    m_vessels_name_map[_entity]);
            }
        } else {
            m_logger->warn(
                "PhysicsInterfacePlugin::deleteVessel: Removing vessel {}. No vessel name found, vessel not part of the system.",
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
    } catch (const std::runtime_error& e) {  // Catch the specific exception
        m_logger->error(
            "PhysicsInterfacePlugin::deleteVessel: Failed for vessel {}. \nReason:{}",
            _entity,
            e.what());
    } catch (...) {
        m_logger->error(
            "PhysicsInterfacePlugin::deleteVessel: Failed for vessel {} for unknown reason.",
            _entity);
    }
    return false;
}

}  // namespace lotusim::gazebo
GZ_ADD_PLUGIN(
    lotusim::gazebo::PhysicsInterfacePlugin,
    gz::sim::System,
    lotusim::gazebo::PhysicsInterfacePlugin::ISystemConfigure,
    lotusim::gazebo::PhysicsInterfacePlugin::ISystemUpdate)