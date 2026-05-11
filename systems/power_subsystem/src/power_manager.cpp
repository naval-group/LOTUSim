#include "power_subsystem/power_manager.hpp"
 
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/common/Console.hh>
#include "lotusim_common/common.hpp"
 
GZ_ADD_PLUGIN(
    lotusim::gazebo::PowerManager,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemUpdate)
 
namespace lotusim::gazebo
{

PowerManager::PowerManager() = default;
PowerManager::~PowerManager() = default;

void PowerManager::Configure(
    const gz::sim::Entity& _entity,
    const std::shared_ptr<const sdf::Element>& /*_sdf*/,
    gz::sim::EntityComponentManager& _ecm,
    gz::sim::EventManager& /*_eventMgr*/)
{
    m_entity = _entity;
    m_ecm    = &_ecm;
 
    m_world_name = lotusim::common::getWorldName(_ecm);
 
    const std::string logger_name = m_world_name + "_power_manager";
    m_logger = logger::createConsoleAndFileLogger(logger_name, logger_name + ".txt");
 
    // world-level ROS2 node used for publishing
    // per-vessel nodes are created inside each PowerManagerInstance
    m_ros_node = rclcpp::Node::make_shared(
        m_world_name + "_power_manager",
        m_world_name);
 
    m_logger->info("PowerManager::Configure: power subsystem started for world [{}]", m_world_name);
}

void PowerManager::Update(
    const gz::sim::UpdateInfo& _info,
    gz::sim::EntityComponentManager& _ecm)
{
    // ── Detect new vessels ────────────────────────────────────────────
    _ecm.EachNew<gz::sim::components::Model,
                 gz::sim::components::ModelSdf>(
        [&](const gz::sim::Entity& _entity,
            const gz::sim::components::Model*,
            const gz::sim::components::ModelSdf* _model) -> bool
        {
            loadVessel(_entity, _model, _ecm);
            return true;
        });
 
    // ── Detect removed vessels ────────────────────────────────────────
    _ecm.EachRemoved<gz::sim::components::Model>(
        [&](const gz::sim::Entity& _entity,
            const gz::sim::components::Model*) -> bool
        {
            deleteVessel(_entity);
            return true;
        });
 
    // ── Tick all active vessel instances ──────────────────────────────
    for (auto& [entity, instance] : m_vessel_instances) {
        instance->Update(_info, _ecm);
    }
}

// ============================================================
// Private helpers
// ============================================================

bool PowerManager::loadVessel(
    const gz::sim::Entity& _entity,
    const gz::sim::components::ModelSdf* /*_model*/,
    gz::sim::EntityComponentManager& _ecm)
{
    // get vessel name
    auto nameOpt = lotusim::common::getModelName(_ecm, _entity);
    if (!nameOpt) {
        m_logger->warn(
            "PowerManager::loadVessel: could not get name for entity [{}]", _entity);
        return false;
    }
    const std::string vesselName = nameOpt->second;
 
    // skip if already registered
    if (m_vessel_instances.count(_entity)) {
        m_logger->warn("PowerManager::loadVessel: vessel [{}] already registered", vesselName);
        return false;
    }
 
    // check vessel has at least one <lotusim_power> link before creating
    // an instance
    const auto* modelSdfComp = _ecm.Component<gz::sim::components::ModelSdf>(_entity);
    if (!modelSdfComp) {
        m_logger->error(
            "PowerManagerInstance [{}]: no ModelSdf component found", vesselName);
        return false;
    }

    // Walk <link> elements in the SDF
    const sdf::ElementPtr modelEl = modelSdfComp->Data().Element();
    if (!modelEl) {
        m_logger->error(
            "PowerManagerInstance [{}]: ModelSdf has no element", vesselName);
        return false;
    }

    bool hasPowerConfig = false;
    auto linkEl = modelEl->GetElement("link");
    while (linkEl) {
        // Check for <lotusim_power> tag
        if (linkEl->HasElement("lotusim_power")) {
            hasPowerConfig = true;
            break;
        }
        linkEl = linkEl->GetNextElement("link");
    }

    if (!hasPowerConfig) {
        m_logger->warn(
            "PowerManager::loadVessel [{}]: no <lotusim_power> tag found, skipping PowerManager creation",
            vesselName);
        return false;
    }
 
    // creates per-vessel node
    auto vesselNode = rclcpp::Node::make_shared(
        vesselName + "_power",
        m_world_name);
 
    // instance & constructor parses providers and consumers from ECM
    auto instance = std::make_unique<PowerManagerInstance>(_entity, vesselName, vesselNode, _ecm);
 
    m_vessel_instances.emplace(_entity, std::move(instance));
 
    m_logger->info("PowerManager::loadVessel: registered vessel [{}]", vesselName);
 
    return true;
}

bool PowerManager::deleteVessel(const gz::sim::Entity& _entity)
{
    auto it = m_vessel_instances.find(_entity);
    if (it == m_vessel_instances.end()) {
        return false;
    }
 
    m_logger->info("PowerManager::deleteVessel: removing vessel [{}]", it->second->vesselName());
 
    m_vessel_instances.erase(it);
    return true;
}

}