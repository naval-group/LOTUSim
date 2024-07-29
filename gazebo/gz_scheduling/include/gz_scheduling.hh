#pragma once

#include <gz/msgs.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <random>
#include <sdf/Element.hh>
#include <sdf/Material.hh>
#include <sdf/sdf.hh>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include "generic_effector.hh"
#include "pose_effector.hh"

namespace scheduling_plugin {
class gz_scheduling :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemUpdate,
    public gz::sim::ISystemPostUpdate
// public gz::sim::ISystemReset
{
private:
    float previousSimTime;
    std::string worldName;
    gz::sim::EntityComponentManager *ecm_;
    gz::sim::Entity entity;
    gz::transport::Node node;
    std::shared_ptr<gz::transport::Node> m_gz_node;
    gz::transport::Node::Publisher pose_sensor, agent_demo_sched,
        plugin_demo_sched;
    std::vector<std::shared_ptr<GenericEffector>> effectors;
    bool m_debug;

public:
    gz_scheduling();

    ~gz_scheduling() override;

    void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_eventMgr) override;

    void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) override;

    void Update(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) override;

    void PostUpdate(
        const gz::sim::UpdateInfo &_info,
        const gz::sim::EntityComponentManager &_ecm) override;

    // public: void Reset(const gz::sim::UpdateInfo &_info,
    //              gz::sim::EntityComponentManager &_ecm) override;

    void PoseEffectorCallback(const gz::msgs::Pose &msg);
};
} // namespace scheduling_plugin