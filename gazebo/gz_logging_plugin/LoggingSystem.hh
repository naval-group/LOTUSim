#pragma once

#include <gz/msgs.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>

#include <atomic>
#include <chrono>
#include <csignal>
#include <gz/plugin/Register.hh>
#include <iostream>
#include <sdf/Element.hh>
#include <sdf/Material.hh>
#include <sdf/sdf.hh>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace logging_system {
class LoggingSystem :
    // This class is a system.
    public gz::sim::System,

    public gz::sim::ISystemConfigure,
    // This class also implements the ISystemPreUpdate, ISystemUpdate,
    // and ISystemPostUpdate interfaces.

    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemUpdate,
    public gz::sim::ISystemPostUpdate
// public gz::sim::ISystemReset
{
private:
    bool debug;
    bool has_published_poc_data = false;
    float previousSimTime1, previousRealTime1, previousSimTime2,
        previousRealTime2, previousSimTime3, previousRealTime3;
    std::string worldName;
    gz::math::Color contactColor, defaultColor;
    gz::sim::EntityComponentManager *ecm_;

private:
    gz::sim::Entity entity;

private:
    gz::transport::Node node;

private:
    std::shared_ptr<gz::transport::Node> m_gz_node;

private:
    gz::transport::Node::Publisher nodePub, schedNodePub, pocPub;

public:
    LoggingSystem();

public:
    ~LoggingSystem() override;

public:
    void EditEntityVisualColor(
        const gz::sim::Entity &_entity, gz::math::Color _color);

public:
    void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_eventMgr) override;

public:
    void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) override;

public:
    void Update(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) override;

public:
    void PostUpdate(
        const gz::sim::UpdateInfo &_info,
        const gz::sim::EntityComponentManager &_ecm) override;

    // public: void Reset(const gz::sim::UpdateInfo &_info,
    //              gz::sim::EntityComponentManager &_ecm) override;

public:
    void slowDownCallback(const gz::msgs::Boolean &_msg);
};
} // namespace logging_system