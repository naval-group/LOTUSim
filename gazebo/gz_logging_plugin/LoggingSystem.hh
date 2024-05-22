#ifndef SAMPLEPLUGIN_HH_
#define SAMPLEPLUGIN_HH_

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs.hh>
#include <gz/sim/Model.hh>

#include <string>
#include <vector>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

namespace logging_system
{
  class LoggingSystem:
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
      bool slowdown = false;
      float previousSimTime1, previousRealTime1, previousSimTime2, previousRealTime2, previousSimTime3, previousRealTime3;

    private: gz::sim::Entity entity;
    private: gz::transport::Node node;
    private: gz::transport::Node::Publisher nodePub, schedNodePub;

    public: LoggingSystem();
 
    public: ~LoggingSystem() override;

    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;
 
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;
 
    public: void Update(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;
 
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;
 
    // public: void Reset(const gz::sim::UpdateInfo &_info,
    //              gz::sim::EntityComponentManager &_ecm) override;

    public: void slowDownCallback(const gz::msgs::Boolean &_msg);
  };
}

#endif