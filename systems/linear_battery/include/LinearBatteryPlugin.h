/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef GZ_SIM_SYSTEMS_LINEAR_BATTERY_PLUGIN_HH_
#define GZ_SIM_SYSTEMS_LINEAR_BATTERY_PLUGIN_HH_

#include <string>
#include <map>
#include <memory>

#include <gz/common/Battery.hh>

#include "gz/sim/System.hh"

#include <gz/msgs/battery_state.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/transport/TopicUtils.hh>


#include <algorithm>
#include <atomic>
#include <deque>
#include <functional>
#include <string>
#include <vector>

#include <gz/common/Battery.hh>
#include <gz/common/Profiler.hh>
#include <gz/common/Util.hh>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>
#include <sdf/Physics.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include "gz/sim/components/BatteryPowerLoad.hh"
#include "gz/sim/components/BatterySoC.hh"
#include "gz/sim/components/Joint.hh"
#include <gz/sim/components/AngularVelocity.hh>
#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Model.hh"
#include <gz/sim/components/Link.hh>

#include "logging_system/logger.hpp"


namespace lotusim::gazebo {

 // Forward declaration
class LinearBatteryPluginPrivate : public gz::sim::System {
 
  /// \brief Reset the plugin
  public: void Reset();

  /// \brief Get the current state of charge of the battery.
  /// \return State of charge of the battery in range [0.0, 1.0].
  public: double StateOfCharge() const;

  /// \brief Get the power of the battery with RPM
  /// \return Power
 // public: double Power ()

  /// \brief Callback executed to start recharging.
  /// \param[in] _req This value should be true.
  public: void OnEnableRecharge(const gz::msgs::Boolean &_req);

  /// \brief Callback executed to stop recharging.
  /// \param[in] _req This value should be true.
  public: void OnDisableRecharge(const gz::msgs::Boolean &_req);

  /// \brief Callback connected to additional topics that can start battery
  /// draining.
  /// \param[in] _data Message data.
  /// \param[in] _size Message data size.
  /// \param[in] _info Information about the message.
  public: void OnBatteryDrainingMsg(
    const char *_data, const size_t _size,
    const gz::transport::MessageInfo &_info);

  /// \brief Callback connected to additional topics that can stop battery
  /// draining.
  /// \param[in] _data Message data.
  /// \param[in] _size Message data size.
  /// \param[in] _info Information about the message.
  public: void OnBatteryStopDrainingMsg(
    const char *_data, const size_t _size,
    const gz::transport::MessageInfo &_info);

  /// \brief Name of model, only used for printing warning when battery drains.
  public: std::string modelName;

  /// \brief Name that identifies a battery.
  public: std::string batteryName;

  /// \brief Pointer to battery contained in link.
  public: gz::common::BatteryPtr battery;

  /// \brief Whether warning that battery has drained has been printed once.
  public: bool drainPrinted{false};

  /// \brief Battery consumer identifier.
  /// Current implementation limits one consumer (Model) per battery.
  public: int32_t consumerId;

  /// \brief Battery entity
  public: gz::sim::Entity batteryEntity{gz::sim::v8::kNullEntity};

  /// \brief Open-circuit voltage.
  /// E(t) = e0 + e1 * Q(t) / c
  public: double e0{0.0};
  public: double e1{0.0};

  /// \brief Initial battery charge in Ah.
  public: double q0{0.0};

  /// \brief Battery capacity in Ah.
  public: double c{0.0};

  /// \brief Battery inner resistance in Ohm.
  public: double r{0.0};

  /// \brief Current low-pass filter characteristic time in seconds [0, 1].
  public: double tau{1.0};

  /// \brief Raw battery current in A.
  public: double iraw{0.0};

  /// \brief Smoothed battery current in A.
  public: double ismooth{0.0};

  /// \brief Instantaneous battery charge in Ah.
  public: double q{0.0};

  /// \brief State of charge [0, 1].
  public: double soc{1.0};

  /// \brief Recharge status
  public: std::atomic_bool startCharging{false};

  /// \brief Hours taken to fully charge battery
  public: double tCharge{0.0};

  /// \TODO(caguero) Remove this flag in Gazebo Dome.
  /// \brief Flag to enable some battery fixes.
  public: bool fixIssue225{false};

  /// \TODO(caguero) Remove in Gazebo Dome.
  /// \brief Battery current for a historic time window
  public: std::deque<double> iList;

  /// \TODO(caguero) Remove in Gazebo Dome.
  /// \brief Time interval for a historic time window
  public: std::deque<double> dtList;

  /// \brief Simulation time handled during a single update.
  public: std::chrono::steady_clock::duration stepSize;

  /// \brief Flag on whether the battery should start draining
  public: bool startDraining = false;

  /// \brief The start time when battery starts draining in seconds
  public: int drainStartTime = -1;

  /// \brief Book keep the last time printed, so as to not pollute dbg messages
  /// in minutes
  public: int lastPrintTime = -1;

  /// \brief Model interface
  public: gz::sim::Model model{gz::sim::v8::kNullEntity};

  /// \brief Power
  public: double totalpower = 0.0;

  /// \brief Gazebo communication node
  //public: transport::Node node;

  //public: transport::Node::Publisher powerPub;

  /// \brief Battery state of charge message publisher
  public: gz::transport::Node::Publisher statePub;

  /// \brief Initial power load set trough config
  public: double initialPowerLoad = 0.0;

  gz::transport::Node node;
  gz::transport::Node::Publisher powerPub;
};


  class LinearBatteryPlugin
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate,
        public gz::sim::ISystemUpdate,
        public gz::sim::ISystemPostUpdate {
    /// \brief Constructor
    public: LinearBatteryPlugin();

    /// \brief Destructor
    public: ~LinearBatteryPlugin() override;

    // Documentation inherited
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) final;

    /// Documentation inherited
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    /// Documentation inherited
    public: void Update(const gz::sim::UpdateInfo &_info,
                        gz::sim::EntityComponentManager &_ecm) final;

    /// Documentation inherited
    public: void PostUpdate(
                const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Callback for Battery Update events.
    /// \param[in] _battery Pointer to the battery that is to be updated.
    /// \return The new voltage.
    public: double OnUpdateVoltage(const gz::common::Battery *_battery);

    /// \brief Private data pointer
    private: std::unique_ptr<LinearBatteryPluginPrivate> dataPtr;
  };
  
}  // namespace lotusim::gazebo

#endif
