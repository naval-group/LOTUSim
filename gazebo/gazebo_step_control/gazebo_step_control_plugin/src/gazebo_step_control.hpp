#ifndef STEPCONTROL_HH_
#define STEPCONTROL_HH_

#include <gz/transport/Node.hh>
#include "gz/sim/components/World.hh"
#include "gz/sim/components/Name.hh"
#include <gz/sim/Events.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs.hh>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_value.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/empty.hpp>
#include <gazebo_step_control_interface/srv/step_control.hpp>
#include <gazebo_step_control_interface/srv/copy_param.hpp>

// #include <iostream>
// #include <unistd.h>
// #include <sys/mman.h>
// #include <sys/stat.h>
// #include <fcntl.h>

// #include <string>
// #include <vector>
// #include <atomic>
// #include <chrono>
// #include <csignal>
// #include <iostream>
// #include <thread>

namespace step_control
{
  class GazeboStepControl:
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

  /// Step control after every world update done.
  void UpdateEnd(void);

  /// Callback from ROS service to enable/disable step control.
  /// \param[in] req SetBool request
  /// \param[out] res SetBool response
  void OnUpdateControl(
      std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res);

  /// To enable/disable step control.
  /// \param[in] control_status
  void UpdateControl(bool step_control_status);

  /// Callback from ROS service for step control.
  /// \param[in] req StepControl request
  /// \param[out] res StepControl response
  void OnStepControl(
      gazebo_step_control_interface::srv::StepControl::Request::SharedPtr req,
      gazebo_step_control_interface::srv::StepControl::Response::SharedPtr res);

  void SetPaused(bool pause);

  /// Gazebo-ROS node
  rclcpp::Node::SharedPtr ros_node_;

  /// Publish step complete event
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr step_complete_pub_;

  /// ROS service to handle requests to unpause physics.
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enablecontrol_service_;

  /// ROS service to handle requests to unpause physics.
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr namespace_reset_service_;

  /// ROS service to handle requests to unpause physics.
  rclcpp::Service<gazebo_step_control_interface::srv::StepControl>::SharedPtr stepcontrol_service_;

  /// Holds step control status
  bool step_control_status_;

  /// Number of steps to execute
  int64_t steps_to_execute_;

  /// If the service call to be blocked untill all steps executed
  bool step_blocking_call_;

  /// World name
  std::string worldName;

  /// World entity
  gz::sim::Entity world_;

  // Transport node
  gz::transport::Node node;

  bool paused_ = true;
  };
}

#endif