// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gazebo_step_control.hpp"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace step_control;

void GazeboStepControl::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr)
{
    step_control_status_ = false;
    steps_to_execute_ = 0;
    step_blocking_call_ = false;
    paused_ = true;

    world_ = _ecm.EntityByComponents(components::World());
    this->worldName = _ecm.Component<gz::sim::components::Name>(world_)->Data();

    rclcpp::init(0, nullptr);

    ros_node_ = rclcpp::Node::make_shared("gazebo_step_control_node");

    enablecontrol_service_ = ros_node_->create_service<std_srvs::srv::SetBool>(
        "step_control_enable",
        std::bind(
            &GazeboStepControl::OnUpdateControl,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    stepcontrol_service_ =
        ros_node_
            ->create_service<gazebo_step_control_interface::srv::StepControl>(
                "step",
                std::bind(
                    &GazeboStepControl::OnStepControl,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));
    
    // Offer transient local durability on the clock topic so that if publishing
    // is infrequent (e.g. the simulation is paused), late subscribers can
    // receive the previously published message(s).
    step_complete_pub_ = ros_node_->create_publisher<std_msgs::msg::Empty>(
        "/step_completed", rclcpp::QoS(rclcpp::KeepLast(10)).transient_local());

    // Step control flag in plugin sdf
    bool control_status_sdf = _sdf->Get<bool>("enable_control", false).first;

    UpdateControl(control_status_sdf);
}

void GazeboStepControl::PreUpdate(
    const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{
    if (step_control_status_) {
        UpdateEnd();
    }
    else if (paused_) {
        SetPaused(false);
    }

    rclcpp::spin_some(ros_node_);
}

void GazeboStepControl::Update(
    const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{
}

void GazeboStepControl::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
}

void GazeboStepControl::UpdateControl(bool enable_control)
{
    step_control_status_ = enable_control;
}

void GazeboStepControl::UpdateEnd(void)
{
    if (step_control_status_) {
        steps_to_execute_--;
        if (steps_to_execute_ <= 0 && !paused_) {
            SetPaused(true);

            // publish completion topic only for non blocking service call
            if (!step_blocking_call_) {
                step_complete_pub_->publish(std_msgs::msg::Empty());
            }
        }
    }
}

void GazeboStepControl::OnUpdateControl(
    std_srvs::srv::SetBool::Request::SharedPtr _req,
    std_srvs::srv::SetBool::Response::SharedPtr _res)
{
    UpdateControl(_req->data);
    _res->success = true;
}

void GazeboStepControl::OnStepControl(
    gazebo_step_control_interface::srv::StepControl::Request::SharedPtr _req,
    gazebo_step_control_interface::srv::StepControl::Response::SharedPtr _res)
{
    steps_to_execute_ = _req->steps;
    step_blocking_call_ = _req->block;
    // Unpause physics on each step service call
    if (steps_to_execute_ > 0) {
        SetPaused(false);
        if (step_blocking_call_) {
            while (steps_to_execute_ > 0)
                usleep(1000);
        }
    }
    _res->success = true;
}

void GazeboStepControl::SetPaused(bool pause)
{
    gz::msgs::WorldControl req;
    req.set_pause(pause);

    gz::msgs::Boolean rep;
    bool result;
    unsigned int timeout = 5000;

    std::string service = "/world/" + worldName + "/control";

    bool executed = node.Request(service, req, timeout, rep, result);

    if (executed) {
        if (result) {
            std::cout << "Response: [" << rep.data() << "]" << std::endl;
            paused_ = pause;
        }
        else
            std::cout << "Service call failed" << std::endl;
    }
    else
        std::cerr << "Service call timed out" << std::endl;
}

// void Reset(const gz::sim::UpdateInfo &_info,
//   gz::sim::EntityComponentManager &_ecm)
//   {
//     // gzmsg << "SampleSystem::Reset" << std::endl;
//   }

GZ_ADD_PLUGIN(
    step_control::GazeboStepControl,
    gz::sim::System,
    step_control::GazeboStepControl::ISystemConfigure,
    step_control::GazeboStepControl::ISystemPreUpdate,
    step_control::GazeboStepControl::ISystemUpdate,
    step_control::GazeboStepControl::ISystemPostUpdate)
