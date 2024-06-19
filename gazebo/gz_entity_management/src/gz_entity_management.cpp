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

#include "gz_entity_management.hpp"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace entity_management;

// TODO: Add bridge

void EntityManagement::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr)
{
    ecm_ = &_ecm;

    world_ = _ecm.EntityByComponents(components::World());
    this->worldName = _ecm.Component<gz::sim::components::Name>(world_)->Data();

    // rclcpp::init(0, nullptr);

    ros_node_ = rclcpp::Node::make_shared("gz_entity_management_node");

    add_entity_service_ =
        ros_node_->create_service<liquidai_msgs::srv::AddEntity>(
            "gz_add_entity",
            std::bind(
                &EntityManagement::OnAddEntity,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    remove_entity_service_ =
        ros_node_->create_service<liquidai_msgs::srv::RemoveEntity>(
            "gz_remove_entity",
            std::bind(
                &EntityManagement::OnRemoveEntity,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    this->step_control_client_node_ =
        rclcpp::Node::make_shared("step_control_client_node2");

    this->step_control_client_ =
        step_control_client_node_->create_client<std_srvs::srv::SetBool>(
            "step_control_enable");

    // // Wait for the service to be activated
    // while (!step_control_client_->wait_for_service(std::chrono::seconds(1))) {
    //     // If ROS is shutdown before the service is activated, show this
    //     if (!rclcpp::ok()) {
    //         RCLCPP_ERROR(
    //             rclcpp::get_logger("rclcpp"),
    //             "Interrupted while waiting for the service. Exiting.");
    //         return;
    //     }

    //     RCLCPP_INFO(
    //         rclcpp::get_logger("rclcpp"),
    //         "Service not available, waiting again...");
    // }
}

void EntityManagement::PreUpdate(
    const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{

}

void EntityManagement::Update(
    const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{
    rclcpp::spin_some(ros_node_);
    rclcpp::spin_some(step_control_client_node_);
}

void EntityManagement::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
}

void EntityManagement::OnAddEntity(
    const std::shared_ptr<liquidai_msgs::srv::AddEntity::Request> request,
    std::shared_ptr<liquidai_msgs::srv::AddEntity::Response> response)
{
    gzmsg << "EntityManagement::OnAddEntity" << std::endl;

    // // Pause the simulation
    // auto pause_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    // pause_request->data = true;
    // auto pause_req_res =
    //     step_control_client_->async_send_request(pause_request);
    
    // if (rclcpp::spin_until_future_complete(
    //         step_control_client_node_, pause_req_res) !=
    //     rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(
    //         rclcpp::get_logger("rclcpp"),
    //         "Failed to call service 'step_control_enable'");
    //     response->result = false;
    //     return;
    // }

    std::string full_model_filepath =
        ament_index_cpp::get_package_share_directory("assets") + "/" +
        request->model_filepath;

    gz::msgs::EntityFactory req;
    req.set_name(request->name.c_str());
    req.set_sdf_filename(full_model_filepath.c_str());

    gz::msgs::Pose *pose = new gz::msgs::Pose();

    auto location = new gz::msgs::Vector3d();
    location->set_x(request->location.x);
    location->set_y(request->location.y);
    location->set_z(request->location.z);
    pose->set_allocated_position(location);

    gz::math::Quaterniond rotation(
        request->rotation.x, request->rotation.y, request->rotation.z);
    auto orientation = new gz::msgs::Quaternion();
    orientation->set_x(rotation.X());
    orientation->set_y(rotation.Y());
    orientation->set_z(rotation.Z());
    orientation->set_w(rotation.W());
    pose->set_allocated_orientation(orientation);

    req.set_allocated_pose(pose);

    gzmsg << "Request: [" << req.DebugString() << "]" << std::endl;

    gz::msgs::Boolean rep;
    bool result;
    unsigned int timeout = 5000;

    std::string service = "/world/" + worldName + "/create";

    bool executed = node.Request(service, req, timeout, rep, result);

    if (executed) {
        if (result) {
            std::cout << "Response: [" << rep.data() << "]" << std::endl;
            response->result = rep.data();
        }
        else {
            std::cout << "Service call failed" << std::endl;
            response->result = false;
        }
    }
    else {
        std::cerr << "Service call timed out" << std::endl;
        response->result = false;
    }

    // pause_request->data = false;
    // auto pause_req_res2 =
    //     step_control_client_->async_send_request(pause_request);

    // if (rclcpp::spin_until_future_complete(
    //         step_control_client_node_, pause_req_res2) !=
    //     rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_ERROR(
    //         rclcpp::get_logger("rclcpp"),
    //         "Failed to call service 'step_control_enable'");
    //     return;
    // }
}

void EntityManagement::OnRemoveEntity(
    const std::shared_ptr<liquidai_msgs::srv::RemoveEntity::Request> request,
    std::shared_ptr<liquidai_msgs::srv::RemoveEntity::Response> response)
{
    gzmsg << "EntityManagement::OnRemoveEntity" << std::endl;

    gz::msgs::Entity req;
    req.set_name(request->name.c_str());
    req.set_type(gz::msgs::Entity::MODEL);

    gzmsg << "Request: [" << req.DebugString() << "]" << std::endl;

    gz::msgs::Boolean rep;
    bool result;
    unsigned int timeout = 5000;

    std::string service = "/world/" + worldName + "/remove";

    bool executed = node.Request(service, req, timeout, rep, result);

    if (executed) {
        if (result) {
            std::cout << "Response: [" << rep.data() << "]" << std::endl;
            response->result = rep.data();
        }
        else {
            std::cout << "Service call failed" << std::endl;
            response->result = false;
        }
    }
    else {
        std::cerr << "Service call timed out" << std::endl;
        response->result = false;
    }
}

GZ_ADD_PLUGIN(
    entity_management::EntityManagement,
    gz::sim::System,
    entity_management::EntityManagement::ISystemConfigure,
    entity_management::EntityManagement::ISystemPreUpdate,
    entity_management::EntityManagement::ISystemUpdate,
    entity_management::EntityManagement::ISystemPostUpdate)