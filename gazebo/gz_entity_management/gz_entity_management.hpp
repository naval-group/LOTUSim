#pragma once

#include <gz/msgs.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>

#include "liquidai_msgs/msg/add_entity.hpp"
#include "liquidai_msgs/srv/add_entity_srv.hpp"
#include "liquidai_msgs/srv/add_entity_srv_array.hpp"
#include "liquidai_msgs/srv/get_id_by_name.hpp"
#include "liquidai_msgs/srv/remove_entity.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace entity_management {
class EntityManagement :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemUpdate,
    public gz::sim::ISystemPostUpdate
// public gz::sim::ISystemReset
{
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
    void OnAddEntity(
        const std::shared_ptr<liquidai_msgs::srv::AddEntitySrv::Request>
            request,
        std::shared_ptr<liquidai_msgs::srv::AddEntitySrv::Response> response);

public:
    void OnAddEntity_V(
        const std::shared_ptr<liquidai_msgs::srv::AddEntitySrvArray::Request>
            request,
        std::shared_ptr<liquidai_msgs::srv::AddEntitySrvArray::Response>
            response);

public:
    void OnRemoveEntity(
        const std::shared_ptr<liquidai_msgs::srv::RemoveEntity::Request>
            request,
        std::shared_ptr<liquidai_msgs::srv::RemoveEntity::Response> response);

public:
    void OnGetIdByName(
        const std::shared_ptr<liquidai_msgs::srv::GetIdByName::Request> request,
        std::shared_ptr<liquidai_msgs::srv::GetIdByName::Response> response);

public:
    void CreateBridge(
        std::string topic_name,
        std::string ros_type_name,
        std::string gz_type_name,
        std::string direction = "@");

private:
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Service<liquidai_msgs::srv::AddEntitySrv>::SharedPtr
        add_entity_service_;
    rclcpp::Service<liquidai_msgs::srv::AddEntitySrvArray>::SharedPtr
        add_entity_V_service_;
    rclcpp::Service<liquidai_msgs::srv::RemoveEntity>::SharedPtr
        remove_entity_service_;
    rclcpp::Service<liquidai_msgs::srv::GetIdByName>::SharedPtr
        get_id_by_name_service_;
    rclcpp::Node::SharedPtr step_control_client_node_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr step_control_client_;

    gz::sim::EntityComponentManager *ecm_;
    std::string worldName;
    gz::sim::Entity world_;
    gz::transport::Node node;
};
} // namespace entity_management