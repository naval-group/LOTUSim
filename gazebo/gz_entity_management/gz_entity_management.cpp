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

    ros_node_ = rclcpp::Node::make_shared("gz_entity_management_node");

    add_entity_service_ =
        ros_node_->create_service<liquidai_msgs::srv::AddEntitySrv>(
            "gz_add_entity",
            std::bind(
                &EntityManagement::OnAddEntity,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    add_entity_V_service_ =
        ros_node_->create_service<liquidai_msgs::srv::AddEntitySrvArray>(
            "gz_add_entity_V",
            std::bind(
                &EntityManagement::OnAddEntity_V,
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

    get_id_by_name_service_ =
        ros_node_->create_service<liquidai_msgs::srv::GetIdByName>(
            "gz_get_id_by_name",
            std::bind(
                &EntityManagement::OnGetIdByName,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    this->step_control_client_node_ =
        rclcpp::Node::make_shared("step_control_client_node");
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

/// @brief Callback for adding one entity. Prefer using the array version of
/// this function.
/// @param request
/// @param response
void EntityManagement::OnAddEntity(
    const std::shared_ptr<liquidai_msgs::srv::AddEntitySrv::Request> request,
    std::shared_ptr<liquidai_msgs::srv::AddEntitySrv::Response> response)
{
    gzmsg << "EntityManagement::OnAddEntity" << std::endl;

    liquidai_msgs::srv::AddEntitySrvArray::Request::SharedPtr req_array;
    req_array->data.push_back(request->data);
    liquidai_msgs::srv::AddEntitySrvArray::Response::SharedPtr res_array;
    OnAddEntity_V(req_array, res_array);
    response->result = res_array->result;
}

/// @brief Callback for adding one or multiple entities
/// @param request
/// @param response
void EntityManagement::OnAddEntity_V(
    const std::shared_ptr<liquidai_msgs::srv::AddEntitySrvArray::Request>
        request,
    std::shared_ptr<liquidai_msgs::srv::AddEntitySrvArray::Response> response)
{
    gzmsg << "EntityManagement::OnAddEntity_V" << std::endl;

    gz::msgs::EntityFactory_V reqs;

    for (liquidai_msgs::msg::AddEntity msg : request->data) {

        ::gz::msgs::EntityFactory *req = reqs.add_data();
        req->set_name(msg.name.c_str());
        // If a sdf file is provided, use it. Else, get the sdf provided by the
        // path.
        if (!msg.model_file.empty()) {
            req->set_sdf(msg.model_file.c_str());
        }
        else if (!msg.model_filepath.empty()) {
            std::string full_model_filepath =
                ament_index_cpp::get_package_share_directory("assets") + "/" +
                msg.model_filepath;
            req->set_sdf_filename(full_model_filepath.c_str());
        }

        gz::msgs::Pose *pose = new gz::msgs::Pose();

        auto location = new gz::msgs::Vector3d();
        location->set_x(msg.location.x);
        location->set_y(msg.location.y);
        location->set_z(msg.location.z);
        pose->set_allocated_position(location);

        gz::math::Quaterniond rotation(
            msg.rotation.x, msg.rotation.y, msg.rotation.z);
        auto orientation = new gz::msgs::Quaternion();
        orientation->set_x(rotation.X());
        orientation->set_y(rotation.Y());
        orientation->set_z(rotation.Z());
        orientation->set_w(rotation.W());
        pose->set_allocated_orientation(orientation);

        req->set_allocated_pose(pose);

        CreateBridge(
            "/model/" + msg.name + "/pose",
            "geometry_msgs/msg/PoseArray",
            "ignition.msgs.Pose_V");
    }

    gz::msgs::Boolean rep;
    bool result;
    unsigned int timeout = 10000;

    std::string service = "/world/" + worldName + "/create_multiple";

    bool executed = node.Request(service, reqs, timeout, rep, result);

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

void EntityManagement::OnGetIdByName(
    const std::shared_ptr<liquidai_msgs::srv::GetIdByName::Request> request,
    std::shared_ptr<liquidai_msgs::srv::GetIdByName::Response> response)
{
    int id = ecm_->EntityByComponents(components::Name(request->entity_name));
    response->id = id;
    gzmsg << "The id of " << request->entity_name << " is " << id;
}

// TODO: Improve this function
void EntityManagement::CreateBridge(
    std::string topic_name,
    std::string ros_type_name,
    std::string gz_type_name,
    std::string direction)
{
    std::ostringstream command_stream;
    command_stream << "ros2 run ros_gz_bridge parameter_bridge " << topic_name
                   << "@" << ros_type_name << direction << gz_type_name;
    std::string command = command_stream.str();
    std::thread cmdThread([this, command]() { system(command.data()); });
    cmdThread.detach(); // Detach the thread to run it independently
}

GZ_ADD_PLUGIN(
    entity_management::EntityManagement,
    gz::sim::System,
    entity_management::EntityManagement::ISystemConfigure,
    entity_management::EntityManagement::ISystemPreUpdate,
    entity_management::EntityManagement::ISystemUpdate,
    entity_management::EntityManagement::ISystemPostUpdate)