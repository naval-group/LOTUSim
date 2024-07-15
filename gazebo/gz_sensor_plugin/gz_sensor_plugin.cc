#include "gz_sensor_plugin.hh"

using namespace sensor_plugin;

gz_sensor_plugin::gz_sensor_plugin()
{
    m_gz_node = std::make_shared<gz::transport::Node>();
}

gz_sensor_plugin::~gz_sensor_plugin() {}

void gz_sensor_plugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr)
{
    ecm_ = &_ecm;
    this->entity = _entity;

    auto sdf = _sdf->Clone();

    gz::sim::Entity world =
        _ecm.EntityByComponents(gz::sim::components::World());
    this->worldName = _ecm.Component<gz::sim::components::Name>(world)->Data();

    if (sdf->HasElement("colors")) {
        sdf::ElementPtr colors = sdf->GetElement("colors");
    }

    sensor_demo_sched = node.Advertise<gz::msgs::Int32>("/sensor_demo_sched");
    agent_demo_sched = node.Advertise<gz::msgs::Int32>("/agent_demo_sched");
    plugin_demo_sched = node.Advertise<gz::msgs::Int32>("/plugin_demo_sched");

    node.Subscribe(
        "/effector_demo_sched",
        std::function<void(const gz::msgs::Pose &)>(std::bind(
            &gz_sensor_plugin::EffectorCallback, this, std::placeholders::_1)));
}

void gz_sensor_plugin::EffectorCallback(const gz::msgs::Pose &msg)
{
    gz::msgs::Int32 dbg_msg;
    dbg_msg.set_data(msg.id());
    agent_demo_sched.Publish(dbg_msg);

    gz::msgs::Boolean rep;
    bool result;
    unsigned int timeout = 5000;

    std::string service = "/world/" + this->worldName + "/set_pose";

    bool executed = m_gz_node->Request(service, msg, timeout, rep, result);

    dbg_msg.set_data(0);
    agent_demo_sched.Publish(dbg_msg);
}

void gz_sensor_plugin::PreUpdate(
    const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{
}

void gz_sensor_plugin::Update(
    const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{
}

void gz_sensor_plugin::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
    if (_info.simTime.count() - previousSimTime > 1e9) {
        gz::msgs::Int32 msg;
        msg.set_data(0);
        plugin_demo_sched.Publish(msg);

        previousSimTime = _info.simTime.count();
        msg.set_data(10);
        sensor_demo_sched.Publish(msg);
        plugin_demo_sched.Publish(msg);

        msg.set_data(0);
        plugin_demo_sched.Publish(msg);
    }
}

// void gz_sensor_plugin::Reset(
//     const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
// {
// }

GZ_ADD_PLUGIN(
    sensor_plugin::gz_sensor_plugin,
    gz::sim::System,
    sensor_plugin::gz_sensor_plugin::ISystemConfigure,
    sensor_plugin::gz_sensor_plugin::ISystemPreUpdate,
    sensor_plugin::gz_sensor_plugin::ISystemUpdate,
    sensor_plugin::gz_sensor_plugin::ISystemPostUpdate)
