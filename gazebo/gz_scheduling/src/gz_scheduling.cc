#include "gz_scheduling.hh"

using namespace scheduling_plugin;

gz_scheduling::gz_scheduling()
    : m_debug(false)
{
    m_gz_node = std::make_shared<gz::transport::Node>();
}

gz_scheduling::~gz_scheduling() {}

void gz_scheduling::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr)
{
    ecm_ = &_ecm;
    this->entity = _entity;
    auto sdfPtr = const_cast<sdf::Element *>(_sdf.get());

    gz::sim::Entity world =
        _ecm.EntityByComponents(gz::sim::components::World());
    this->worldName = _ecm.Component<gz::sim::components::Name>(world)->Data();

    if (sdfPtr->HasElement("debug")) {
        m_debug = sdfPtr->Get<bool>("debug");
        plugin_demo_sched =
            node.Advertise<gz::msgs::Int32>("/plugin_demo_sched");
        agent_demo_sched = node.Advertise<gz::msgs::Int32>("/agent_demo_sched");
    }

    /*
    Publishing sensor information from this plugin is optional since it is
    usually done by distributed Gazebo plugins
    */
    pose_sensor = node.Advertise<gz::msgs::Int32>("/pose_sensor");
    // ... other sensors to publish to if needed

    /*
    Every ROS2 agent should publish its effector commands to a topic
    registered here. This plugin will then dispatch the message in an
    unbiased way to the simulation.

    Most certainly, this topic will need to be bridged.
    */
    node.Subscribe(
        "/pose_effector",
        std::function<void(const gz::msgs::Pose &)>(std::bind(
            &gz_scheduling::PoseEffectorCallback,
            this,
            std::placeholders::_1)));
    // ... other effector
}

void gz_scheduling::PoseEffectorCallback(const gz::msgs::Pose &msg)
{
    if (m_debug) {
        gz::msgs::Int32 dbg_msg;
        dbg_msg.set_data(0);
        agent_demo_sched.Publish(dbg_msg);
        dbg_msg.set_data(msg.id());
        agent_demo_sched.Publish(dbg_msg);
    }

    // Creates an effector and adds it to the effectors list for processing
    auto effector = std::make_shared<PoseEffector>(m_gz_node, msg, worldName);
    effectors.push_back(effector);

    if (m_debug) {
        gz::msgs::Int32 dbg_msg;
        dbg_msg.set_data(msg.id());
        agent_demo_sched.Publish(dbg_msg);
        dbg_msg.set_data(0);
        agent_demo_sched.Publish(dbg_msg);
    }
}

void gz_scheduling::PreUpdate(
    const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{
    if (!effectors.empty()) {
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(effectors.begin(), effectors.end(), g);

        if (m_debug) {
            gz::msgs::Int32 msg;
            msg.set_data(0);
            plugin_demo_sched.Publish(msg);
            msg.set_data(50);
            plugin_demo_sched.Publish(msg);
            msg.set_data(0);
            plugin_demo_sched.Publish(msg);
        }

        while (!effectors.empty()) {
            effectors.back()->apply_effector();
            effectors.pop_back();
        }
    }
}

void gz_scheduling::Update(
    const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{
}

void gz_scheduling::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
    // Publishes sensor information every 1 simulation second
    if (_info.simTime.count() - previousSimTime > 1e9) {
        gz::msgs::Int32 msg;

        if (m_debug) {
            msg.set_data(0);
            plugin_demo_sched.Publish(msg);
        }

        previousSimTime = _info.simTime.count();
        msg.set_data(100);
        pose_sensor.Publish(msg);

        if (m_debug) {
            plugin_demo_sched.Publish(msg);
            msg.set_data(0);
            plugin_demo_sched.Publish(msg);
        }
    }
}

// void gz_scheduling::Reset(
//     const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
// {
// }

GZ_ADD_PLUGIN(
    scheduling_plugin::gz_scheduling,
    gz::sim::System,
    scheduling_plugin::gz_scheduling::ISystemConfigure,
    scheduling_plugin::gz_scheduling::ISystemPreUpdate,
    scheduling_plugin::gz_scheduling::ISystemUpdate,
    scheduling_plugin::gz_scheduling::ISystemPostUpdate)
