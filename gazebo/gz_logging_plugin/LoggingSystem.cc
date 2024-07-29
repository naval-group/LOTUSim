#include "LoggingSystem.hh"

using namespace logging_system;

LoggingSystem::LoggingSystem() : debug(false)
{
    m_gz_node = std::make_shared<gz::transport::Node>();
}

LoggingSystem::~LoggingSystem() {}

void LoggingSystem::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr)
{
    ecm_ = &_ecm;
    gzmsg << "[" << "Configuring..." << "] SampleSystem::Configure the Entity ["
          << _entity << "]" << std::endl;
    this->entity = _entity;

    auto sdf = _sdf->Clone();

    gz::sim::Entity world =
        _ecm.EntityByComponents(gz::sim::components::World());
    this->worldName = _ecm.Component<gz::sim::components::Name>(world)->Data();

    gz::sim::Model model_ = gz::sim::Model(entity);
    gzmsg << "This entity is a model of name " << model_.Name(_ecm);

    std::string topic = model_.Name(_ecm) + "/sched";
    gz::transport::AdvertiseMessageOptions options;
    nodePub = node.Advertise<gz::msgs::Int32>(topic, options);

    gz::transport::AdvertiseMessageOptions options2;
    schedNodePub = node.Advertise<gz::msgs::Int32>("/gz_sched", options2);

    if (!nodePub) {
        gzerr << "Error advertising topic [" << topic << "]" << std::endl;
        return;
    }
}

void LoggingSystem::PreUpdate(
    const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{
    gz::msgs::Int32 dbg_msg;
    dbg_msg.set_data(0);
    schedNodePub.Publish(dbg_msg);
    if (!_info.paused) {
        gz::msgs::Int32 msg;
        msg.set_data(entity);
        nodePub.Publish(msg);
        schedNodePub.Publish(msg);

        auto seconds = std::chrono::duration_cast<std::chrono::milliseconds>(
            _info.simTime);
        float simDelta = _info.simTime.count() - previousSimTime1;
        float realDelta = _info.realTime.count() - previousRealTime1;
        previousSimTime1 = _info.simTime.count();
        previousRealTime1 = _info.realTime.count();
        if (this->debug) {
            gzmsg << "[simTime:" << _info.simTime.count() << "]"
                  << " [realTime:" << _info.realTime.count()
                  << "] [simDelta:" << simDelta << "] [realDelta:" << realDelta
                  << "] [Id:" << entity << "] SampleSystem::PreUpdate"
                  << std::endl;
            sleep(1);
        }

        nodePub.Publish(msg);
        schedNodePub.Publish(msg);
    }
}

void LoggingSystem::Update(
    const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{
    if (!_info.paused) {
        gz::msgs::Int32 msg;
        msg.set_data(entity);
        nodePub.Publish(msg);
        schedNodePub.Publish(msg);

        float simDelta = _info.simTime.count() - previousSimTime2;
        float realDelta = _info.realTime.count() - previousRealTime2;
        previousSimTime2 = _info.simTime.count();
        previousRealTime2 = _info.realTime.count();
        auto seconds = std::chrono::duration_cast<std::chrono::milliseconds>(
            _info.simTime);
        if (this->debug) {
            gzmsg << "[simTime:" << _info.simTime.count() << "]"
                  << " [realTime:" << _info.realTime.count()
                  << "] [simDelta:" << simDelta << "] [realDelta:" << realDelta
                  << "] [Id:" << entity << "] SampleSystem::Update"
                  << std::endl;
            sleep(0.5);
        }

        nodePub.Publish(msg);
        schedNodePub.Publish(msg);
    }
}

void LoggingSystem::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
    if (!_info.paused) {
        gz::msgs::Int32 msg;
        msg.set_data(entity);
        nodePub.Publish(msg);
        schedNodePub.Publish(msg);

        // gzmsg << "["<< _info.simTime.count() <<"] [Entity Id=" << entity <<
        // "] SampleSystem::PostUpdate" << std::endl; sleep(3);

        nodePub.Publish(msg);
        schedNodePub.Publish(msg);
    }
}

// void LoggingSystem::Reset(
//     const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
// {
// }

GZ_ADD_PLUGIN(
    logging_system::LoggingSystem,
    gz::sim::System,
    logging_system::LoggingSystem::ISystemConfigure,
    logging_system::LoggingSystem::ISystemPreUpdate,
    logging_system::LoggingSystem::ISystemUpdate,
    logging_system::LoggingSystem::ISystemPostUpdate)
