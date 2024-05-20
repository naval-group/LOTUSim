#include "LoggingSystem.hh"
#include <gz/plugin/Register.hh>
#include <iostream>
#include<unistd.h>

using namespace logging_system;
 
LoggingSystem::LoggingSystem()
{
}
 
LoggingSystem::~LoggingSystem()
{

}

void LoggingSystem::Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr)
{
  gzmsg << "["<< "Configuring..." <<"] SampleSystem::Configure the Entity [" << _entity << "]" <<std::endl;
  this->entity = _entity;

  gz::sim::Model model_ = gz::sim::Model(entity);
  gzmsg << "This entity is a model of name " << model_.Name(_ecm);

  std::string topic = model_.Name(_ecm) + "/sched";
  gz::transport::AdvertiseMessageOptions options;
  nodePub = node.Advertise<gz::msgs::Int32>(topic, options);

  gz::transport::AdvertiseMessageOptions options2;
  schedNodePub = node.Advertise<gz::msgs::Int32>("/gz_sched", options2);

  gz::transport::SubscribeOptions options3;
  node.Subscribe("/activate_slowdown", std::function<void(const gz::msgs::Boolean &)>(std::bind(&LoggingSystem::slowDownCallback, this, std::placeholders::_1)), options3);

  if (!nodePub)
  {
    gzerr << "Error advertising topic [" << topic << "]" << std::endl;
    return;
  }
}

void LoggingSystem::slowDownCallback(const gz::msgs::Boolean &_msg)
{
  gzmsg << "Callback called" << std::endl;
  if(_msg.data()){
    gzmsg << "Slowdown activated" << std::endl;
    this->slowdown = true;
  }
  else{
    gzmsg << "Slowdown deactivated" << std::endl;
    this->slowdown = false;
  }
}

void LoggingSystem::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if(!_info.paused){
    gz::msgs::Int32 msg;
    msg.set_data(entity);
    nodePub.Publish(msg);
    schedNodePub.Publish(msg);

    auto seconds = std::chrono::duration_cast<std::chrono::milliseconds>(_info.simTime);
    float simDelta = _info.simTime.count() - previousSimTime1;
    float realDelta = _info.realTime.count() - previousRealTime1;
    previousSimTime1 = _info.simTime.count();
    previousRealTime1 = _info.realTime.count();
    if(this->slowdown){
      gzmsg << "[simTime:"<< _info.simTime.count() <<"]" << " [realTime:"<< _info.realTime.count() << "] [simDelta:"<< simDelta <<"] [realDelta:"<< realDelta <<"] [Id:" << entity << "] SampleSystem::PreUpdate" << std::endl;
      sleep(1);
    }

    nodePub.Publish(msg);
    schedNodePub.Publish(msg);
  }
}

void LoggingSystem::Update(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if(!_info.paused){
    gz::msgs::Int32 msg;
    msg.set_data(entity);
    nodePub.Publish(msg);
    schedNodePub.Publish(msg);

    float simDelta = _info.simTime.count() - previousSimTime2;
    float realDelta = _info.realTime.count() - previousRealTime2;
    previousSimTime2 = _info.simTime.count();
    previousRealTime2 = _info.realTime.count();
    auto seconds = std::chrono::duration_cast<std::chrono::milliseconds>(_info.simTime);
    if(this->slowdown){
      gzmsg << "[simTime:"<< _info.simTime.count() <<"]" << " [realTime:"<< _info.realTime.count() << "] [simDelta:"<< simDelta <<"] [realDelta:"<< realDelta <<"] [Id:" << entity << "] SampleSystem::Update" << std::endl;
      sleep(2);
    }

    nodePub.Publish(msg);
    schedNodePub.Publish(msg);
  }
}
 
void LoggingSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  if(!_info.paused){
    gz::msgs::Int32 msg;
    msg.set_data(entity);
    nodePub.Publish(msg);
    schedNodePub.Publish(msg);

    // gzmsg << "["<< _info.simTime.count() <<"] [Entity Id=" << entity << "] SampleSystem::PostUpdate" << std::endl;
    // sleep(3);

    nodePub.Publish(msg);
    schedNodePub.Publish(msg);
  }
}

// void Reset(const gz::sim::UpdateInfo &_info,
//   gz::sim::EntityComponentManager &_ecm)
//   {
//     // gzmsg << "SampleSystem::Reset" << std::endl;
//   }

GZ_ADD_PLUGIN(
    logging_system::LoggingSystem,
    gz::sim::System,
    logging_system::LoggingSystem::ISystemConfigure,
    logging_system::LoggingSystem::ISystemPreUpdate,
    logging_system::LoggingSystem::ISystemUpdate,
    logging_system::LoggingSystem::ISystemPostUpdate)
