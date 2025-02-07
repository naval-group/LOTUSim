#include "UnderwaterCurrentPlugin.hh"
#include "GaussMarkovProcess.hh"


namespace lotusim::gazebo {
using namespace std::placeholders;

UnderwaterCurrentPlugin::UnderwaterCurrentPlugin()
{
  // Doing nothing for now
}


void UnderwaterCurrentPlugin::Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr)
{
    ecm_ = &_ecm;

    world_ = _ecm.EntityByComponents(gz::sim::components::World());
    this->worldName = _ecm.Component<gz::sim::components::Name>(world_)->Data();
  gzdbg << "World name : " << this->worldName << std::endl;
  
  if (!_sdf->HasElement("constant_current")) {
    gzerr << "Missing required parameter <current_vel>." << std::endl;
    return;
  }
  
  auto sdf = _sdf->Clone();
 

    sdf::ElementPtr currentVelocityParams = sdf->GetElement("constant_current");


  if (currentVelocityParams->HasElement("topic")) {
    this->currentVelocityTopic = currentVelocityParams->Get<std::string>("topic");
    
  }
  else {
  
    this->currentVelocityTopic = "ocean_current";
    
  }
  

  if (currentVelocityParams->HasElement("velocity")) {
    sdf::ElementPtr elem = currentVelocityParams->GetElement("velocity");
    if (elem->HasElement("mean"))
      this->currentVelModel.mean = elem->Get<double>("mean");
    if (elem->HasElement("min"))
      this->currentVelModel.min = elem->Get<double>("min");
    if (elem->HasElement("max"))
      this->currentVelModel.max = elem->Get<double>("max");
    if (elem->HasElement("mu"))
      this->currentVelModel.mu = elem->Get<double>("mu");
    if (elem->HasElement("noiseAmp"))
      this->currentVelModel.noiseAmp = elem->Get<double>("noiseAmp");
  }


   this->currentVelModel.var = this->currentVelModel.mean;
  gzmsg << "Current velocity [m/s] Gauss-Markov process model:" << std::endl;
  this->currentVelModel.Print();

  if (currentVelocityParams->HasElement("horizontal_angle"))
  {
    sdf::ElementPtr elem =
      currentVelocityParams->GetElement("horizontal_angle");

    if (elem->HasElement("mean"))
      this->currentHorzAngleModel.mean = elem->Get<double>("mean");
    if (elem->HasElement("min"))
      this->currentHorzAngleModel.min = elem->Get<double>("min");
    if (elem->HasElement("max"))
      this->currentHorzAngleModel.max = elem->Get<double>("max");
    if (elem->HasElement("mu"))
      this->currentHorzAngleModel.mu = elem->Get<double>("mu");
    if (elem->HasElement("noiseAmp"))
      this->currentHorzAngleModel.noiseAmp = elem->Get<double>("noiseAmp");
      
   }
   
   this->currentHorzAngleModel.var = this->currentHorzAngleModel.mean;
  gzmsg <<
    "Current velocity horizontal angle [rad] Gauss-Markov process model:"
    << std::endl;
  this->currentHorzAngleModel.Print();

  if (currentVelocityParams->HasElement("vertical_angle"))
  {
    sdf::ElementPtr elem = currentVelocityParams->GetElement("vertical_angle");

    if (elem->HasElement("mean"))
      this->currentVertAngleModel.mean = elem->Get<double>("mean");
    if (elem->HasElement("min"))
      this->currentVertAngleModel.min = elem->Get<double>("min");
    if (elem->HasElement("max"))
      this->currentVertAngleModel.max = elem->Get<double>("max");
    if (elem->HasElement("mu"))
      this->currentVertAngleModel.mu = elem->Get<double>("mu");
    if (elem->HasElement("noiseAmp"))
      this->currentVertAngleModel.noiseAmp = elem->Get<double>("noiseAmp");
  }

  this->currentVertAngleModel.var = this->currentVertAngleModel.mean;
  gzmsg <<
    "Current velocity horizontal angle [rad] Gauss-Markov process model:"
    << std::endl;
  this->currentHorzAngleModel.Print();
   

 

  this->currentVelModel.lastUpdate = 0;
  this->currentHorzAngleModel.lastUpdate = 0;
  this->currentVertAngleModel.lastUpdate = 0;

  this->publisher = this->node.Advertise<gz::msgs::Vector3d>("/ocean_current");
  
  gzmsg << "Current velocity topic name: " << this->ns + "/" + this->currentVelocityTopic << std::endl;
  
  
}


//////////////////////////////////////////////////
void UnderwaterCurrentPlugin::Update(const gz::sim::UpdateInfo &_info,
                                     gz::sim::EntityComponentManager &_ecm)
{
  this->simTime = _info.simTime;

  // Calculate the flow velocity and the direction using the Gauss-Markov model

  // Update current velocity
  double currentVelMag = this->currentVelModel.Update(simTime.count());

  // Update current horizontal direction around z axis of flow frame
  double horzAngle = this->currentHorzAngleModel.Update(simTime.count());

  // Update current vertical direction around z axis of flow frame
  double vertAngle = this->currentVertAngleModel.Update(simTime.count());

  // Generating the current velocity vector as in the NED frame
  this->currentVelocity = gz::math::Vector3<double>(
      currentVelMag * cos(horzAngle) * cos(vertAngle),
      currentVelMag * sin(horzAngle) * cos(vertAngle),
      currentVelMag * sin(vertAngle));

  // Update time stamp
  this->lastUpdate = simTime;
  this->PublishCurrentVelocity(); // Update logic here


   gzmsg << "Current velocity mag: " <<  currentVelMag  << std::endl
   << "Horz angle: " <<  horzAngle  << std::endl
   << "Vert angle: " <<  vertAngle  << std::endl;
}



//////////////////////////////////////////////////
void UnderwaterCurrentPlugin::PublishCurrentVelocity()
{
  gz::msgs::Vector3d currentVel;
  gz::msgs::Set(&currentVel, gz::math::Vector3<double>(this->currentVelocity.X(),
                                           this->currentVelocity.Y(),
                                           this->currentVelocity.Z()));;
     
                                           
  this->publisher.Publish(currentVel);
}


}  // namespace lotusim::gazebo

GZ_ADD_PLUGIN(
    lotusim::gazebo::UnderwaterCurrentPlugin,
    gz::sim::System,
    lotusim::gazebo::UnderwaterCurrentPlugin::ISystemConfigure,
    lotusim::gazebo::UnderwaterCurrentPlugin::ISystemUpdate)
