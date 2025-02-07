#ifndef GZ_SIM_WINDADJUSTERPLUGIN_HH_
#define GZ_SIM_WINDADJUSTERPLUGIN_HH_

#include <gz/sim/gui/GuiSystem.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/wind.pb.h>  // Include for wind message type

#include <gz/sim/EntityComponentManager.hh>

class WindAdjusterPlugin : public gz::sim::GuiSystem
{
  Q_OBJECT
  Q_PROPERTY(double xVelocity READ XVelocity WRITE SetXVelocity NOTIFY XVelocityChanged)
  Q_PROPERTY(double yVelocity READ YVelocity WRITE SetYVelocity NOTIFY YVelocityChanged)
  Q_PROPERTY(double zVelocity READ ZVelocity WRITE SetZVelocity NOTIFY ZVelocityChanged)

public:
  WindAdjusterPlugin();
  ~WindAdjusterPlugin() override;

  void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;
  void Update(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override;

  double XVelocity() const;
  double YVelocity() const;
  double ZVelocity() const;

  Q_INVOKABLE void setWindVelocity(double x, double y, double z);  // Function to set wind velocity

  void SetXVelocity(double x);
  void SetYVelocity(double y);
  void SetZVelocity(double z);

  private: std::unique_ptr<WindAdjusterPlugin> dataPtr;

signals:
  void XVelocityChanged();
  void YVelocityChanged();
  void ZVelocityChanged();

private:
  double xVelocity = 0.0;
  double yVelocity = 0.0;
  double zVelocity = 0.0;

  gz::transport::Node node;  // Node for communication
  gz::transport::Node::Publisher windPublisher;  // Publisher for wind topic
};

#endif
