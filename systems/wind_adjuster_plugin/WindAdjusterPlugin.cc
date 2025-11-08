#include <gz/gui/Plugin.hh>
#include "WindAdjusterPlugin.hh"
#include <gz/plugin/Register.hh>
#include <QQmlContext>
#include <QQmlEngine>
#include <QQmlComponent>
#include <QtQml>
#include <gz/gui/Helpers.hh>

WindAdjusterPlugin::WindAdjusterPlugin()
{
  // Optional: register your object to use in QML
  qmlRegisterType<WindAdjusterPlugin>("WindAdjusterPlugin", 1, 0, "WindAdjusterPlugin");

  // Init wind publisher
  this->windPublisher = node.Advertise<gz::msgs::Wind>("/world/" + gz::gui::worldNames()[0].toStdString() + "/wind/");
}

WindAdjusterPlugin::~WindAdjusterPlugin() {}  // Define destructor explicitly

void WindAdjusterPlugin::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
  // Set the plugin instance as a context property in QML
  auto *engine = qmlEngine(this);
  if (engine) {
    engine->rootContext()->setContextProperty("plugin", this);
  }
  
  if (this->title.empty())
    this->title = "Wind Adjuster";
}

double WindAdjusterPlugin::XVelocity() const { return xVelocity; }
double WindAdjusterPlugin::YVelocity() const { return yVelocity; }
double WindAdjusterPlugin::ZVelocity() const { return zVelocity; }

void WindAdjusterPlugin::SetXVelocity(double x)
{
  if (this->xVelocity != x) {
    this->xVelocity = x;
    emit XVelocityChanged();
    setWindVelocity(x, this->yVelocity, this->zVelocity);
  }
}

void WindAdjusterPlugin::SetYVelocity(double y)
{
  if (this->yVelocity != y) {
    this->yVelocity = y;
    emit YVelocityChanged();
    setWindVelocity(this->xVelocity, y, this->zVelocity);
  }
}

void WindAdjusterPlugin::SetZVelocity(double z)
{
  if (this->zVelocity != z) {
    this->zVelocity = z;
    emit ZVelocityChanged();
    setWindVelocity(this->xVelocity, this->yVelocity, z);
  }
}

void WindAdjusterPlugin::setWindVelocity(double x, double y, double z)
{
  gz::msgs::Wind windMsg;
  windMsg.mutable_linear_velocity()->set_x(x);
  windMsg.mutable_linear_velocity()->set_y(y);
  windMsg.mutable_linear_velocity()->set_z(z);

  if (x == 0.0 && y == 0.0 && z == 0.0)
    windMsg.set_enable_wind(false);
  else
    windMsg.set_enable_wind(true);

  this->windPublisher.Publish(windMsg);
}

GZ_ADD_PLUGIN(WindAdjusterPlugin,
              gz::gui::Plugin)
