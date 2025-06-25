#ifndef SYSTEM_PLUGIN_EXPLOSIVEMINE_HH_
#define SYSTEM_PLUGIN_EXPLOSIVEMINE_HH_

#include <algorithm>
#include <functional>
#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Color.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/transport/Node.hh>
#include <optional>
#include <ostream>
#include <sdf/Element.hh>
#include <sdf/Material.hh>
#include <string>
#include <vector>

#include "GaussMarkovProcess.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/ContactSensorData.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/World.hh"
#include "lotusim_common/logger.hpp"

namespace lotusim::gazebo {

class UnderwaterCurrentPlugin : public gz::sim::System,
                                public gz::sim::ISystemConfigure,
                                public gz::sim::ISystemUpdate {
public:
    UnderwaterCurrentPlugin();

    // Documentation inherited
public:
    void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_eventMgr) override;

    /// Documentation inherited

public:
    void Update(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm);

protected:
    void PublishCurrentVelocity();

protected:
    std::string worldName;

protected:
    gz::sim::EntityComponentManager *ecm_;

protected:
    gz::sim::Entity world_;

    /// \brief Copy of the sdf configuration used for this plugin
protected:
    sdf::ElementPtr sdfConfig;

    /// \brief Pointer to a node for communication
protected:
    gz::transport::Node node;

    /// \brief Publisher which publishes a message
protected:
    gz::transport::Node::Publisher publisher;

    /// \brief Current velocity topic
protected:
    std::string currentVelocityTopic;

    /// \brief Namespace for transport topics.
protected:
    std::string ns;

protected:
    lotusim::gazebo::GaussMarkovProcess currentVelModel;

    /// \brief Gauss-Markov process instance for horizontal angle model
protected:
    lotusim::gazebo::GaussMarkovProcess currentHorzAngleModel;

    /// \brief Gauss-Markov process instance for vertical angle model
protected:
    lotusim::gazebo::GaussMarkovProcess currentVertAngleModel;

    /// \brief Current simulation time.
protected:
    std::chrono::steady_clock::duration simTime{0};

    /// \brief Time when light was last updated
protected:
    std::chrono::steady_clock::duration lastUpdate{0};

protected:
    gz::math::Vector3<double> currentVelocity;
};

}  // namespace lotusim::gazebo
#endif  // EXPLOSIVEMINE_LIBRARY_H
