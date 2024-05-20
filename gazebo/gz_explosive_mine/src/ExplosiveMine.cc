#include "ExplosiveMine.hh"

#include <algorithm>
#include <functional>
#include <gz/common/Console.hh>
#include <gz/math/Color.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Visual.hh>
#include <optional>
#include <ostream>
#include <sdf/Material.hh>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/ContactSensorData.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/World.hh"
#include <gz/sim/components/ParentEntity.hh>

using namespace gz;
using namespace sim;
using namespace systems;
using namespace mine;

class mine::ExplosiveMinePrivate {
  // Initialize the plugin
public:
  void Load(const gz::sim::EntityComponentManager &_ecm,
            const sdf::ElementPtr &_sdf);

  /// \brief Actual function that enables the plugin.
  /// \param[in] _value True to enable plugin.
public:
  void Enable(const bool _value);

  /// \brief Process contact sensor data and determine if a touch event occurs
  /// \param[in] _info Simulation update info
  /// \param[in] _ecm Immutable reference to the EntityComponentManager
public:
  void Update(const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm);

  /// \brief Add target entities. Called when new collisions are found
  /// \param[in] _ecm Immutable reference to the EntityComponentManager
  /// \param[in] _entities List of potential entities to add to targetEntities.
public:
  void AddTargetEntities(const gz::sim::EntityComponentManager &_ecm,
                         const std::vector<gz::sim::Entity> &_entities);

public:
  void EditEntityVisualColor(const gz::sim::Entity &_entity);
  /// \brief Model interface
public:
  gz::sim::Model model{gz::sim::kNullEntity};

public:
  /// \brief Transport node to keep services alive
  gz::transport::Node node;

public:
  std::string worldName;

public:
  gz::math::Color contactColor{1.0, 0.0, 0.0, 1.0};

public:
  gz::sim::Entity visualEntity;

  /// \brief Collision entities that have been designated as contact sensors.
  /// These will be checked against the targetEntities to establish whether this
  /// model is touching the targets
public:
  std::vector<gz::sim::Entity> collisionEntities;

public:
  std::vector<std::string> targetList;

  /// \brief Target collisions which this model should be touching.
public:
  std::vector<gz::sim::Entity> targetEntities;

public:
  std::string touchedEntityName;

  /// \brief std::chrono::duration type used throught this plugin
public:
  using DurationType = std::chrono::duration<double>;

  /// \brief Target time to continuously touch.
public:
  DurationType targetTime{0};

  /// \brief Time when started touching.
public:
  DurationType touchStart{0};

  /// \brief Namespace for transport topics.
public:
  std::string ns;

  /// \brief Publisher which publishes a message after touched for enough time
public:
  std::optional<gz::transport::Node::Publisher> touchedPub;

  /// \brief Copy of the sdf configuration used for this plugin
public:
  sdf::ElementPtr sdfConfig;

  /// \brief Initialization flag
public:
  bool initialized{false};

  /// \brief Set during Load to true if the configuration for the plugin is
  /// valid and the pre and post update can run
public:
  bool validConfig{false};

  /// \brief Whether the plugin is enabled.
public:
  bool enabled{false};

  /// \brief Mutex for variables mutated by the service callback.
  /// The variables are: touchPub, touchStart, enabled
public:
  std::mutex serviceMutex;
};
//////////////////////////////////////////////////
void ExplosiveMinePrivate::Load(const gz::sim::EntityComponentManager &_ecm,
                                const sdf::ElementPtr &_sdf) {
  Entity world = _ecm.EntityByComponents(components::World());
  this->worldName = _ecm.Component<gz::sim::components::Name>(world)->Data();
  gzdbg << "World name : " << this->worldName << std::endl;
  // Get target substring

  if (!_sdf->HasElement("targets")) {
    gzerr << "Missing required parameter <targets>." << std::endl;
    return;
  }
  if (!_sdf->GetElement("targets")->HasElement("item")) {
    gzerr << "Missing required parameter <item>." << std::endl;
    return;
  }
  if (_sdf->HasElement("colors")) {
    sdf::ElementPtr colors = _sdf->GetElement("colors");
    if (colors->HasElement("r") && colors->HasElement("g") &&
        colors->HasElement("b") && colors->HasElement("a")) {
      this->contactColor =
          gz::math::Color(colors->GetElement("r")->Get<double>(),
                          colors->GetElement("g")->Get<double>(),
                          colors->GetElement("b")->Get<double>(),
                          colors->GetElement("a")->Get<double>());

      gzdbg << "Color set to (" << this->contactColor.R() << ","
            << this->contactColor.G() << "," << this->contactColor.B() << ","
            << this->contactColor.A() << ")" << std::endl;
    }
  }
  sdf::ElementPtr targetsItemElement =
      _sdf->GetElement("targets")->GetElement("item");
  while (targetsItemElement) {
    this->targetList.push_back(targetsItemElement->Get<std::string>());
    targetsItemElement = targetsItemElement->GetNextElement("item");
  }

  std::vector<gz::sim::Entity> potentialEntities;
  _ecm.Each<gz::sim::components::Collision>(
      [&](const gz::sim::Entity &_entity,
          const gz::sim::components::Collision *) -> bool {
        potentialEntities.push_back(_entity);
        return true;
      });

  this->AddTargetEntities(_ecm, potentialEntities);

  // Create a list of collision entities that have been marked as contact
  // sensors in this model. These are collisions that have a ContactSensorData
  // component
  auto allLinks = _ecm.ChildrenByComponents(this->model.Entity(),
                                            gz::sim::components::Link());

  for (const gz::sim::Entity linkEntity : allLinks) {
    auto linkCollisions =
        _ecm.ChildrenByComponents(linkEntity, gz::sim::components::Collision());
    for (const gz::sim::Entity colEntity : linkCollisions) {
      if (_ecm.EntityHasComponentType(
              colEntity, gz::sim::components::ContactSensorData::typeId)) {
        this->collisionEntities.push_back(colEntity);
      }
    }
  }

  // Namespace
  if (!_sdf->HasElement("namespace")) {
    gzerr << "Missing required parameter <namespace>" << std::endl;
    return;
  }
  this->ns = gz::transport::TopicUtils::AsValidTopic(
      _sdf->Get<std::string>("namespace"));
  if (this->ns.empty()) {
    gzerr << "<namespace> [" << _sdf->Get<std::string>("namespace")
          << "] is invalid." << std::endl;
    return;
  }

  // Target time
  if (!_sdf->HasElement("time")) {
    gzerr << "Missing required parameter <time>" << std::endl;
    return;
  }

  this->targetTime = DurationType(_sdf->Get<double>("time"));

  // Start/stop "service"
  std::string enableService{"/" + this->ns + "/enable"};
  std::function<void(const gz::msgs::Boolean &)> enableCb =
      [this](const gz::msgs::Boolean &_req) { this->Enable(_req.data()); };
  this->node.Advertise(enableService, enableCb);

  this->validConfig = true;

  // Start enabled or not
  if (_sdf->Get<bool>("enabled", false).first) {
    this->Enable(true);
  }
}

//////////////////////////////////////////////////
void ExplosiveMinePrivate::Enable(const bool _value) {
  std::lock_guard<std::mutex> lock(this->serviceMutex);

  if (_value) {
    this->touchedPub.reset();
    this->touchedPub =
        this->node.Advertise<gz::msgs::StringMsg>("/" + this->ns + "/touched");

    this->touchStart = DurationType::zero();
    this->enabled = true;

    gzdbg << "Started touch plugin [" << this->ns << "]" << std::endl;
  } else {
    this->touchedPub.reset();
    this->enabled = false;

    gzdbg << "Stopped touch plugin [" << this->ns << "]" << std::endl;
  }
}

//////////////////////////////////////////////////
ExplosiveMine::ExplosiveMine()
    : System(), dataPtr(std::make_unique<ExplosiveMinePrivate>()) {}

//////////////////////////////////////////////////
void ExplosiveMinePrivate::Update(const gz::sim::UpdateInfo &_info,
                                  gz::sim::EntityComponentManager &_ecm) {
  GZ_PROFILE("TouchPluginPrivate::Update");
  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero()) {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  {
    std::lock_guard<std::mutex> lock(this->serviceMutex);
    if (!this->enabled)
      return;
  }

  if (_info.paused)
    return;

  bool touching{false};
  // Iterate through all the target entities and check if there is a contact
  // between the target entity and this model
  for (const gz::sim::Entity colEntity : this->collisionEntities) {
    auto *contacts =
        _ecm.Component<gz::sim::components::ContactSensorData>(colEntity);
    if (contacts) {
      // Check if the contacts include one of the target entities.
      for (const auto &contact : contacts->Data().contact()) {
        bool col1Target = std::binary_search(this->targetEntities.begin(),
                                             this->targetEntities.end(),
                                             contact.collision1().id());
        bool col2Target = std::binary_search(this->targetEntities.begin(),
                                             this->targetEntities.end(),
                                             contact.collision2().id());
        if (col1Target || col2Target) {
          touching = true;
          // Get the link component of the touched entity
          const gz::sim::components::ParentEntity *parent =
              _ecm.Component<gz::sim::components::ParentEntity>(
                  contact.collision2().id());
          visualEntity = _ecm.EntityByComponents(
              components::Visual(), components::ParentEntity(parent->Data()));
          if (parent) {
            while (parent) {
              std::string parentName =
                  _ecm.Component<gz::sim::components::Name>(parent->Data())
                      ->Data();
              if (parentName == "default") {
                break;
              } else {
                touchedEntityName = parentName;
                // keep going up the tree
                parent = _ecm.Component<gz::sim::components::ParentEntity>(
                    parent->Data());
              }
            }
          } else {
            gzerr << "ParentEntity not found" << std::endl;
          }
        }
      }
    }
  }

  if (!touching) {
    std::lock_guard<std::mutex> lock(this->serviceMutex);
    if (this->touchStart != DurationType::zero()) {
      gzdbg << "Model [" << this->model.Name(_ecm)
            << "] not touching anything at [" << _info.simTime.count() << "]"
            << std::endl;
    }
    this->touchStart = DurationType::zero();
    return;
  }

  // Start touch timer
  {
    std::lock_guard<std::mutex> lock(this->serviceMutex);
    if (this->touchStart == DurationType::zero()) {
      this->touchStart =
          std::chrono::duration_cast<DurationType>(_info.simTime);

      gzdbg << "Model [" << this->model.Name(_ecm) << "] started touching ["
            << this->touchedEntityName << "] at " << this->touchStart.count()
            << " s" << std::endl;
    }
  }

  // Check if it has been touched for long enough
  auto completed = (std::chrono::duration_cast<DurationType>(_info.simTime) -
                    this->touchStart) > this->targetTime;

  // This is a single-use plugin. After touched, publish a message
  // and stop updating
  if (completed) {
    gzdbg << "Model [" << this->model.Name(_ecm) << "] touched ["
          << this->touchedEntityName << "] exclusively for "
          << this->targetTime.count() << " s" << std::endl;
    {
      std::lock_guard<std::mutex> lock(this->serviceMutex);
      if (this->touchedPub.has_value()) {
        gz::msgs::StringMsg msg;
        msg.set_data("{\"touchedEntity\": \"" + this->touchedEntityName +
                     "\", \"mineName\": \"" + this->model.Name(_ecm) + "\"}");
        this->touchedPub->Publish(msg);
      }
    }
    // Disable
    this->Enable(false);
  }
}

void ExplosiveMinePrivate::EditEntityVisualColor(
    const gz::sim::Entity &_entity) {
  std::function<void(const msgs::Boolean &, const bool)> cb =
      [](const msgs::Boolean & /*_rep*/, const bool _result) {
        if (!_result)
          gzerr << "Error setting material color configuration" << " on visual"
                << std::endl;
      };
  msgs::Visual req;
  req.set_id(_entity);

  msgs::Set(req.mutable_material()->mutable_ambient(), this->contactColor);
  msgs::Set(req.mutable_material()->mutable_diffuse(), this->contactColor);
  msgs::Set(req.mutable_material()->mutable_specular(), this->contactColor);
  msgs::Set(req.mutable_material()->mutable_emissive(), this->contactColor);

  auto test = std::string("/world/" + this->worldName + "/visual_config");
  auto materialCmdService = transport::TopicUtils::AsValidTopic(test);
  if (materialCmdService.empty()) {
    gzerr << "Invalid material command service topic provided" << std::endl;
    return;
  }
  this->node.Request(materialCmdService, req, cb);
}

//////////////////////////////////////////////////
void ExplosiveMinePrivate::AddTargetEntities(
    const gz::sim::EntityComponentManager &_ecm,
    const std::vector<gz::sim::Entity> &_entities) {
  if (_entities.empty())
    return;

  for (gz::sim::Entity entity : _entities) {
    // The target name can be a substring of the desired collision name so we
    // have to iterate through all collisions and check if their scoped name has
    // this substring
    std::string name = scopedName(entity, _ecm);
    for (const std::string &target : this->targetList) {
      if (name.find(target) != std::string::npos) {
        this->targetEntities.push_back(entity);
      }
    }
  }

  // Sort so that we can do binary search later on.
  std::sort(this->targetEntities.begin(), this->targetEntities.end());
}

//////////////////////////////////////////////////
void ExplosiveMine::Configure(const gz::sim::Entity &_entity,
                              const std::shared_ptr<const sdf::Element> &_sdf,
                              gz::sim::EntityComponentManager &_ecm,
                              gz::sim::EventManager &) {
  this->dataPtr->model = gz::sim::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm)) {
    gzerr << "Touch plugin should be attached to a model entity. "
          << "Failed to initialize." << std::endl;
    return;
  }
  this->dataPtr->sdfConfig = _sdf->Clone();
}

//////////////////////////////////////////////////
void ExplosiveMine::PreUpdate(const gz::sim::UpdateInfo &,
                              gz::sim::EntityComponentManager &_ecm) {
  GZ_PROFILE("TouchPlugin::PreUpdate");
  if ((!this->dataPtr->initialized) && this->dataPtr->sdfConfig) {
    // We call Load here instead of Configure because we can't be guaranteed
    // that all entities have been created when Configure is called
    this->dataPtr->Load(_ecm, this->dataPtr->sdfConfig);
    this->dataPtr->initialized = true;
  }

  // If Load() was successful, validConfig is set to true
  if (this->dataPtr->validConfig) {
    // Update target entities when new collisions are added
    std::vector<gz::sim::Entity> potentialEntities;
    _ecm.EachNew<gz::sim::components::Collision>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::Collision *) -> bool {
          potentialEntities.push_back(_entity);
          return true;
        });
    this->dataPtr->AddTargetEntities(_ecm, potentialEntities);
    if (this->dataPtr->touchedEntityName != "" && this->dataPtr->enabled) {
      this->dataPtr->EditEntityVisualColor(this->dataPtr->visualEntity);
      this->dataPtr->EditEntityVisualColor(_ecm.EntityByComponents(
          components::ParentEntity(this->dataPtr->model.Links(_ecm).data()[0]), components::Visual()));
    }
  }
}

//////////////////////////////////////////////////
void ExplosiveMine::PostUpdate(const gz::sim::UpdateInfo &_info,
                               const gz::sim::EntityComponentManager &_ecm) {
  GZ_PROFILE("TouchPlugin::PostUpdate");
  if (this->dataPtr->validConfig) {
    this->dataPtr->Update(_info,
                          const_cast<gz::sim::EntityComponentManager &>(_ecm));
  }
}

GZ_ADD_PLUGIN(mine::ExplosiveMine, gz::sim::System,
              mine::ExplosiveMine::ISystemConfigure,
              mine::ExplosiveMine::ISystemPreUpdate,
              mine::ExplosiveMine::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(ExplosiveMine, "mine::ExplosiveMine")
