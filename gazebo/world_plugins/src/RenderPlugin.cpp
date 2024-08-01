#include "world_plugins/RenderPlugin.h"

namespace liquidai {
namespace gazebo {

RenderPlugin::RenderPlugin() {}

void RenderPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr)
{
    auto sdfPtr = const_cast<sdf::Element *>(_sdf.get());

    std::string connection_protocol;
    if (sdfPtr->HasElement("connection_protocol")) {
        connection_protocol = sdfPtr->Get<std::string>("connection_protocol");
        gzmsg << "RenderPlugin::Configure: Creating connection: "
              << connection_protocol << "\n";
    }
    else {
        gzerr << "RenderPlugin::Configure: Connection protocol not found\n";
        return;
    }

    m_render_interface = CreateRenderInterface(connection_protocol);
    m_render_interface->ConfigureInterface(_sdf);
}

void RenderPlugin::PreUpdate(
    const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &_ecm)
{
}

void RenderPlugin::Update(
    const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{
    // TODO: To handle the MAS
    _ecm.EachNew<
        gz::sim::components::ModelSdf,
        gz::sim::components::ParentEntity>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::ModelSdf *_model,
            const gz::sim::components::ParentEntity *_parent) -> bool {
            sdf::Model data = _model->Data();
            bool publish_render{false};
            sdf::ElementPtr sdfptr = data.Element();

            gzmsg << "[LOTUSim]: Detected creation of a new entity"
                  << std::endl;

            if (sdfptr->HasElement("publish_render")) {
                publish_render = sdfptr->Get<bool>("publish_render");
            }
            if (publish_render) {
                auto name_opt =
                    _ecm.Component<gz::sim::components::Name>(_entity);
                m_vessel_entity[name_opt->Data()] = _entity;

                gzmsg << "[LOTUSim]: Creation detected of vessel named "
                      << name_opt->Data() << std::endl;

                m_render_interface->CreateVessel(name_opt->Data(), sdfptr);
            }
            return true;
        });
}

void RenderPlugin::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
    if (!m_render_interface)
        return;

    _ecm.EachRemoved<
        gz::sim::components::ModelSdf,
        gz::sim::components::ParentEntity>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::ModelSdf *_model,
            const gz::sim::components::ParentEntity *_parent) -> bool {
            sdf::Model data = _model->Data();
            bool publish_render{false};
            sdf::ElementPtr sdfptr = data.Element();
            if (sdfptr->HasElement("publish_render")) {
                publish_render = sdfptr->Get<bool>("publish_render");
            }

            if (publish_render) {
                auto name_opt =
                    _ecm.Component<gz::sim::components::Name>(_entity);

                gzmsg << "[LOTUSim]: Deletion detected of vessel named "
                      << name_opt->Data() << std::endl;

                m_render_interface->DestroyVessel(name_opt->Data());
            }
            for (auto it = m_vessel_entity.begin(); it != m_vessel_entity.end();
                 ++it) {
                if (it->second == _entity) {
                    m_vessel_entity.erase(
                        it); // Erase the element using the iterator
                    break;   // Exit the loop after erasing the element
                }
            }
            return true;
        });

    // get a vector of vessel name, pose
    std::vector<std::pair<std::string, gz::math::Pose3d>> vessel_pose;
    for (auto &&entity : m_vessel_entity) {
        gz::math::Pose3d pose =
            _ecm.Component<gz::sim::components::Pose>(entity.second)->Data();

        vessel_pose.push_back({entity.first, pose});
    }
    m_render_interface->SendPosition(_info.simTime, vessel_pose);
}
} // namespace gazebo
} // namespace liquidai

GZ_ADD_PLUGIN(
    liquidai::gazebo::RenderPlugin,
    gz::sim::System,
    liquidai::gazebo::RenderPlugin::ISystemConfigure,
    liquidai::gazebo::RenderPlugin::ISystemPreUpdate,
    liquidai::gazebo::RenderPlugin::ISystemUpdate,
    liquidai::gazebo::RenderPlugin::ISystemPostUpdate)