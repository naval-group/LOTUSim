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
    m_render_interface->ConfigureInterface(_sdf, m_gz_node);
}

void RenderPlugin::PreUpdate(
    const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &_ecm)
{
    _ecm.EachNew<
        gz::sim::components::ModelSdf,
        gz::sim::components::ParentEntity>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::ModelSdf *_model,
            const gz::sim::components::ParentEntity *_parent) -> bool {
            sdf::Model data = _model->Data();
            bool publish_render{false};
            std::string unity_type_name = "";
            sdf::ElementPtr sdfptr = data.Element();
            if (sdfptr->HasElement("publish_render")) {
                publish_render = sdfptr->Get<bool>("publish_render");
            }
            if (sdfptr->HasElement("unity_type_name")) {
                unity_type_name = sdfptr->Get<std::string>("unity_type_name");
            }
            gzmsg << "RenderPlugin checking model: " << data.Name()
                  << " publishing model " << publish_render << "\n";
            if (publish_render) {
                auto name_opt =
                    _ecm.Component<gz::sim::components::Name>(_entity);
                m_vessel_entity[name_opt->Data()] = _entity;

                if(unity_type_name != ""){
                    // m_render_interface->SendCreateMessage(name_opt->Data(), unity_type_name);
                }
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
    liquidai::gazebo::RenderPlugin::ISystemPostUpdate)