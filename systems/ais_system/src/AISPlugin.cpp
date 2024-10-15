
#include "AISPlugin.h"

namespace lotusim::gazebo {

AISPlugin::AISPlugin()
    : m_update_period(std::chrono::seconds(2))
    , m_last_pub(std::chrono::seconds(0))
{
    m_gz_node = std::make_shared<gz::transport::Node>();
    m_ais_pub = m_gz_node->Advertise<gz_liquidai_msgs::msgs::AISArray>("AIS");
    m_logger = logger::createConsoleAndFileLogger("AIS", "AIS_sensor.txt");
}

void AISPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr)
{
    const sdf::Element *sdfPtr = _sdf.get();
    if (sdfPtr && sdfPtr->HasElement("period")) {
        m_update_period = std::chrono::seconds(sdfPtr->Get<int>("period"));
    } else {
        m_logger->error("AISPlugin::Configure: sdfPtr not retrieved");
    }
}

void AISPlugin::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
    _ecm.EachNew<gz::sim::components::ModelSdf>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::ModelSdf *_model) -> bool {
            const sdf::Model &data = _model->Data();
            bool publish_ais{false};
            sdf::ElementPtr sdfptr = data.Element();
            if (sdfptr->HasElement("publish_ais")) {
                publish_ais = sdfptr->Get<bool>("publish_ais");
            }
            m_logger->info(
                "AISPlugin::PostUpdate: New AISPlugin checking model: {}. Publish AIS: {}",
                data.Name(),
                publish_ais);
            if (publish_ais) {
                m_vessel_entities.push_back(_entity);
            }
            return true;
        });

    m_pose.clear();
    for (auto &&entity : m_vessel_entities) {
        gz::math::Pose3d pose =
            _ecm.Component<gz::sim::components::Pose>(entity)->Data();
        m_pose.push_back({scopedName(entity, _ecm), pose});
    }

    if (_info.simTime - m_last_pub >= m_update_period) {
        m_last_pub = _info.simTime;
        gz_liquidai_msgs::msgs::AISArray array_msg;

        for (auto &&entity : m_vessel_entities) {
            auto coord_opt = gz::sim::sphericalCoordinates(entity, _ecm);
            auto vel_opt =
                _ecm.Component<gz::sim::components::WorldLinearVelocity>(
                    entity);
            if (coord_opt && vel_opt) {
                gz_liquidai_msgs::msgs::AISArray_AIS *msg =
                    array_msg.add_data();
                msg->set_user_id(entity);
                msg->set_longitude(coord_opt.value().Y());
                msg->set_latitude(coord_opt.value().X());
                double vel = std::sqrt(
                    std::pow(vel_opt->Data()[0], 2) +
                    std::pow(vel_opt->Data()[1], 2));
                msg->set_sog(vel);
                msg->set_name(scopedName(entity, _ecm));
                // msg->set_true_heading(
                //     coord_opt.value().HeadingOffset().Degree());

                // Filler
                msg->set_navigation_status(0);
                msg->set_rot(0);
                msg->set_positional_accuracy(0);
                msg->set_true_heading(0);
                msg->set_cog(0);
            }
        }
        if (array_msg.data_size() > 0) {
            m_ais_pub.Publish(array_msg);
        }
    }
}

}  // namespace lotusim::gazebo

GZ_ADD_PLUGIN(
    lotusim::gazebo::AISPlugin,
    gz::sim::System,
    lotusim::gazebo::AISPlugin::ISystemConfigure,
    lotusim::gazebo::AISPlugin::ISystemPostUpdate)