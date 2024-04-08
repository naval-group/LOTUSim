#include "world_plugins/AISPlugin.h"

namespace liquidai {
namespace gazebo {

namespace ip = boost::asio::ip;

AISPlugin::AISPlugin()
    : m_update_period(std::chrono::seconds(2))
    , m_last_pub(std::chrono::seconds(0))
{
    m_gz_node = std::make_shared<gz::transport::Node>();
    m_ais_pub =
        m_gz_node->Advertise<gz_liquidai_plugins_msgs::msgs::AISArray>("AIS");

    // defining endpoint
    m_socket_ptr = std::make_shared<ip::udp::socket>(m_io_context);
    m_endpoint = ip::udp::endpoint(ip::address::from_string("127.0.0.1"), 8000);
}

AISPlugin::~AISPlugin() { m_socket_ptr->close(); }

void AISPlugin::SendPosition()
{
    boost::system::error_code err;
    std::string msg = "{";
    msg += "\"name\": \"" + m_pose[0].first + "\", ";
    msg += " \"position\": {";
    msg += " \"x\": " + std::to_string(m_pose[0].second.X()) + ", ";
    msg += " \"y\": " + std::to_string(m_pose[0].second.Y()) + ", ";
    msg += " \"z\": " + std::to_string(m_pose[0].second.Z());
    msg += "}, \"rotation\": {";
    /// Quad
    msg += " \"x\": " + std::to_string(m_pose[0].second.Rot().X()) + ", ";
    msg += " \"y\": " + std::to_string(m_pose[0].second.Rot().Y()) + ", ";
    msg += " \"z\": " + std::to_string(m_pose[0].second.Rot().Z()) + ", ";
    msg += " \"w\": " + std::to_string(m_pose[0].second.Rot().W());

    // Euler
    // msg += " \"x\": " + std::to_string(m_pose[0].second.Rot().Roll()) + ", ";
    // msg += " \"y\": " + std::to_string(m_pose[0].second.Rot().Pitch()) + ",
    // "; msg += " \"z\": " + std::to_string(m_pose[0].second.Rot().Yaw()) + ",
    // "; msg += " \"w\": 0";

    msg += "}}";

    auto sent =
        m_socket_ptr->send_to(boost::asio::buffer(msg), m_endpoint, 0, err);
}

void AISPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr)
{
    auto sdfPtr = const_cast<sdf::Element *>(_sdf.get());
    if (sdfPtr->HasElement("period")) {
        m_update_period = std::chrono::seconds(sdfPtr->Get<int>("period"));
    }

    auto parent = _ecm.EntitiesByComponents(gz::sim::v7::components::Model());
    for (auto &&v : parent) {
        auto name_opt = _ecm.Component<gz::sim::v7::components::Name>(v);
        if (name_opt && name_opt->Data().find("vessel") != std::string::npos) {
            m_vessel_entities.push_back(v);
        }
    }
    gzmsg << "AIS plugin configuration, vessels detected "
          << m_vessel_entities.size() << std::endl;

    m_socket_ptr->open(ip::udp::v4());
}

void AISPlugin::Update(
    const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{
    // Bring bol for update out, and merge the 2 for loops

    m_pose.clear();
    for (auto &&entity : m_vessel_entities) {
        gz::math::Pose3d pose =
            _ecm.Component<gz::sim::v7::components::Pose>(entity)->Data();
        m_pose.push_back({scopedName(entity, _ecm), pose});
    }

    if (_info.simTime - m_last_pub >= m_update_period) {
        m_last_pub = _info.simTime;
        gz_liquidai_plugins_msgs::msgs::AISArray array_msg;

        for (auto &&entity : m_vessel_entities) {
            auto coord_opt = gz::sim::v7::sphericalCoordinates(entity, _ecm);
            auto vel_opt =
                _ecm.Component<gz::sim::v7::components::WorldLinearVelocity>(
                    entity);
            if (coord_opt && vel_opt) {
                gz_liquidai_plugins_msgs::msgs::AISArray_AIS *msg =
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
    SendPosition();
    m_io_context.run_one();
}

} // namespace gazebo
} // namespace liquidai

GZ_ADD_PLUGIN(
    liquidai::gazebo::AISPlugin,
    gz::sim::System,
    liquidai::gazebo::AISPlugin::ISystemConfigure,
    liquidai::gazebo::AISPlugin::ISystemUpdate)