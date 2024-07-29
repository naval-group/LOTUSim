#include "generic_effector.hh"

/// @brief An example of an effector that directly modifies the Pose of a Gazebo
/// entity
class PoseEffector : public GenericEffector {
public:
    // Sets up its data in the constructor
    PoseEffector(
        std::shared_ptr<gz::transport::Node> m_gz_node,
        const gz::msgs::Pose &msg,
        std::string worldName,
        gz::sim::EntityComponentManager *_ecm)
        : GenericEffector()
    {
        msg_ = msg;
        worldName_ = worldName;
        m_gz_node_ = m_gz_node;
        ecm_ = _ecm;

        auto color = gz::math::Color(1.0, 1.0, 1.0, 1.0);
        EditEntityVisualColor(6, color);
    };

    // Applies its logic in the apply_effector function
    void apply_effector()
    {
        gz::msgs::Boolean rep;
        bool result;
        unsigned int timeout = 5000;

        std::string service = "/world/" + this->worldName_ + "/set_pose";
        bool executed =
            m_gz_node_->Request(service, msg_, timeout, rep, result);

        WallColorDemo();
    };

    void WallColorDemo()
    {
        auto color = gz::math::Color(1.0, 1.0, 1.0, 1.0);
        switch (msg_.id()) {
        case 7:
            color = gz::math::Color(1.0, 0.0, 0.0, 1.0);
            break;
        case 23:
            color = gz::math::Color(1.0, 1.0, 0.0, 1.0);
            break;
        case 39:
            color = gz::math::Color(0.0, 1.0, 0.0, 1.0);
            break;
        case 55:
            color = gz::math::Color(0.0, 0.0, 1.0, 1.0);
            break;
        default:
            break;
        }

        EditEntityVisualColor(msg_.id() + 2, color);
        EditEntityVisualColor(msg_.id() + 5, color);
        EditEntityVisualColor(msg_.id() + 8, color);
        EditEntityVisualColor(msg_.id() + 11, color);

        gz::math::Pose3d pose_after =
            ecm_->Component<gz::sim::components::Pose>(msg_.id())->Data();
        if (pose_after.X() >= 10 && has_published_poc_data == false) {
            has_published_poc_data = true;

            EditEntityVisualColor(6, color);
            gz::msgs::WorldControl req;
            req.set_pause(true);
            gz::msgs::Boolean rep;
            bool result;
            unsigned int timeout = 5000;
            std::string service = "/world/" + worldName_ + "/control";

            bool executed =
                m_gz_node_->Request(service, req, timeout, rep, result);
        }
    }

    void EditEntityVisualColor(
        const gz::sim::Entity &_entity, gz::math::Color _color)
    {
        std::function<void(const gz::msgs::Boolean &, const bool)> cb =
            [](const gz::msgs::Boolean & /*_rep*/, const bool _result) {
                if (!_result)
                    gzerr << "Error setting material color configuration"
                          << " on visual" << std::endl;
            };
        gz::msgs::Visual req;
        req.set_id(_entity);

        gz::msgs::Set(req.mutable_material()->mutable_ambient(), _color);
        gz::msgs::Set(req.mutable_material()->mutable_diffuse(), _color);
        gz::msgs::Set(req.mutable_material()->mutable_specular(), _color);
        gz::msgs::Set(req.mutable_material()->mutable_emissive(), _color);

        auto service =
            std::string("/world/" + this->worldName_ + "/visual_config");
        auto materialCmdService =
            gz::transport::TopicUtils::AsValidTopic(service);
        if (materialCmdService.empty()) {
            gzerr << "Invalid material command service topic provided"
                  << std::endl;
            return;
        }
        this->m_gz_node_->Request(materialCmdService, req, cb);
    }

private:
    std::shared_ptr<gz::transport::Node> m_gz_node_;
    gz::msgs::Pose msg_;
    std::string worldName_;
    gz::sim::EntityComponentManager *ecm_;
    bool has_published_poc_data = false;
};