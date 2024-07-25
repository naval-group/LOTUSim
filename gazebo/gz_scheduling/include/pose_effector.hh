#include "generic_effector.hh"

/// @brief An example of an effector that directly modifies the Pose of a Gazebo
/// entity
class PoseEffector : public GenericEffector {
public:
    // Sets up its data in the constructor
    PoseEffector(
        std::shared_ptr<gz::transport::Node> m_gz_node,
        const gz::msgs::Pose &msg,
        std::string worldName)
        : GenericEffector()
    {
        msg_ = msg;
        worldName_ = worldName;
        m_gz_node_ = m_gz_node;
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
    };

private:
    std::shared_ptr<gz::transport::Node> m_gz_node_;
    gz::msgs::Pose msg_;
    std::string worldName_;
};