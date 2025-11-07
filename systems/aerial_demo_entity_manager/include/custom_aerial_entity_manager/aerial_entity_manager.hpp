#ifndef AERIAL_ENTITY_MANAGER_HPP_
#define AERIAL_ENTITY_MANAGER_HPP_

#include "entity_manager/entity_manager.hpp"

namespace lotusim::gazebo {

using ClientGoalHandleMASCmd =
    rclcpp_action::ClientGoalHandle<lotusim_msgs::action::MASCmd>;

class AerialEntityManager : public EntityManager {
public:
    AerialEntityManager();
    ~AerialEntityManager();

private:
    void customUserConfiguration(
        const std::shared_ptr<const sdf::Element> &_sdf);

    void customUserAddEntity(
        const lotusim_msgs::msg::MASCmd &msg) override final;

    void customUserDeleteEntity(
        const lotusim_msgs::msg::MASCmd &msg) override final;

    void resultCB(const ClientGoalHandleMASCmd::WrappedResult &result);

    void goalResponseCB(ClientGoalHandleMASCmd::SharedPtr goal_handle);

private:
    rclcpp_action::Client<lotusim_msgs::action::MASCmd>::SharedPtr
        m_aerial_entity_manager_client;

    std::string m_aerial_namespace;
};
}  // namespace lotusim::gazebo
#endif