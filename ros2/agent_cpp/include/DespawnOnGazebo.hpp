#ifndef DESPAWN_GZ_HH_
#define DESPAWN_GZ_HH_

#include "DespawnInterface.hpp"

using namespace std;

class DespawnOnGazebo : public DespawnInterface {
public:
    DespawnOnGazebo(
        liquidai_msgs::srv::RemoveEntity::Request request,
        rclcpp::Node::SharedPtr node);

public:
    /// @brief Despawn command called on OnCleanup() or OnShutdown()
    /// @return Returns true if successfully despawned
    bool despawn() override;

private:
    rclcpp::Client<liquidai_msgs::srv::RemoveEntity>::SharedPtr
        remove_entity_client_;
    liquidai_msgs::srv::RemoveEntity::Request request_;
    rclcpp::Node::SharedPtr node_;
};

#endif