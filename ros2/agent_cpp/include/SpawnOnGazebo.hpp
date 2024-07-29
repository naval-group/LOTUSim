#ifndef SPAWN_GZ_HH_
#define SPAWN_GZ_HH_

#include "SpawnInterface.hpp"

using namespace std;

class SpawnOnGazebo : public SpawnInterface {

public:
    SpawnOnGazebo(
        liquidai_msgs::srv::AddEntitySrv::Request request,
        rclcpp::Node::SharedPtr node);

    /// @brief Spawn command called on OnConfigure()
    /// @return Returns true if successfully spawned entity
    bool spawn() override;

private:
    rclcpp::Client<liquidai_msgs::srv::AddEntitySrvArray>::SharedPtr
        add_entity_client_;
    liquidai_msgs::srv::AddEntitySrv::Request request_;
    rclcpp::Node::SharedPtr node_;
};

#endif