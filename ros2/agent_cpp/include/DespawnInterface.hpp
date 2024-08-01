#ifndef DESPAWN_INTERFACE_HH_
#define DESPAWN_INTERFACE_HH_

#include "liquidai_msgs/msg/add_entity.hpp"
#include "liquidai_msgs/srv/add_entity_srv.hpp"
#include "liquidai_msgs/srv/add_entity_srv_array.hpp"
#include "liquidai_msgs/srv/remove_entity.hpp"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

class DespawnInterface {
public:
    virtual bool despawn() = 0;
};

#endif