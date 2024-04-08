#ifndef __MAP_PUBLISHER_HPP__
#define __MAP_PUBLISHER_HPP__

#include "map_interface/interface_utils.hpp"

#include <rclcpp/rclcpp.hpp>

/**
 * @brief This interface is to s
 *
 */
class MapPublisher : public rclcpp::Node {

public:
    MapPublisher();

private:
    void AISInfoCB(const liquidai_msgs::msg::AISArray &msg);

private:
    rclcpp::Subscription<liquidai_msgs::msg::AISArray>::SharedPtr m_ais_sub;
    std::string m_topic_name;
    std::string m_ip_address;
};

#endif