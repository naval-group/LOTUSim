#include "map_interface/map_interface.hpp"

MapPublisher::MapPublisher()
    : m_topic_name("ais"), rclcpp::Node("map_publisher", rclcpp::NodeOptions())
{
    declare_parameter("ip_address", rclcpp::ParameterValue("localhost"));
    declare_parameter("ais_topic_name", rclcpp::ParameterValue("ais"));
    get_parameter("ip_address", m_ip_address);
    get_parameter("ais_topic_name", m_topic_name);

    m_ais_sub = this->create_subscription<liquidai_msgs::msg::AISArray>(
        m_topic_name,
        1,
        std::bind(&MapPublisher::AISInfoCB, this, std::placeholders::_1));
}

void MapPublisher::AISInfoCB(const liquidai_msgs::msg::AISArray &msg)
{
    for (auto &&ais : msg.data) {
        pubAIS(m_ip_address, ais, PublisherMethod::HTTP);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}