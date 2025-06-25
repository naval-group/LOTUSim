#ifndef __ROS_RENDER_INTERFACE_HH__
#define __ROS_RENDER_INTERFACE_HH__

#include <gz/msgs/contacts.pb.h>

#include <boost/asio.hpp>
#include <gz/sim/components/Name.hh>
#include <rclcpp/rclcpp.hpp>

#include "RenderInterface.hpp"
#include "gz/sim/Util.hh"
#include "lotusim_msgs/msg/renderer_cmd.hpp"
#include "lotusim_msgs/msg/vessel_position_array.hpp"

namespace lotusim::gazebo {

/**
 * @brief ROS interface for Renderer
 *
 */

class ROSInterface : public RenderInterfaceBase {
public:
    ROSInterface(
        const std::string &world_name,
        std::shared_ptr<spdlog::logger> logger);
    ~ROSInterface();

    bool ConfigureInterface(const std::shared_ptr<const sdf::Element> &_sdf);

    bool SendPosition(
        const std::chrono::steady_clock::duration &runTime,
        const std::vector<std::pair<std::string, gz::math::Pose3d>> &poses);

    /**
     * @brief Method to create new vessel
     * Handles reading of vessel params, and how the renderer create the vessel
     *
     * @param vessel_name
     * @param type_name
     * @return true
     * @return false
     */
    bool CreateVessel(
        const std::string &vessel_name,
        const gz::math::Pose3d &pose,
        sdf::ElementPtr sdfptr);

    bool DestroyVessel(const std::string &vessel_name);

    virtual bool CustomPreUpdates(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) final;

    virtual bool CustomUpdates(
        const gz::sim::UpdateInfo &_info,
        const gz::sim::EntityComponentManager &_ecm) final;

private:
    void SendCreateMessage(
        const std::string &name,
        const gz::math::Pose3d &pose,
        const std::string &type);

    void SendDestroyMessage(const std::string &name);

    void SendExplodeMessage(const std::string &name);

private:
    rclcpp::Node::SharedPtr m_ros_node;

    rclcpp::Publisher<lotusim_msgs::msg::RendererCmd>::SharedPtr
        m_renderer_cmd_pub;
    rclcpp::Publisher<lotusim_msgs::msg::VesselPositionArray>::SharedPtr
        m_pose_pub;
};

}  // namespace lotusim::gazebo
#endif