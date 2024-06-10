#ifndef __WAVE_PARAM_NODE_H__
#define __WAVE_PARAM_NODE_H__

#include "gz/sim/Util.hh"
#include "gz_liquidai_msgs/msgs/wavemsg.pb.h"
#include "world_params/ParamNode.h"

#include <liquidai_msgs/msg/wave_param.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gz/msgs/vector2d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/World.hh>
#include <gz/transport/Node.hh>

#include <ctype.h>

namespace liquidai {
namespace gazebo {

/**
 * @brief WaveParam plugin example
 * <WaveNodeParam>
        <topic>wave_param</topic>
        <param_file>dir</param_file>
        <file_type>yaml</file_type>
        <wave_param>
            <amplitude>1.0</amplitude>
            <period>1.0</period>
            <heading>
                <x>1.0</x>
                <y>0</y>
            </heading>
        </wave_param>
    </WaveNodeParam>
 *
 */
class WaveParamPlugin :
    public ParamNode<WaveParameters>,
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate {
public:
    WaveParamPlugin();

    void Configure(
        const gz::sim::Entity &entity,
        const std::shared_ptr<const sdf::Element> &sdf,
        gz::sim::EntityComponentManager &ecm,
        gz::sim::EventManager &eventMgr) override;

    void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) override;

private:
    void logError(std::string err_statement) override;

    bool getParamCB(gz_liquidai_msgs::msgs::WaveParam &msg);

    void setParamMsg(const WaveParameters &msg);

    void addParameter(
        const std::string &name,
        const rclcpp::ParameterValue &default_value,
        const std::string &description = "",
        bool read_only = false);

    rcl_interfaces::msg::SetParametersResult
    dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

private:
    /**
     * @brief Gazebo wave param msg
     *
     */
    gz_liquidai_msgs::msgs::WaveParam m_param_msg;
    std::shared_ptr<gz::transport::Node> m_gz_node;
    gz::transport::Node::Publisher m_param_pub;

    std::shared_ptr<rclcpp::Node> m_nh;
    rclcpp::Publisher<liquidai_msgs::msg::WaveParam>::SharedPtr
        m_wave_param_pub;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        m_dyn_params_handler;
};

} // namespace gazebo
} // namespace liquidai
#endif
