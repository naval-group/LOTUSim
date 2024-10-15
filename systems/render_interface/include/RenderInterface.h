#ifndef __RENDER_INTERFACE_HH__
#define __RENDER_INTERFACE_HH__

#include <chrono>
#include <gz/common/Console.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>
#include <vector>

#include "logging_system/logger.hpp"

namespace lotusim::gazebo {

/**
 * @brief Base class for Render Interface.
 * All different type of render interface must be able to handle all these
 * cases.
 *
 */
class RenderInterfaceBase {
public:
    RenderInterfaceBase(std::shared_ptr<spdlog::logger> logger)
        : m_logger(logger) {};

    virtual bool ConfigureInterface(
        const std::shared_ptr<const sdf::Element> &_sdf) = 0;

    virtual bool SendPosition(
        const std::chrono::steady_clock::duration &runTime,
        const std::vector<std::pair<std::string, gz::math::Pose3d>> &poses) = 0;

    virtual bool CreateVessel(
        const std::string &vessel_name,
        const gz::math::Pose3d &pose,
        sdf::ElementPtr sdfptr) = 0;

    virtual bool DestroyVessel(const std::string &vessel_name) = 0;

    virtual bool CustomPreUpdates(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm)
    {
        return true;
    };

    virtual bool CustomUpdates(
        const gz::sim::UpdateInfo &_info,
        const gz::sim::EntityComponentManager &_ecm)
    {
        return true;
    };

protected:
    std::shared_ptr<spdlog::logger> m_logger;
    gz::transport::Node m_gz_node;
};

}  // namespace lotusim::gazebo

#endif