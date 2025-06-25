#ifndef __RENDER_INTERFACE_HH__
#define __RENDER_INTERFACE_HH__

#include <chrono>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>
#include <vector>

#include "lotusim_common/logger.hpp"

namespace lotusim::gazebo {

/**
 * @brief Base class for Render Interface.
 * All different type of render interface must be able to handle all these
 * cases.
 *
 */
class RenderInterfaceBase {
public:
    RenderInterfaceBase(
        const std::string &world_name,
        std::shared_ptr<spdlog::logger> logger)
        : m_logger(logger)
    {
        m_world_name = world_name;

        m_logger = logger::createConsoleAndFileLogger(
            "render_interface",
            "render_interface.txt");
    };

    virtual bool ConfigureInterface(
        const std::shared_ptr<const sdf::Element> &_sdf) = 0;

    /**
     * @brief Send the simulation frame update to renderer
     *
     * @param runTime
     * @param poses
     * @return true
     * @return false
     */
    virtual bool SendPosition(
        const std::chrono::steady_clock::duration &runTime,
        const std::vector<std::pair<std::string, gz::math::Pose3d>> &poses) = 0;

    /**
     * @brief Create a Vessel in the simulation.
     * lotus param will be given and it is up to the user to choose how they
     * will like to interface
     *
     * @param vessel_name
     * @param pose
     * @param sdfptr lotus_param sdfptr
     * @return true
     * @return false
     */
    virtual bool CreateVessel(
        const std::string &vessel_name,
        const gz::math::Pose3d &pose,
        sdf::ElementPtr sdfptr) = 0;

    /**
     * @brief Destroy vessel in the simulation
     *
     * @param vessel_name
     * @return true
     * @return false
     */
    virtual bool DestroyVessel(const std::string &vessel_name) = 0;

    /**
     * @brief CustomPreupdate loop if required
     *
     * @param _info
     * @param _ecm
     * @return true
     * @return false
     */
    virtual bool CustomPreUpdates(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm)
    {
        return true;
    };

    /**
     * @brief CustomUpdate loop if required
     *
     * @param _info
     * @param _ecm
     * @return true
     * @return false
     */
    virtual bool CustomUpdates(
        const gz::sim::UpdateInfo &_info,
        const gz::sim::EntityComponentManager &_ecm)
    {
        return true;
    };

protected:
    std::shared_ptr<spdlog::logger> m_logger;
    gz::transport::Node m_gz_node;
    std::string m_world_name;
};

}  // namespace lotusim::gazebo

#endif