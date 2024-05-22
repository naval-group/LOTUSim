#ifndef __WAVE_RAO_HH__
#define __WAVE_RAO_HH__

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>

// #include "world_plugins/ManualRAO.h"
// #include "world_plugins/XdynGrpc.h"
#include "world_plugins/WaveRaoInterface.h"
#include "world_plugins/XdynWebSocket.h"

namespace liquidai {
namespace gazebo {

/**
 * @brief This is a plugin for interfacing with external physics library
 * It will
 * - handle all vehicles in the world that require interfacing with
 * physics calculation by asynchronously sending out request.
 * - handle the model transition between different boundaries and swap server
 *
 * IT WILL NOT HANDLE receiving vessel commands and calculating when is the
 * transition. These are to be handled by the integrator in the interface with
 * the server. Reason being that these elements are server defined and
 * non-standard
 *
 * All mapping is stored as model entity. The only non-model entity stored is
 * base link entity
 *
 * Models that requires update should include <physics_server_interface> tag.
 * Those without the tag are assumed to be static object.
 *
 * In the <physics_server_interface> tag, user is suppose to define all the
 * server used by vessel in different domain if needed.
 * E.g if the vessel is a submarine, only the underwater and surface definition
 * is needed.
 * User needs to define <init_state> to state the initial server to use.
 *
 * Plugin format:
 *
 * <plugin filename="wave_rao_plugin" name="liquidai::gazebo::WaveRaoPlugin">
 * </plugin>
 *
 * <model name="drone">
 *  <physics_server_interface>
 *   <aerial>
 *     <ConnectionType>XDynWebSocket</ConnectionType>
 *     <uri>127.0.0.1:1234</uri>
 *     <thrusters>
 *       <thursters1>propeller1</thursters1>
 *     </thrusters>
 *   </aerial>
 *   <surface>
 *     <ConnectionType>XDynGRPC</ConnectionType>
 *     <uri>127.0.0.1:1235</uri>
 *   </surface>
 *   <underwater>
 *     <ConnectionType>Manual</ConnectionType>
 *     <uri>127.0.0.1:1236</uri>
 *   </underwater>
 *   <init_state>surface</init_state>
 *  </physics_server_interface>
 * </model>
 */

class WaveRaoPlugin :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemUpdate {

public:
    WaveRaoPlugin();

    void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_eventMgr);

    void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) final;

    void Update(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) final;

private:
    /**
     * @brief Create a physics interface. Users are to edit this function based
     * on the interface created
     *
     * @param entity
     * @param name
     * @param protocol_type
     * @param sdf
     * @return std::shared_ptr<WaveRaoInterface>
     */
    std::shared_ptr<WaveRaoInterface> createConnection(
        const gz::sim::Entity &entity,
        const std::string &name,
        const ConnectionType &protocol_type,
        const sdf::ElementPtr sdf);

    /**
     * @brief When vessel trasit between domain and require to switch server
     *
     * @param _vessel model entity in gz
     * @param _new_mode the domain vessel is switching to
     * @return true
     * @return false
     */
    bool vesselTransition(gz::sim::Entity _vessel, DomainType _new_mode);

private:
    /**
     * @brief Flag for debugging
     *
     */
    bool m_debug;

    /**
     * @brief Mapping for vessel name to base_link entity
     *
     */
    std::unordered_map<std::string, gz::sim::Entity> m_vessels_base_link_map;

    /**
     * @brief Mapping for vessel name to the model entity
     *
     */
    std::unordered_map<std::string, gz::sim::Entity> m_vessels_model_map;

    /**
     * @brief Mapping for model entity to vessel name
     *
     */
    std::unordered_map<gz::sim::Entity, std::string> m_vessels_name_map;

    /**
     * @brief Current vessel interface connected
     *
     */
    std::unordered_map<gz::sim::Entity, std::shared_ptr<WaveRaoInterface>>
        m_current_vessel_interface;

    /**
     * @brief Current domain the vehicle is in.
     *
     */
    std::unordered_map<gz::sim::Entity, DomainType> m_vehicle_current_mode;

    /**
     * @brief Mapping for user set aerial server to use
     *
     */
    std::unordered_map<gz::sim::Entity, std::shared_ptr<WaveRaoInterface>>
        m_aerial_interface;

    /**
     * @brief Mapping for user set surface server to use
     *
     */
    std::unordered_map<gz::sim::Entity, std::shared_ptr<WaveRaoInterface>>
        m_surface_interface;

    /**
     * @brief Mapping for user set underwater server to use
     *
     */
    std::unordered_map<gz::sim::Entity, std::shared_ptr<WaveRaoInterface>>
        m_underwater_interface;
};

} // namespace gazebo
} // namespace liquidai
#endif