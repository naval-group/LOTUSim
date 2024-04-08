#ifndef __WAVE_RAO_INTERFACE_HH__
#define __WAVE_RAO_INTERFACE_HH__

#include <gz/transport/Node.hh>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>

namespace liquidai {
namespace gazebo {

using json = nlohmann::json;

enum class ConnectionType { XDynWebSocket, XDynGRPC, Manual };

class WaveRaoInterface {
public:
    WaveRaoInterface();

    /**
     * @brief Get the New State object
     *
     * @param name
     * @param previous_state  Json will have time, position, orientation in
     * quaternion
     * @param time_dif In milliseconds
     * @return
     */
    virtual std::optional<json> getNewState(
        const std::string &name, json previous_state, float time_dif) = 0;

    virtual bool
    createConnection(const std::string &name, const std::string &uri) = 0;

protected:
    gz::transport::Node m_node;
};

typedef std::shared_ptr<WaveRaoInterface> ClientPtr;

} // namespace gazebo
} // namespace liquidai
#endif