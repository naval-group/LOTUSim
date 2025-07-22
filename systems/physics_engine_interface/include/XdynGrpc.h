#ifndef __GRPC_RAO_HH__
#define __GRPC_RAO_HH__

#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "PhysicsInterfaceBase.h"

namespace lotusim::gazebo {

using json = nlohmann::json;

class XdynGrpc : public PhysicsInterfaceBase {
public:
    static std::shared_ptr<PhysicsInterfaceBase> getInstance();
    std::optional<json>
    getNewState(const std::string &name, json previous_state, float time_dif);
    bool createConnection(
        const std::string &name,
        const std::vector<std::string> &thrusters_name,
        const std::string &uri);

private:
    static std::weak_ptr<PhysicsInterfaceBase> m_instance;
};

}  // namespace lotusim::gazebo
#endif