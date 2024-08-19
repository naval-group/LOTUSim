#include "world_plugins/XdynGrpc.h"

namespace liquidai {
namespace gazebo {

std::weak_ptr<WaveRaoInterface> XdynGrpc::m_instance =
    std::shared_ptr<XdynGrpc>(nullptr);

std::optional<json> XdynGrpc::getNewState(
    const std::string &name,
    json previous_state,
    float time_dif)
{
    return std::nullopt;
}
bool XdynGrpc::createConnection(
    const std::string &name,
    const std::vector<std::string> &thrusters_name,
    const std::string &uri)
{
    return true;
}

std::shared_ptr<WaveRaoInterface> XdynGrpc::getInstance()
{
    if (m_instance.expired()) {
        auto instance = std::make_shared<XdynGrpc>();
        m_instance = instance;
        return instance;
    } else {
        return m_instance.lock();
    }
}

}  // namespace gazebo
}  // namespace liquidai