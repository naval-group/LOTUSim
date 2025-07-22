#include "ManualRAO.h"

namespace lotusim::gazebo {

std::weak_ptr<PhysicsInterfaceBase> ManualRAO::m_instance =
    std::shared_ptr<ManualRAO>(nullptr);

std::optional<json> ManualRAO::getNewState(
    const std::string &name,
    json previous_state,
    float time_dif)
{
    return std::nullopt;
}
bool ManualRAO::createConnection(
    const std::string &name,
    const std::string &file_path)
{
    return true;
}

std::shared_ptr<PhysicsInterfaceBase> ManualRAO::getInstance()
{
    if (m_instance.expired()) {
        auto instance = std::make_shared<ManualRAO>();
        m_instance = instance;
        return instance;
    } else {
        return m_instance.lock();
    }
}
}  // namespace gazebo
}  // namespace lotusim