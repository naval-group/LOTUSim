#ifndef __MANUAL_RAO_HH__
#define __MANUAL_RAO_HH__

#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "PhysicsInterfaceBase.h"

namespace lotusim::gazebo {

/**
 * @brief This is an template example plugin to read files from csv
 *
 */
class ManualRAO : public PhysicsInterfaceBase {
public:
    std::optional<nlohmann::json> getNewState(
        const std::string &name,
        nlohmann::json previous_state,
        float time_dif);
    static std::shared_ptr<PhysicsInterfaceBase> getInstance();
    bool createConnection(
        const std::string &name,
        const std::string &file_path);

private:
    static std::weak_ptr<PhysicsInterfaceBase> m_instance;
};

}  // namespace lotusim::gazebo
#endif