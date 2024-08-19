#ifndef __MANUAL_RAO_HH__
#define __MANUAL_RAO_HH__

#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "world_plugins/WaveRaoInterface.h"

namespace liquidai {
namespace gazebo {

/**
 * @brief This is an template example plugin to read files from csv
 *
 */
class ManualRAO : public WaveRaoInterface {
public:
    std::optional<nlohmann::json> getNewState(
        const std::string &name,
        nlohmann::json previous_state,
        float time_dif);
    static std::shared_ptr<WaveRaoInterface> getInstance();
    bool createConnection(
        const std::string &name,
        const std::string &file_path);

private:
    static std::weak_ptr<WaveRaoInterface> m_instance;
};

}  // namespace gazebo
}  // namespace liquidai
#endif