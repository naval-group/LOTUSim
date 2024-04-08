#ifndef __WIND_PARAM_H__
#define __WIND_PARAM_H__

#include <chrono>
#include <map>

#include "DataType.h"
#include "yaml-cpp/node/node.h"
#include "yaml-cpp/parser.h"

namespace liquidai {

/**
 * @brief Expected format
 * {
 *    {
 *        velocity: 1.0,
 *        direction: {1,1},
 *        time: 1.2
 *    },
 *    {
 *        velocity: 1.0,
 *        direction: {1,1},
 *        time: 1.5
 *    }
 * }
 *
 */
class WindYamlLoader {
public:
    WindYamlLoader(
        const std::string &file_path,
        std::map<std::chrono::duration<int>, liquidai::WindParameters>
            &output){};

    void operator()(
        const std::string &file_path,
        std::map<std::chrono::duration<int>, liquidai::WindParameters> &output)
    {
        YAML::Node config = YAML::LoadFile(file_path);
        for (auto &&i : config) {
            liquidai::WindParameters param;
            param.velocity = i["velocity"].as<double>();
            param.direction[0] = i["direction"][0].as<int>();
            param.direction[1] = i["direction"][1].as<int>();
            output[std::chrono::seconds(i["time"].as<int>())] = param;
        }
    };
};

} // namespace liquidai
#endif // __WIND_PARAM_H__
