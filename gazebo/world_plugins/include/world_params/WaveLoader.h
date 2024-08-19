#ifndef __WAVE_PARAM_H__
#define __WAVE_PARAM_H__

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <map>

#include "DataType.h"
#include "yaml-cpp/node/node.h"
#include "yaml-cpp/parser.h"

namespace liquidai {

/**
 * @brief This file to include other file types too.
 *
 */

/**
 * @brief Expected format
 * {
 *    {
 *        velocity: 1.0,
 *        direction: {1,1},
 *        period: 1.0,
 *        time: 0
 *    },
 *    {
 *        velocity: 1.0,
 *        direction: {1,1},
 *        period: 1.0,
 *        time: 1.5
 *    }
 * }
 *
 */
class WaveYamlLoader {
public:
    WaveYamlLoader(
        const std::string &file_path,
        std::map<std::chrono::steady_clock::duration, liquidai::WaveParameters>
            &output){};

    void operator()(
        const std::string &file_path,
        std::map<std::chrono::steady_clock::duration, liquidai::WaveParameters>
            &output)
    {
        YAML::Node config = YAML::LoadFile(file_path);
        int start_time;
        if (config.size() == 0) {
            return;
        } else {
            start_time = config[0]["time"].as<double>();
        }
        for (auto &&i : config) {
            liquidai::WaveParameters param;
            param.amplitude = i["amplitude"].as<double>();
            param.direction[0] = i["direction"][0].as<double>();
            param.direction[1] = i["direction"][1].as<double>();
            param.period = i["period"].as<double>();
            output[std::chrono::seconds(i["time"].as<int>() - start_time)] =
                param;
        }
    };
};

}  // namespace liquidai
#endif  // __WAVE_PARAM_H__
