
#ifndef __DATA_FILE_HELPER_H__
#define __DATA_FILE_HELPER_H__

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>

#include "DataType.h"
#include "WaveLoader.h"
#include "WindLoader.h"

namespace liquidai {

enum class FileFormat
{
    YAML,
    UNKNOWN
};

std::unordered_map<std::string, FileFormat> file_type_mapping{
    {"yaml", FileFormat::YAML},
    {"unknown", FileFormat::UNKNOWN}};

/**
 * @brief This is a factory class to load different format
 *
 * @tparam DataType
 * @param file_path Path of file you will like to load.
 * @param format The file format
 * @param output The output params
 * @return true
 * @return false
 */
template <typename DataType>
bool loadFile(
    const std::string &file_path,
    FileFormat format,
    std::map<std::chrono::steady_clock::duration, DataType> &output)
{
    return false;
};

bool loadFile(
    const std::string &file_path,
    FileFormat format,
    std::map<std::chrono::steady_clock::duration, liquidai::WaveParameters>
        &output)
{
    switch (format) {
        case (FileFormat::YAML): {
            WaveYamlLoader(file_path, output);
            if (!output.empty()) {
                return true;
            }
        }
        default: {
            return false;
        }
    }
};

}  // namespace liquidai

#endif  // __DATA_FILE_HELPER_H__