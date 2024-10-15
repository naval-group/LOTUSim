#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <spdlog/common.h>

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

#include "spdlog/formatter.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

namespace lotusim::logger {

// If lotusim is compiled in debug mode, all debug logs will be triggered.
#if defined(DEBUG)
const spdlog::level::level_enum LOG_LEVEL = spdlog::level::trace;
#else
const spdlog::level::level_enum LOG_LEVEL = spdlog::level::info;
#endif

/**
 * @brief Creates both console and file logger object
 * @param logger_name name of the logger, which is also used for getLogger()
 * function
 * @param file_name name of the log file. If file name exist, it will be
 * overwritten
 * @return std::shared_ptr<spdlog::logger> returns a shared_ptr to logger on
 * successful initializations else std::nullopt on any spdlog exceptions
 */
std::shared_ptr<spdlog::logger> createConsoleAndFileLogger(
    const std::string &logger_name,
    const std::string &file_name);

/**
 * @brief creates basic file logger object
 * @param logger_name name of the logger, which is also used for getLogger()
 * function
 * @param file_name name of the log file
 * @return std::shared_ptr<spdlog::logger> returns a shared_ptr to logger on
 * successful initializations else std::nullopt on any spdlog exceptions
 */
std::shared_ptr<spdlog::logger> createBasicFileLogger(
    const std::string &logger_name,
    const std::string &file_name);

/**
 * @brief Get the Logger object
 *
 * @param logger_name  name of the logger
 * @return std::shared_ptr<spdlog::logger> returns a shared_ptr to logger on
 * successful initializations else std::nullopt on any spdlog exceptions
 */
std::shared_ptr<spdlog::logger> getLogger(const std::string &logger_name);

std::string createOrGetLogFolderPath();

}  // namespace lotusim::logger

#endif  // LOGGER_HPP