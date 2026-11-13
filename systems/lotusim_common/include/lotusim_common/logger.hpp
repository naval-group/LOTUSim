/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_LOGGER_HPP_
#define LOTUSIM_LOGGER_HPP_

#include <spdlog/common.h>

#include <cstdlib>
#include <filesystem>
#include <functional>
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
const spdlog::level::level_enum DEFAULT_LOG_LEVEL = spdlog::level::trace;
#else
const spdlog::level::level_enum DEFAULT_LOG_LEVEL = spdlog::level::info;
#endif

using LogCallback = std::function<void(
    const std::string& logger_name,
    spdlog::level::level_enum level,
    const std::string& message)>;

/**
 * @brief callback for forwarding log messages to external consumers like the UI
 *
 * One callback serves the whole process. Only INFO and ERROR messages from
 * loggers made by createConsoleAndFileLogger() are forwarded.
 * @param callback function called with the logger name, log level, and
 * message; nullptr stops forwarding
 */
void setLogCallback(LogCallback callback);

/**
 * @brief Get log level from environment variable or return default
 * Supports: TRACE, DEBUG, INFO, WARN, ERROR, CRITICAL, OFF
 * Set via: export LOTUSIM_LOG_LEVEL=DEBUG (case-insensitive)
 * @return spdlog::level::level_enum
 */
spdlog::level::level_enum getLogLevelFromEnv();

/**
 * @brief Creates both console and file logger object
 * @param logger_name name of the logger, which is also used for getLogger()
 * function
 * @param file_name name of the log file, in createOrGetLogFolderPath(). If file
 * name exist, it will be overwritten
 * @return std::shared_ptr<spdlog::logger> the new logger, or the logger already
 * registered under logger_name (whichever factory made it, unchanged)
 * @throws std::runtime_error if the log folder or file cannot be created
 */
std::shared_ptr<spdlog::logger> createConsoleAndFileLogger(
    const std::string& logger_name,
    const std::string& file_name);

/**
 * @brief creates basic file logger object
 * @param logger_name name of the logger, which is also used for getLogger()
 * function
 * @param file_name name of the log file, in createOrGetLogFolderPath(). If file
 * name exist, it will be overwritten
 * @return std::shared_ptr<spdlog::logger> the new logger, or the logger already
 * registered under logger_name (whichever factory made it, unchanged)
 * @throws std::runtime_error if the log folder or file cannot be created
 */
std::shared_ptr<spdlog::logger> createBasicFileLogger(
    const std::string& logger_name,
    const std::string& file_name);

/**
 * @brief Get the Logger object
 *
 * @param logger_name  name of the logger
 * @return std::shared_ptr<spdlog::logger> the logger registered under
 * logger_name, or nullptr if there is none
 */
std::shared_ptr<spdlog::logger> getLogger(const std::string& logger_name);

/**
 * @brief Get the log folder, creating it if needed
 *
 * The folder is $LOTUSIM_PATH/lotus_logs (the system temp directory if
 * LOTUSIM_PATH is unset), then "debug" below INFO level, else the time of the
 * first call, so one run keeps one folder.
 * @return std::filesystem::path
 * @throws std::filesystem::filesystem_error if the folder cannot be created
 */
std::filesystem::path createOrGetLogFolderPath();

/**
 * @brief Set logger pattern based on log level
 *
 * Below INFO the message alone ("%v"), else "[%D %T] [%l]: %v".
 */
void setLoggerPattern(std::shared_ptr<spdlog::logger>& logger);

}  // namespace lotusim::logger

#endif  // LOGGER_HPP