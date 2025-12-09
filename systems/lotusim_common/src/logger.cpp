/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "lotusim_common/logger.hpp"

namespace lotusim::logger {

spdlog::level::level_enum getLogLevelFromEnv()
{
    const char *env_level = std::getenv("LOTUSIM_SPDLOG_LEVEL");
    if (env_level == nullptr) {
        return DEFAULT_LOG_LEVEL;
    }

    std::string level_str(env_level);
    // Convert to uppercase for case-insensitive comparison
    for (auto &c : level_str)
        c = std::toupper(c);

    if (level_str == "TRACE")
        return spdlog::level::trace;
    if (level_str == "DEBUG")
        return spdlog::level::debug;
    if (level_str == "INFO")
        return spdlog::level::info;
    if (level_str == "WARN" || level_str == "WARNING")
        return spdlog::level::warn;
    if (level_str == "ERROR")
        return spdlog::level::err;
    if (level_str == "CRITICAL")
        return spdlog::level::critical;
    if (level_str == "OFF")
        return spdlog::level::off;

    std::cerr << "Invalid LOTUSIM_LOG_LEVEL: " << env_level
              << ". Using default level." << std::endl;
    return DEFAULT_LOG_LEVEL;
}

auto createConsoleAndFileLogger(
    const std::string &logger_name,
    const std::string &file_name) -> std::shared_ptr<spdlog::logger>
{
    try {
        std::shared_ptr<spdlog::logger> logger;

        // Check that there is no logger name conflict
        logger = getLogger(logger_name);
        if (logger) {
            return logger;
        }

        auto console_sink =
            std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

        std::string file_path = createOrGetLogFolderPath();

        file_path = file_path + "/" + file_name;

        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
            file_path,
            true);
        std::filesystem::permissions(
            file_path,
            std::filesystem::perms::owner_all |
                std::filesystem::perms::group_all);

        // creating logger manually
        logger = std::make_shared<spdlog::logger>(
            logger_name,
            spdlog::sinks_init_list{file_sink, console_sink});

        logger->set_level(getLogLevelFromEnv());

        spdlog::flush_every(std::chrono::seconds(3));
        spdlog::register_logger(logger);

        if (DEFAULT_LOG_LEVEL < spdlog::level::info) {
            logger->set_pattern("%v");
        } else {
            logger->set_pattern("[%D %T] [%l]: %v");
        }

        return logger;
    } catch (const spdlog::spdlog_ex &ex) {
        // logs using spdlog default global logger
        spdlog::info(
            "Log initialization failed while creating console and file logger: {}",
            ex.what());
        throw std::runtime_error(
            "createConsoleAndFileLogger:Error: " + std::string(ex.what()));
    }
}

auto createBasicFileLogger(
    const std::string &logger_name,
    const std::string &file_name) -> std::shared_ptr<spdlog::logger>
{
    try {
        std::string file_path = createOrGetLogFolderPath();

        file_path = file_path + "/" + file_name;
        auto logger = spdlog::basic_logger_mt(logger_name, file_path);
        logger->set_level(getLogLevelFromEnv());
        return logger;
    } catch (const spdlog::spdlog_ex &ex) {
        spdlog::info(
            "Log initialization failed while creating basic file logger: {}",
            ex.what());
        throw std::runtime_error(
            "createBasicFileLogger:Error: " + std::string(ex.what()));
    }
}

auto getLogger(const std::string &logger_name)
    -> std::shared_ptr<spdlog::logger>
{
    return spdlog::get(logger_name);
}

std::string createOrGetLogFolderPath()
{
    const char *env_log_file_path =
        std::getenv("LOTUSIM_PATH") ? std::getenv("LOTUSIM_PATH") : nullptr;
    if (env_log_file_path != nullptr) {
        std::filesystem::path p1 = env_log_file_path;
        std::filesystem::current_path(p1);
    } else {
        spdlog::info(
            "No environmental variable found for log file path, create logs folder in /tmp");
        std::filesystem::current_path(std::filesystem::temp_directory_path());
    }

    std::filesystem::path logs_dir =
        std::filesystem::current_path() / "lotus_logs";
    if (DEFAULT_LOG_LEVEL >= spdlog::level::info) {
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d_%H-%M-%S");
        logs_dir /= ss.str();
    }

    if (!std::filesystem::exists(logs_dir)) {
        std::filesystem::create_directories(logs_dir);
        std::filesystem::permissions(
            logs_dir,
            std::filesystem::perms::owner_all |
                std::filesystem::perms::group_all);
    }

    return logs_dir.generic_string() + "/";
}
}  // namespace lotusim::logger
