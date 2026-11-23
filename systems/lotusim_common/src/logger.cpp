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

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <utility>

namespace lotusim::logger {

namespace {  // private

using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

std::mutex g_logger_creation_mutex;

uint8_t toDiagnosticLevel(spdlog::level::level_enum level)
{
    switch (level) {
        case spdlog::level::err:
        case spdlog::level::critical:
            return DiagnosticStatus::ERROR;
        case spdlog::level::warn:
            return DiagnosticStatus::WARN;
        default:
            return DiagnosticStatus::OK;
    }
}

class DiagnosticsSink : public spdlog::sinks::base_sink<std::mutex> {
private:
    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::Publisher<DiagnosticArray>::SharedPtr m_diagnostics_pub;
    std::string m_hardware_id;

public:
    // Off until enable(), so an unused sink costs one atomic load per log.
    DiagnosticsSink() { set_level(spdlog::level::off); }

    void enable(const std::shared_ptr<rclcpp::Node>& node, const std::string& hardware_id)
    {
        auto pub = node->create_publisher<DiagnosticArray>("/diagnostics", rclcpp::QoS(10));
        {
            std::lock_guard<std::mutex> lock(mutex_);
            m_node = node;
            m_hardware_id = hardware_id;
            m_diagnostics_pub = pub;
        }
        // Records below this level are dropped by the logger before reaching
        // this sink (lock-free level check).
        set_level(getLogLevelFromEnv());
    }
    void disable()
    {
        set_level(spdlog::level::off);
        std::lock_guard<std::mutex> lock(mutex_);
        m_diagnostics_pub.reset();
        m_node.reset();
    }
protected:
    void sink_it_(const spdlog::details::log_msg& msg) override
    {
        if (!m_node || !m_diagnostics_pub) {
            return;  // in case diagnostics are not enabled for this logger
        }

        DiagnosticStatus status;
        status.name.assign(msg.logger_name.data(), msg.logger_name.size());
        status.message.assign(msg.payload.data(), msg.payload.size());
        status.hardware_id = m_hardware_id;
        status.level = toDiagnosticLevel(msg.level);

        DiagnosticArray array_msg;
        array_msg.header.stamp = m_node->now();
        array_msg.status.push_back(std::move(status));
        try{
            m_diagnostics_pub->publish(array_msg);
        } catch (...) {
            // no log, swallow error
        }
    }
    void flush_() override {}
};
}  // namespace

bool enableDiagnostics(
    const std::shared_ptr<spdlog::logger>& logger, const std::shared_ptr<rclcpp::Node>& node, const std::string& hardware_id)
{
    if (!logger || !node) return false;
    for (auto& sink : logger->sinks()) {
        if (auto diag = std::dynamic_pointer_cast<DiagnosticsSink>(sink)) {
            diag->enable(node, hardware_id);
            return true;
        }
    }
    return false; 
}

void disableDiagnostics(const std::shared_ptr<spdlog::logger>& logger)
{
    if (!logger) return;
    for (auto& sink : logger->sinks()) {
        if (auto diag = std::dynamic_pointer_cast<DiagnosticsSink>(sink)) {
            diag->disable();
        }
    }
}

spdlog::level::level_enum getLogLevelFromEnv()
{
    const char* env_level = std::getenv("LOTUSIM_LOG_LEVEL");
    if (env_level == nullptr) {
        return DEFAULT_LOG_LEVEL;
    }

    std::string level_str(env_level);
    // Convert to uppercase for case-insensitive comparison
    for (auto& c : level_str)
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
    const std::string& logger_name,
    const std::string& file_name) -> std::shared_ptr<spdlog::logger>
{
    std::lock_guard<std::mutex> creation_lock(g_logger_creation_mutex);
    try {
        std::shared_ptr<spdlog::logger> logger;

        // Check that there is no logger name conflict
        logger = getLogger(logger_name);
        if (logger) {
            return logger;
        }

        auto console_sink =
            std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

        std::filesystem::path file_path = createOrGetLogFolderPath();
        file_path /= file_name;

        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
            file_path,
            true);

        auto diagnostics_sink = std::make_shared<DiagnosticsSink>();

        try {
            std::filesystem::permissions(
                file_path,
                std::filesystem::perms::owner_all |
                    std::filesystem::perms::group_all);
        } catch (const std::filesystem::filesystem_error& perm_ex) {
            spdlog::warn(
                "Failed to set file permissions for {}: {}",
                file_path.string(),
                perm_ex.what());
        }

        // creating logger manually
        logger = std::make_shared<spdlog::logger>(
            logger_name,
            spdlog::sinks_init_list{file_sink, console_sink, diagnostics_sink});

        logger->set_level(getLogLevelFromEnv());

        spdlog::flush_every(std::chrono::seconds(3));
        spdlog::register_logger(logger);
        setLoggerPattern(logger);

        return logger;
    } catch (const spdlog::spdlog_ex& ex) {
        // logs using spdlog default global logger
        spdlog::info(
            "Log initialization failed while creating console and file logger: {}",
            ex.what());
        throw std::runtime_error(
            "createConsoleAndFileLogger:Error: " + std::string(ex.what()));
    }
}

auto createBasicFileLogger(
    const std::string& logger_name,
    const std::string& file_name) -> std::shared_ptr<spdlog::logger>
{
    std::lock_guard<std::mutex> creation_lock(g_logger_creation_mutex);
    try {
        std::shared_ptr<spdlog::logger> logger;

        logger = getLogger(logger_name);
        if (logger) {
            return logger;
        }

        std::filesystem::path file_path = createOrGetLogFolderPath();
        file_path /= file_name;

        logger = spdlog::basic_logger_mt(logger_name, file_path, true);
        logger->set_level(getLogLevelFromEnv());

        try {
            std::filesystem::permissions(
                file_path,
                std::filesystem::perms::owner_all |
                    std::filesystem::perms::group_all);
        } catch (const std::filesystem::filesystem_error& perm_ex) {
            spdlog::warn(
                "Failed to set file permissions for {}: {}",
                file_path.string(),
                perm_ex.what());
        }

        spdlog::flush_every(std::chrono::seconds(3));
        setLoggerPattern(logger);

        return logger;
    } catch (const spdlog::spdlog_ex& ex) {
        spdlog::info(
            "Log initialization failed while creating basic file logger: {}",
            ex.what());
        throw std::runtime_error(
            "createBasicFileLogger:Error: " + std::string(ex.what()));
    }
}

auto getLogger(const std::string& logger_name)
    -> std::shared_ptr<spdlog::logger>
{
    return spdlog::get(logger_name);
}

std::filesystem::path createOrGetLogFolderPath()
{
    std::filesystem::path base_path;

    const char* env_log_file_path = std::getenv("LOTUSIM_PATH");
    if (env_log_file_path != nullptr) {
        base_path = env_log_file_path;
    } else {
        spdlog::info(
            "No environmental variable found for log file path, create logs folder in /tmp");
        base_path = std::filesystem::temp_directory_path();
    }

    std::filesystem::path logs_dir = base_path / "lotus_logs";

    auto env_level = getLogLevelFromEnv();
    if (env_level >= spdlog::level::info) {
        // Taken once, so every logger of a run lands in the same folder.
        static const std::string run_timestamp = [] {
            auto now = std::chrono::system_clock::now();
            std::time_t now_c = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d_%H-%M-%S");
            return ss.str();
        }();
        logs_dir /= run_timestamp;
    } else {
        logs_dir /= "debug";
    }

    if (!std::filesystem::exists(logs_dir)) {
        std::filesystem::create_directories(logs_dir);

        try {
            std::filesystem::permissions(
                logs_dir,
                std::filesystem::perms::owner_all |
                    std::filesystem::perms::group_all);
        } catch (const std::filesystem::filesystem_error& perm_ex) {
            spdlog::warn(
                "Failed to set directory permissions for {}: {}",
                logs_dir.string(),
                perm_ex.what());
        }
    }

    return logs_dir;
}

void setLoggerPattern(std::shared_ptr<spdlog::logger>& logger)
{
    if (logger->level() < spdlog::level::info) {
        logger->set_pattern("%v");
    } else {
        logger->set_pattern("[%D %T] [%l]: %v");
    }
}
}  // namespace lotusim::logger