#include "logging_system/logger.hpp"

namespace lotusim::logger {

auto createConsoleAndFileLogger(
    const std::string &logger_name,
    const std::string &file_name) -> std::shared_ptr<spdlog::logger>
{
    try {
        auto console_sink =
            std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

        std::string file_path = createOrGetLogFolderPath();

        file_path = file_path + "/" + file_name;
        auto file_sink =
            std::make_shared<spdlog::sinks::basic_file_sink_mt>(file_path);

        std::filesystem::permissions(
            file_path,
            std::filesystem::perms::owner_all |
                std::filesystem::perms::group_all);

        // creating logger manually
        auto logger = std::make_shared<spdlog::logger>(
            logger_name,
            spdlog::sinks_init_list{file_sink, console_sink});

        logger->set_level(LOG_LEVEL);

        spdlog::flush_every(std::chrono::seconds(3));
        spdlog::register_logger(logger);

        if (LOG_LEVEL < spdlog::level::info) {
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
        logger->set_level(LOG_LEVEL);
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

    const std::string logs_dir = "lotus_logs";
    if (!std::filesystem::exists(logs_dir)) {
        std::filesystem::create_directory(logs_dir);
        std::filesystem::permissions(
            logs_dir,
            std::filesystem::perms::owner_all |
                std::filesystem::perms::group_all);
    }

    auto log_folder_path =
        std::filesystem::current_path().generic_string() + "/" + logs_dir + "/";

    return log_folder_path;
}
}  // namespace lotusim::logger
