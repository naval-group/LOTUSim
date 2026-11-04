/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <spdlog/sinks/ostream_sink.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "lotusim_common/logger.hpp"

using namespace lotusim;

namespace {

std::shared_ptr<spdlog::logger> testLogger()
{
    static auto logger = logger::createConsoleAndFileLogger(
        "test_log_callback",
        "test_log_callback.txt");
    return logger;
}

// Sets an environment variable (unsets it for nullptr) for one scope, then
// puts back what was there.
class ScopedEnv
{
public:
    ScopedEnv(const char* name, const char* value)
        : m_name(name)
    {
        if (const char* previous = std::getenv(name)) {
            m_previous = previous;
        }
        set(value);
    }
    ~ScopedEnv()
    {
        set(m_previous ? m_previous->c_str() : nullptr);
    }
    ScopedEnv(const ScopedEnv&) = delete;
    ScopedEnv& operator=(const ScopedEnv&) = delete;

private:
    void set(const char* value)
    {
        if (value) {
            setenv(m_name.c_str(), value, 1);
        } else {
            unsetenv(m_name.c_str());
        }
    }

    std::string m_name;
    std::optional<std::string> m_previous;
};

// A fresh directory for one test, removed with everything in it at the end.
class ScopedTempDir
{
public:
    explicit ScopedTempDir(const std::string& name)
        : m_path(
              std::filesystem::temp_directory_path() /
              ("lotusim_test_logger_" + std::to_string(getpid()) + "_" + name))
    {
        std::filesystem::remove_all(m_path);
        std::filesystem::create_directories(m_path);
    }
    ~ScopedTempDir()
    {
        std::error_code ignored;
        std::filesystem::remove_all(m_path, ignored);
    }
    ScopedTempDir(const ScopedTempDir&) = delete;
    ScopedTempDir& operator=(const ScopedTempDir&) = delete;

    const std::filesystem::path& path() const
    {
        return m_path;
    }

private:
    std::filesystem::path m_path;
};

std::string readFile(const std::filesystem::path& path)
{
    std::ifstream file(path);
    std::stringstream content;
    content << file.rdbuf();
    return content.str();
}

}  // namespace

TEST(LogCallback, ForwardsInfoAndError)
{
    std::vector<std::string> received;
    logger::setLogCallback(
        [&](const std::string&,
            spdlog::level::level_enum,
            const std::string& m) { received.push_back(m); });
    testLogger()->info("hello");
    testLogger()->warn("not forwarded");
    testLogger()->error("oops");
    logger::setLogCallback(nullptr);
    EXPECT_EQ(received, (std::vector<std::string>{"hello", "oops"}));
}

TEST(LogCallback, ClearedCallbackIsNotCalled)
{
    // What an owner does before it is destroyed (MultiAgentSystem::shutdown).
    int calls = 0;
    logger::setLogCallback([&](const std::string&,
                               spdlog::level::level_enum,
                               const std::string&) { ++calls; });
    logger::setLogCallback(nullptr);
    testLogger()->info("after clearing");
    EXPECT_EQ(calls, 0);
}

TEST(LogCallback, SetWhileOtherThreadsLog)
{
    std::atomic<bool> stop{false};
    std::atomic<int> calls{0};
    std::vector<std::thread> loggers;
    for (int i = 0; i < 4; ++i) {
        loggers.emplace_back([&] {
            while (!stop) {
                testLogger()->info("busy");
            }
        });
    }
    for (int i = 0; i < 2000; ++i) {
        logger::setLogCallback([&](const std::string&,
                                   spdlog::level::level_enum,
                                   const std::string&) { ++calls; });
        logger::setLogCallback(nullptr);
    }
    stop = true;
    for (auto& t : loggers) {
        t.join();
    }
    SUCCEED();
}

TEST(LogCallback, ClearingWaitsForARunningCallback)
{
    // Once setLogCallback(nullptr) returns, no call into the old callback is
    // still running, so its owner can be destroyed.
    std::atomic<bool> entered{false};
    std::atomic<bool> release{false};
    std::atomic<bool> running{false};
    logger::setLogCallback(
        [&](const std::string&, spdlog::level::level_enum, const std::string&) {
            running = true;
            entered = true;
            while (!release) {
                std::this_thread::yield();
            }
            running = false;
        });
    std::thread logging([] { testLogger()->info("slow"); });
    while (!entered) {
        std::this_thread::yield();
    }

    std::atomic<bool> cleared{false};
    std::atomic<bool> running_after_clear{true};
    std::thread clearing([&] {
        logger::setLogCallback(nullptr);
        running_after_clear = running.load();
        cleared = true;
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_FALSE(cleared);

    release = true;
    clearing.join();
    logging.join();
    EXPECT_TRUE(cleared);
    EXPECT_FALSE(running_after_clear);
}

TEST(LogCallback, ReceivesLoggerNameLevelAndFormattedMessage)
{
    using Call =
        std::tuple<std::string, spdlog::level::level_enum, std::string>;
    std::vector<Call> received;
    logger::setLogCallback([&](const std::string& name,
                               spdlog::level::level_enum level,
                               const std::string& m) {
        received.emplace_back(name, level, m);
    });
    testLogger()->info("x={}", 3);
    testLogger()->error("{} failed", "step");
    logger::setLogCallback(nullptr);
    EXPECT_EQ(
        received,
        (std::vector<Call>{
            {"test_log_callback", spdlog::level::info, "x=3"},
            {"test_log_callback", spdlog::level::err, "step failed"}}));
}

TEST(LogCallback, LoggerLevelFiltersBeforeTheCallback)
{
    std::vector<std::string> received;
    logger::setLogCallback(
        [&](const std::string&,
            spdlog::level::level_enum,
            const std::string& m) { received.push_back(m); });
    const auto previous_level = testLogger()->level();
    testLogger()->set_level(spdlog::level::err);
    testLogger()->info("below the logger level");
    testLogger()->error("kept");
    testLogger()->set_level(previous_level);
    logger::setLogCallback(nullptr);
    EXPECT_EQ(received, (std::vector<std::string>{"kept"}));
}

TEST(LogLevel, ReadFromLotusimLogLevel)
{
    const std::vector<std::pair<const char*, spdlog::level::level_enum>>
        cases = {
            {"TRACE", spdlog::level::trace},
            {"DEBUG", spdlog::level::debug},
            {"INFO", spdlog::level::info},
            {"WARN", spdlog::level::warn},
            {"WARNING", spdlog::level::warn},
            {"ERROR", spdlog::level::err},
            {"CRITICAL", spdlog::level::critical},
            {"OFF", spdlog::level::off},
            {"debug", spdlog::level::debug},
            {"Error", spdlog::level::err},
            {"verbose", logger::DEFAULT_LOG_LEVEL},
            {"", logger::DEFAULT_LOG_LEVEL},
        };
    for (const auto& [value, expected] : cases) {
        ScopedEnv level("LOTUSIM_LOG_LEVEL", value);
        EXPECT_EQ(logger::getLogLevelFromEnv(), expected)
            << "LOTUSIM_LOG_LEVEL=" << value;
    }
}

TEST(LogLevel, UnsetUsesTheDefault)
{
    ScopedEnv level("LOTUSIM_LOG_LEVEL", nullptr);
    EXPECT_EQ(logger::getLogLevelFromEnv(), logger::DEFAULT_LOG_LEVEL);
}

TEST(LoggerFactory, SameNameReturnsTheExistingLogger)
{
    // Logger names are process-wide: asking for a taken name returns the
    // logger already registered under it, whichever factory made it. A basic
    // file logger has no callback sink, so its messages are never forwarded.
    auto basic = logger::createBasicFileLogger(
        "test_same_name",
        "test_same_name_basic.txt");
    auto console = logger::createConsoleAndFileLogger(
        "test_same_name",
        "test_same_name_console.txt");
    EXPECT_EQ(console, basic);

    std::vector<std::string> received;
    logger::setLogCallback(
        [&](const std::string&,
            spdlog::level::level_enum,
            const std::string& m) { received.push_back(m); });
    console->info("not forwarded");
    logger::setLogCallback(nullptr);
    EXPECT_TRUE(received.empty());
}

TEST(LoggerFactory, ConcurrentCreationOfOneName)
{
    // Several rounds widen the window for a check-then-register race.
    constexpr int kRounds = 20;
    constexpr int kThreads = 8;
    for (int round = 0; round < kRounds; ++round) {
        const std::string name = "test_concurrent_" + std::to_string(round);
        std::vector<std::shared_ptr<spdlog::logger>> created(kThreads);
        std::atomic<int> failures{0};
        std::atomic<bool> go{false};
        std::vector<std::thread> threads;
        for (int i = 0; i < kThreads; ++i) {
            threads.emplace_back([&, i] {
                while (!go) {
                    std::this_thread::yield();
                }
                try {
                    created[i] = logger::createConsoleAndFileLogger(
                        name,
                        name + ".txt");
                } catch (const std::exception&) {
                    ++failures;
                }
            });
        }
        go = true;
        for (auto& t : threads) {
            t.join();
        }
        ASSERT_EQ(failures, 0) << "round " << round;
        ASSERT_NE(created[0], nullptr);
        for (const auto& logger : created) {
            ASSERT_EQ(logger, created[0]) << "round " << round;
        }
    }
}

TEST(LogFolder, SameFolderAcrossSeconds)
{
    // Loggers created at different times in one run share one folder.
    const auto first = logger::createOrGetLogFolderPath();
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));
    EXPECT_EQ(logger::createOrGetLogFolderPath(), first);
}

TEST(LogFolder, UnderLotusimPath)
{
    ScopedTempDir dir("under_lotusim_path");
    ScopedEnv path("LOTUSIM_PATH", dir.path().c_str());
    ScopedEnv level("LOTUSIM_LOG_LEVEL", "info");
    const auto folder = logger::createOrGetLogFolderPath();
    EXPECT_EQ(folder.parent_path(), dir.path() / "lotus_logs");
    EXPECT_NE(folder.filename(), "debug");
    EXPECT_TRUE(std::filesystem::is_directory(folder));
}

TEST(LogFolder, DebugFolderBelowInfo)
{
    ScopedTempDir dir("debug_folder");
    ScopedEnv path("LOTUSIM_PATH", dir.path().c_str());
    ScopedEnv level("LOTUSIM_LOG_LEVEL", "debug");
    const auto folder = logger::createOrGetLogFolderPath();
    EXPECT_EQ(folder, dir.path() / "lotus_logs" / "debug");
    EXPECT_TRUE(std::filesystem::is_directory(folder));
}

TEST(LogFolder, TempDirectoryWithoutLotusimPath)
{
    ScopedTempDir dir("temp_fallback");
    ScopedEnv tmpdir("TMPDIR", dir.path().c_str());
    ScopedEnv path("LOTUSIM_PATH", nullptr);
    ScopedEnv level("LOTUSIM_LOG_LEVEL", "debug");
    EXPECT_EQ(
        logger::createOrGetLogFolderPath(),
        dir.path() / "lotus_logs" / "debug");
}

TEST(LoggerFactory, ThrowsWhenTheLogFolderCannotBeCreated)
{
    // A directory cannot be made under a regular file, even as root.
    ScopedEnv path("LOTUSIM_PATH", "/dev/null/lotusim");
    EXPECT_THROW(
        logger::createConsoleAndFileLogger(
            "test_unwritable",
            "test_unwritable.txt"),
        std::runtime_error);
    EXPECT_THROW(
        logger::createBasicFileLogger(
            "test_unwritable_basic",
            "test_unwritable_basic.txt"),
        std::runtime_error);
    EXPECT_EQ(logger::getLogger("test_unwritable"), nullptr);
}

TEST(LoggerFactory, FileSinkWritesAndTruncatesOnCreation)
{
    ScopedTempDir dir("file_sink");
    ScopedEnv path("LOTUSIM_PATH", dir.path().c_str());
    ScopedEnv level("LOTUSIM_LOG_LEVEL", "info");
    const auto file =
        logger::createOrGetLogFolderPath() / "test_file_sink.txt";

    auto first =
        logger::createConsoleAndFileLogger("test_file_sink", file.filename());
    first->info("first run");
    first->flush();
    EXPECT_NE(readFile(file).find("first run"), std::string::npos);

    // A new logger on the same file starts it over.
    spdlog::drop("test_file_sink");
    first.reset();
    auto second =
        logger::createConsoleAndFileLogger("test_file_sink", file.filename());
    second->info("second run");
    second->flush();
    const auto content = readFile(file);
    EXPECT_EQ(content.find("first run"), std::string::npos);
    EXPECT_NE(content.find("second run"), std::string::npos);
    spdlog::drop("test_file_sink");
}

TEST(LoggerPattern, TimestampAndLevelFromInfo)
{
    std::ostringstream out;
    auto logger = std::make_shared<spdlog::logger>(
        "test_pattern_info",
        std::make_shared<spdlog::sinks::ostream_sink_mt>(out));
    logger->set_level(spdlog::level::info);
    logger::setLoggerPattern(logger);
    logger->info("hello");
    EXPECT_TRUE(std::regex_match(
        out.str(),
        std::regex(R"(\[\d\d/\d\d/\d\d \d\d:\d\d:\d\d\] \[info\]: hello\n)")))
        << out.str();
}

TEST(LoggerPattern, MessageOnlyBelowInfo)
{
    std::ostringstream out;
    auto logger = std::make_shared<spdlog::logger>(
        "test_pattern_debug",
        std::make_shared<spdlog::sinks::ostream_sink_mt>(out));
    logger->set_level(spdlog::level::debug);
    logger::setLoggerPattern(logger);
    logger->debug("hello");
    EXPECT_EQ(out.str(), "hello\n");
}

// gz-sim's dependency chain can crash during static teardown (see
// test_frame_change), so exit without running it.
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    const int result = RUN_ALL_TESTS();
    std::_Exit(result);
}