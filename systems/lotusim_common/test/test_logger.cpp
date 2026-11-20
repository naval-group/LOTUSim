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
#include <spdlog/sinks/ostream_sink.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "lotusim_common/logger.hpp"

using namespace lotusim;

namespace {

using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

std::shared_ptr<spdlog::logger> testLogger()
{
    static auto logger = logger::createConsoleAndFileLogger(
        "test_diagnostics",
        "test_diagnostics.txt");
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

// Enables diagnostics on a private node, subscribes to /diagnostics and
// collects what arrives. Disables diagnostics again on destruction, like
// MultiAgentSystem::shutdown does.
class DiagnosticsCapture
{
public:
    explicit DiagnosticsCapture(
    const std::string& hardware_id = "test_world",
    std::shared_ptr<spdlog::logger> log = testLogger())
    : m_log(std::move(log)),
      m_node(rclcpp::Node::make_shared(
            "logger_test_" + std::to_string(getpid()) + "_" +
            std::to_string(s_counter++)))
    {
        m_sub = m_node->create_subscription<DiagnosticArray>(
            "/diagnostics",
            10,
            [this](DiagnosticArray::SharedPtr msg) {
                if (!msg->status.empty() &&
                    msg->status[0].message == kWarmup) {
                    m_warm = true;
                    return;
                }
                m_received.push_back(*msg);
            });
        logger::enableDiagnostics(m_log, m_node, hardware_id);

        // The first messages can be lost until publisher and subscriber have
        // found each other, so log until one comes through
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (!m_warm && std::chrono::steady_clock::now() < deadline) {
            m_log->info(kWarmup);
            rclcpp::spin_some(m_node);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
    ~DiagnosticsCapture()
    {
        logger::disableDiagnostics(m_log);
    }
    DiagnosticsCapture(const DiagnosticsCapture&) = delete;
    DiagnosticsCapture& operator=(const DiagnosticsCapture&) = delete;

    bool ready() const
    {
        return m_warm;
    }

    // Spin until `count` arrays arrived or the timeout passes
    bool waitFor(
        size_t count,
        std::chrono::milliseconds timeout = std::chrono::seconds(2))
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (m_received.size() < count &&
               std::chrono::steady_clock::now() < deadline) {
            rclcpp::spin_some(m_node);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return m_received.size() >= count;
    }

    // Spin for a fixed time, for checking that nothing arrives.
    void spinFor(std::chrono::milliseconds duration)
    {
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < deadline) {
            rclcpp::spin_some(m_node);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    const std::vector<DiagnosticArray>& received() const
    {
        return m_received;
    }

private:
    static constexpr const char* kWarmup = "__diagnostics_warmup__";
    static inline std::atomic<int> s_counter{0};

    std::shared_ptr<spdlog::logger> m_log;  
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Subscription<DiagnosticArray>::SharedPtr m_sub;
    std::vector<DiagnosticArray> m_received;
    bool m_warm = false;
};

}  // namespace

TEST(Diagnostics, PublishesLevelNameMessageAndHardwareId)
{
    DiagnosticsCapture capture("test_world");
    ASSERT_TRUE(capture.ready());

    testLogger()->error("boom");
    ASSERT_TRUE(capture.waitFor(1));

    const auto& array = capture.received()[0];
    ASSERT_EQ(array.status.size(), 1u);
    EXPECT_EQ(array.status[0].name, "test_diagnostics");
    EXPECT_EQ(array.status[0].message, "boom");
    EXPECT_EQ(array.status[0].hardware_id, "test_world");
    EXPECT_EQ(array.status[0].level, DiagnosticStatus::ERROR);
    EXPECT_GT(array.header.stamp.sec, 0);
}

TEST(Diagnostics, FormattedMessageIsPublished)
{
    DiagnosticsCapture capture;
    ASSERT_TRUE(capture.ready());

    testLogger()->info("x={}", 3);
    testLogger()->error("{} failed", "step");
    ASSERT_TRUE(capture.waitFor(2));

    EXPECT_EQ(capture.received()[0].status[0].message, "x=3");
    EXPECT_EQ(capture.received()[1].status[0].message, "step failed");
}

TEST(Diagnostics, MapsSpdlogLevelsToDiagnosticLevels)
{
    DiagnosticsCapture capture;
    ASSERT_TRUE(capture.ready());

    testLogger()->info("info");
    testLogger()->warn("warn");
    testLogger()->error("error");
    testLogger()->critical("critical");
    ASSERT_TRUE(capture.waitFor(4));

    EXPECT_EQ(capture.received()[0].status[0].level, DiagnosticStatus::OK);
    EXPECT_EQ(capture.received()[1].status[0].level, DiagnosticStatus::WARN);
    EXPECT_EQ(capture.received()[2].status[0].level, DiagnosticStatus::ERROR);
    EXPECT_EQ(capture.received()[3].status[0].level, DiagnosticStatus::ERROR);
}

TEST(Diagnostics, LoggerLevelFiltersBeforeTheSink)
{
    DiagnosticsCapture capture;
    ASSERT_TRUE(capture.ready());

    const auto previous_level = testLogger()->level();
    testLogger()->set_level(spdlog::level::err);
    testLogger()->info("below the logger level");
    testLogger()->error("kept");
    testLogger()->set_level(previous_level);

    ASSERT_TRUE(capture.waitFor(1));
    capture.spinFor(std::chrono::milliseconds(200));
    ASSERT_EQ(capture.received().size(), 1u);
    EXPECT_EQ(capture.received()[0].status[0].message, "kept");
}

TEST(Diagnostics, NothingIsPublishedAfterDisable)
{
    DiagnosticsCapture capture;
    ASSERT_TRUE(capture.ready());

    logger::disableDiagnostics(testLogger());
    testLogger()->info("after disabling");
    testLogger()->error("also after disabling");
    capture.spinFor(std::chrono::milliseconds(300));
    EXPECT_TRUE(capture.received().empty());
}

TEST(Diagnostics, LoggingWithoutEnableDoesNothing)
{
    logger::disableDiagnostics(testLogger());
    EXPECT_NO_THROW(testLogger()->info("diagnostics were never enabled"));
}

TEST(Diagnostics, EnableAndDisableWhileOtherThreadsLog)
{
    auto node = rclcpp::Node::make_shared(
        "logger_test_concurrent_" + std::to_string(getpid()));
    std::atomic<bool> stop{false};
    std::vector<std::thread> loggers;
    for (int i = 0; i < 4; ++i) {
        loggers.emplace_back([&] {
            while (!stop) {
                testLogger()->info("busy");
            }
        });
    }
    for (int i = 0; i < 100; ++i) {
        logger::enableDiagnostics(testLogger(), node, "busy_world");
        logger::disableDiagnostics(testLogger());
    }
    stop = true;
    for (auto& t : loggers) {
        t.join();
    }
    SUCCEED();
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
    auto basic = logger::createBasicFileLogger("test_same_name", "test_same_name_basic.txt");
    auto console = logger::createConsoleAndFileLogger("test_same_name", "test_same_name_console.txt");
    EXPECT_EQ(console, basic);

    // A basic file logger has no diagnostics sink, so enabling it fails
    auto node = rclcpp::Node::make_shared("logger_test_same_name");
    EXPECT_FALSE(logger::enableDiagnostics(console, node, "w"));
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

TEST(Diagnostics, DisablingOneLoggerLeavesAnotherPublishing)
{
    auto a = logger::createConsoleAndFileLogger("test_diag_a", "test_diag_a.txt");
    auto b = logger::createConsoleAndFileLogger("test_diag_b", "test_diag_b.txt");

    DiagnosticsCapture capture_a("world_a", a);
    DiagnosticsCapture capture_b("world_b", b);
    ASSERT_TRUE(capture_a.ready());
    ASSERT_TRUE(capture_b.ready());

    logger::disableDiagnostics(a);
    a->error("from a");
    b->error("from b");

    ASSERT_TRUE(capture_b.waitFor(1));
    EXPECT_EQ(capture_b.received()[0].status[0].message, "from b");
}

TEST(Diagnostics, EnableFailsWithoutSinkOrNode)
{
    EXPECT_FALSE(logger::enableDiagnostics(nullptr, nullptr, "w"));
}

// gz-sim's dependency chain can crash during static teardown (see
// test_frame_change), so exit without running it.
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    std::_Exit(result);
}