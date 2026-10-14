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
#include <cstdlib>
#include <string>
#include <thread>
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

}  // namespace

TEST(LogCallback, ForwardsInfoAndError)
{
    std::vector<std::string> received;
    logger::setLogCallback(
        [&](const std::string&, spdlog::level::level_enum, const std::string& m) {
            received.push_back(m);
        });
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
    logger::setLogCallback(
        [&](const std::string&, spdlog::level::level_enum, const std::string&) {
            ++calls;
        });
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
        logger::setLogCallback(
            [&](const std::string&, spdlog::level::level_enum, const std::string&) {
                ++calls;
            });
        logger::setLogCallback(nullptr);
    }
    stop = true;
    for (auto& t : loggers) {
        t.join();
    }
    SUCCEED();
}

// gz-sim's dependency chain can crash during static teardown (see
// test_frame_change), so exit without running it.
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    const int result = RUN_ALL_TESTS();
    std::_Exit(result);
}
