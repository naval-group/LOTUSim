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
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <future>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "lotusim_common/spin_thread.hpp"

using lotusim::common::SpinThread;
using namespace std::chrono_literals;

namespace {

// A stop that hangs would hang the whole test binary: run it aside and fail
// at once after a while instead (the hung thread cannot be joined).
void finishWithin(
    std::chrono::seconds limit,
    const char* what,
    std::function<void()> work)
{
    auto done = std::async(std::launch::async, std::move(work));
    if (done.wait_for(limit) != std::future_status::ready) {
        std::fprintf(stderr, "FAILED: %s still running after %llds\n", what,
            static_cast<long long>(limit.count()));
        std::_Exit(EXIT_FAILURE);
    }
    done.get();
}

class SpinThreadTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
    }
    void TearDown() override
    {
        rclcpp::shutdown();
    }
};

}  // namespace

// The stop comes before the thread has reached spin(): a single cancel() is
// lost then, so it is the case that hung the server.
TEST_F(SpinThreadTest, StopRightAfterStartReturns)
{
    finishWithin(30s, "500 starts and stops", []() {
        for (int i = 0; i < 500; ++i) {
            rclcpp::executors::MultiThreadedExecutor executor;
            SpinThread thread(executor);
        }
    });
}

TEST_F(SpinThreadTest, StopWhileSpinningReturns)
{
    auto node = rclcpp::Node::make_shared("spin_thread_test");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::atomic_int ticks{0};
    auto timer = node->create_wall_timer(1ms, [&ticks]() { ++ticks; });

    auto thread = std::make_unique<SpinThread>(executor);
    const auto deadline = std::chrono::steady_clock::now() + 10s;
    while (ticks.load() < 5 && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(1ms);
    }
    EXPECT_GE(ticks.load(), 5) << "the executor never spun";

    finishWithin(10s, "a stop while spinning", [&thread]() { thread.reset(); });
}

// Once ROS is shut down, spin() returns on its own: neither the start nor
// the stop may wait.
TEST_F(SpinThreadTest, StartAndStopAfterRosShutdownDoNotWait)
{
    rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::shutdown();
    const auto start = std::chrono::steady_clock::now();
    finishWithin(10s, "a start and stop after shutdown", [&executor]() {
        SpinThread thread(executor);
        thread.stop();
    });
    EXPECT_LT(std::chrono::steady_clock::now() - start, 1s);
}

TEST_F(SpinThreadTest, StopTwiceIsHarmless)
{
    rclcpp::executors::MultiThreadedExecutor executor;
    SpinThread thread(executor);
    finishWithin(10s, "two stops", [&thread]() {
        thread.stop();
        thread.stop();
    });
}
