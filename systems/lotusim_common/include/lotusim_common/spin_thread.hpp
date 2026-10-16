/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_COMMON_SPIN_THREAD_HPP
#define LOTUSIM_COMMON_SPIN_THREAD_HPP

#include <chrono>
#include <thread>

#include <rclcpp/executor.hpp>

namespace lotusim::common {

/**
 * @brief A thread spinning `executor`, returned once it does spin.
 *
 * A system stops its thread with executor.cancel() then join(). cancel()
 * only stops a spin under way: one made before the thread reached spin() was
 * lost, the spin then ran for ever and join() hung the server's shutdown (a
 * server stopped right after it started never exited). Waiting here for the
 * spin to start closes that window.
 */
inline std::thread startSpinThread(
    rclcpp::Executor& executor,
    std::chrono::milliseconds timeout = std::chrono::seconds(10))
{
    std::thread thread([&executor]() { executor.spin(); });
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    // A spin that cannot start (ROS shut down) must not block the caller.
    while (!executor.is_spinning() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return thread;
}

}  // namespace lotusim::common

#endif
