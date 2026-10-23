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

#include <atomic>
#include <chrono>
#include <thread>

#include <rclcpp/executor.hpp>

namespace lotusim::common {

/**
 * @brief A thread spinning `executor`, stopped by stop() or on destruction.
 *
 * executor.cancel() only stops a spin under way: one made before the thread
 * reached spin() was lost, the spin then ran for ever and join() hung the
 * server's shutdown. stop() therefore cancels again until spin() has
 * returned, so a lost cancel does not matter, whenever and from whichever
 * thread the stop comes. A spin that ends on its own (ROS shut down) needs
 * no cancel at all.
 *
 * The executor must outlive the SpinThread.
 */
class SpinThread {
public:
    explicit SpinThread(rclcpp::Executor& executor)
        : m_executor(executor), m_thread([this]() {
              m_executor.spin();
              m_done.store(true);
          })
    {
    }

    ~SpinThread()
    {
        stop();
    }

    SpinThread(const SpinThread&) = delete;
    SpinThread& operator=(const SpinThread&) = delete;

    void stop()
    {
        if (!m_thread.joinable()) {
            return;
        }
        while (!m_done.load()) {
            m_executor.cancel();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        m_thread.join();
    }

private:
    rclcpp::Executor& m_executor;
    std::atomic_bool m_done{false};
    std::thread m_thread;  // last, so the members it uses exist first
};

}  // namespace lotusim::common

#endif
