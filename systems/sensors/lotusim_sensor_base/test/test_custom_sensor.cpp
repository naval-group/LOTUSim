/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "lotusim_sensor_base/custom_sensor.hpp"

#include <gtest/gtest.h>
#include <spdlog/sinks/null_sink.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <sdf/Root.hh>
#include <string>

using namespace std::chrono_literals;

namespace {

/**
 * @brief The smallest concrete CustomSensor, loaded from an SDF sensor with
 * the given <update_rate>; exposes EnableMeasurement().
 */
class RateProbe : public lotusim::sensor::CustomSensor {
public:
    explicit RateProbe(const std::string& update_rate)
        : CustomSensor(
              std::make_shared<spdlog::logger>(
                  "test_custom_sensor",
                  std::make_shared<spdlog::sinks::null_sink_mt>()),
              node(),
              0,
              0,
              "vessel",
              "probe" + std::to_string(++s_count))
    {
        sdf::Root root;
        EXPECT_TRUE(root.LoadSdfString(
                            "<sdf version='1.9'><model name='vessel'><link name='base_link'>"
                            "<sensor name='probe' type='custom' gz:type='probe'>"
                            "<update_rate>" +
                            update_rate +
                            "</update_rate></sensor></link></model></sdf>")
                        .empty());
        Load(*root.Model()->LinkByIndex(0)->SensorByIndex(0));
    }

    /// Counts the measurements a sensor stepped with the simulation takes.
    int measurementsOver(
        std::chrono::steady_clock::duration duration,
        std::chrono::steady_clock::duration step)
    {
        int count = 0;
        for (auto now = step; now <= duration; now += step) {
            if (EnableMeasurement(now)) {
                m_last_measurement_time = now;
                ++count;
            }
        }
        return count;
    }

    void turnOff()
    {
        m_is_on = false;
    }

    bool UpdateSensor(
        const gz::sim::UpdateInfo&,
        const gz::sim::EntityComponentManager&) override
    {
        return true;
    }

protected:
    bool CustomSensorLoad(const sdf::Sensor&) override
    {
        return true;
    }

private:
    // Load() serves the sensor's change_state on a ROS node.
    static rclcpp::Node::SharedPtr node()
    {
        static auto node = rclcpp::Node::make_shared("test_custom_sensor");
        return node;
    }
    static inline int s_count = 0;
};

}  // namespace

TEST(CustomSensorRate, TenHertzOnAMillisecondStep)
{
    RateProbe sensor("10");
    EXPECT_EQ(sensor.measurementsOver(10s, 1ms), 100);
}

TEST(CustomSensorRate, TenHertzOnATenthOfASecondStep)
{
    RateProbe sensor("10");
    EXPECT_EQ(sensor.measurementsOver(10s, 100ms), 100);
}

TEST(CustomSensorRate, FasterThanTheStepMeasuresEveryStep)
{
    // LOTUSim worlds step at 0.2 s: a 1000 Hz IMU measures 5 times a second.
    RateProbe sensor("1000.0");
    EXPECT_EQ(sensor.measurementsOver(10s, 200ms), 50);
}

TEST(CustomSensorRate, OneHertz)
{
    RateProbe sensor("1");
    EXPECT_EQ(sensor.measurementsOver(10s, 200ms), 10);
}

TEST(CustomSensorRate, FractionalRate)
{
    RateProbe sensor("0.5");
    EXPECT_EQ(sensor.measurementsOver(10s, 200ms), 5);
}

TEST(CustomSensorRate, ZeroRateMeasuresEveryStep)
{
    RateProbe sensor("0");
    EXPECT_EQ(sensor.measurementsOver(10s, 200ms), 50);
}

TEST(CustomSensorRate, OffSensorNeverMeasures)
{
    RateProbe sensor("10");
    sensor.turnOff();
    EXPECT_EQ(sensor.measurementsOver(10s, 200ms), 0);
}

// gz-sim's dependency chain can crash during static teardown (see
// physics_engine_interface's tests), so exit without running it.
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    const int result = RUN_ALL_TESTS();
    std::_Exit(result);
}
