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

#include <cmath>
#include <cstdlib>
#include <sdf/Root.hh>
#include <string>
#include <vector>

#include "subsea_pressure_sensor/subsea_pressure_sensor.hh"

using lotusim::sensor::SubseaPressureSensor;

namespace {

/// The pressure's noise, in the form of Gazebo's air pressure sensor. A
/// fixed <seed>, so the statistics give the same result on every run.
constexpr char kSdf[] = R"(
<sdf version="1.9">
  <model name="vessel">
    <link name="base_link">
      <sensor name="pressure" type="custom" gz:type="subsea_pressure">
        <update_rate>1</update_rate>
        <seed>42</seed>
        <pressure>
          <noise type="gaussian"><mean>0</mean><stddev>0.5</stddev></noise>
        </pressure>
      </sensor>
    </link>
  </model>
</sdf>)";

/// A sensor without noise, with the given tags.
std::string plainSdf(const std::string& tags)
{
    return "<sdf version='1.9'><model name='vessel'><link name='base_link'>"
           "<sensor name='pressure' type='custom' gz:type='subsea_pressure'>"
           "<update_rate>1</update_rate>" +
           tags + "</sensor></link></model></sdf>";
}

/// The messages of a sensor `depth` metres deep, one per simulated second.
std::vector<lotusim_sensor_msgs::msg::PressureDepth> measure(
    int count,
    const std::string& sdf_string = kSdf,
    double depth = 10.0)
{
    auto node = rclcpp::Node::make_shared("test_pressure_noise");
    std::vector<lotusim_sensor_msgs::msg::PressureDepth> received;
    auto sub =
        node->create_subscription<lotusim_sensor_msgs::msg::PressureDepth>(
            "vessel/pressure/pressure_sensor",
            rclcpp::QoS(count),
            [&](const lotusim_sensor_msgs::msg::PressureDepth& msg) {
                received.push_back(msg);
            });

    sdf::Root root;
    EXPECT_TRUE(root.LoadSdfString(sdf_string).empty());
    const sdf::Sensor& sdf = *root.Model()->LinkByIndex(0)->SensorByIndex(0);

    SubseaPressureSensor sensor(
        std::make_shared<spdlog::logger>("test_pressure_noise"),
        node,
        1,
        2,
        "vessel",
        "pressure");
    sensor.Load(sdf);
    sensor.Position({0.0, 0.0, -depth});

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    gz::sim::EntityComponentManager ecm;
    gz::sim::UpdateInfo info;
    for (int i = 1; i <= count; ++i) {
        info.simTime = std::chrono::seconds(i);
        EXPECT_TRUE(sensor.UpdateSensor(info, ecm));
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (received.size() < static_cast<size_t>(i) &&
               std::chrono::steady_clock::now() < deadline) {
            executor.spin_some(std::chrono::milliseconds(10));
        }
    }
    EXPECT_EQ(received.size(), static_cast<size_t>(count));
    return received;
}

}  // namespace

// The noise of <pressure><noise> is on the pressure, and its stddev in the
// message.
TEST(PressureNoise, NoiseFromThePressureElement)
{
    const auto msgs = measure(400);
    double mean = 0.0;
    for (const auto& msg : msgs) {
        mean += msg.pressure / msgs.size();
    }
    double variance = 0.0;
    for (const auto& msg : msgs) {
        variance +=
            (msg.pressure - mean) * (msg.pressure - mean) / (msgs.size() - 1);
    }
    // 101.325 kPa at the surface, 9.80638 kPa per metre.
    EXPECT_NEAR(mean, 101.325 + 10 * 9.80638, 0.25 * 0.5);
    EXPECT_NEAR(std::sqrt(variance), 0.5, 0.2 * 0.5);
    EXPECT_DOUBLE_EQ(msgs.at(0).stddev, 0.5);
}

// 400 m deep is about 4024 kPa: the sensor reads at most <saturation>,
// 3000 kPa by default, and still reads <m_saturation>, its former name.
TEST(PressureSaturation, CapsThePressure)
{
    EXPECT_DOUBLE_EQ(measure(1, plainSdf(""), 400.0).at(0).pressure, 3000.0);
    EXPECT_DOUBLE_EQ(
        measure(1, plainSdf("<saturation>2000</saturation>"), 400.0)
            .at(0)
            .pressure,
        2000.0);
    EXPECT_DOUBLE_EQ(
        measure(1, plainSdf("<m_saturation>2500</m_saturation>"), 400.0)
            .at(0)
            .pressure,
        2500.0);
    // Below it, the pressure is the depth's.
    EXPECT_DOUBLE_EQ(
        measure(1, plainSdf(""), 10.0).at(0).pressure,
        101.325 + 10 * 9.80638);
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
