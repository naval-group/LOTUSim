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
#include <vector>

#include "imu_sensor/imu_sensor.hpp"

using lotusim::sensor::IMUSensor;

namespace {

// noise_amplitude * N(0, noise_sigma): standard deviation 0.5.
constexpr double kStdDev = 0.5;
constexpr char kSdf[] = R"(
<sdf version="1.9">
  <model name="vessel">
    <link name="base_link">
      <sensor name="imu" type="custom" gz:type="imu">
        <update_rate>1</update_rate>
        <noise_sigma>0.25</noise_sigma>
        <noise_amplitude>2</noise_amplitude>
      </sensor>
    </link>
  </model>
</sdf>)";

double stdDev(const std::vector<double>& values)
{
    double mean = 0.0;
    for (const double v : values) {
        mean += v / values.size();
    }
    double variance = 0.0;
    for (const double v : values) {
        variance += (v - mean) * (v - mean) / (values.size() - 1);
    }
    return std::sqrt(variance);
}

/// The messages a vessel at rest's IMU publishes, one per simulated second.
std::vector<sensor_msgs::msg::Imu> measure(int count)
{
    auto node = rclcpp::Node::make_shared("test_imu_noise");
    std::vector<sensor_msgs::msg::Imu> received;
    auto sub = node->create_subscription<sensor_msgs::msg::Imu>(
        "vessel/imu/IMU", rclcpp::QoS(count),
        [&](const sensor_msgs::msg::Imu& msg) { received.push_back(msg); });

    sdf::Root root;
    EXPECT_TRUE(root.LoadSdfString(kSdf).empty());
    const sdf::Sensor& sdf = *root.Model()->LinkByIndex(0)->SensorByIndex(0);

    IMUSensor sensor(
        std::make_shared<spdlog::logger>("test_imu_noise"), node, 1, 2,
        "vessel", "imu");
    sensor.Load(sdf);

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

TEST(IMUNoise, MeasurementsCarryTheSdfNoise)
{
    std::vector<double> roll, gyro_z, accel_x;
    for (const auto& msg : measure(400)) {
        const gz::math::Quaterniond q(
            msg.orientation.w, msg.orientation.x, msg.orientation.y,
            msg.orientation.z);
        roll.push_back(q.Euler().X());
        gyro_z.push_back(msg.angular_velocity.z);
        accel_x.push_back(msg.linear_acceleration.x);
    }
    // At rest, without noise, every value is 0.
    EXPECT_NEAR(stdDev(roll), kStdDev, 0.1);
    EXPECT_NEAR(stdDev(gyro_z), kStdDev, 0.1);
    EXPECT_NEAR(stdDev(accel_x), kStdDev, 0.1);
}

TEST(IMUNoise, CovariancesGiveTheVariance)
{
    const auto msg = measure(1).at(0);
    for (const int i : {0, 4, 8}) {
        EXPECT_DOUBLE_EQ(msg.orientation_covariance[i], kStdDev * kStdDev);
        EXPECT_DOUBLE_EQ(msg.angular_velocity_covariance[i], kStdDev * kStdDev);
        EXPECT_DOUBLE_EQ(
            msg.linear_acceleration_covariance[i], kStdDev * kStdDev);
    }
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
