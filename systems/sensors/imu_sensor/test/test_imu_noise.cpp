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

#include <array>
#include <cmath>
#include <cstdlib>
#include <sdf/Root.hh>
#include <vector>

#include "imu_sensor/imu_sensor.hpp"

using lotusim::sensor::IMUSensor;

namespace {

// One value per quantity and axis, as in Gazebo's IMU tags, plus
// <orientation> in the same form. The SDFs fix <seed>, so the statistics
// below give the same result on every run.
constexpr char kSdf[] = R"(
<sdf version="1.9">
  <model name="vessel">
    <link name="base_link">
      <sensor name="imu" type="custom" gz:type="imu">
        <update_rate>1</update_rate>
        <seed>42</seed>
        <imu>
          <orientation>
            <z><noise type="gaussian"><mean>0</mean><stddev>0.3</stddev></noise></z>
          </orientation>
          <angular_velocity>
            <x><noise type="gaussian"><mean>0.1</mean><stddev>0.2</stddev></noise></x>
            <z><noise type="gaussian"><mean>0</mean><stddev>0.4</stddev></noise></z>
          </angular_velocity>
          <linear_acceleration>
            <y><noise type="gaussian"><mean>-0.5</mean><stddev>0.6</stddev></noise></y>
          </linear_acceleration>
        </imu>
      </sensor>
    </link>
  </model>
</sdf>)";

// A bias on one axis: bias_mean is its magnitude, its sign drawn on load.
constexpr char kBiasSdf[] = R"(
<sdf version="1.9">
  <model name="vessel">
    <link name="base_link">
      <sensor name="imu" type="custom" gz:type="imu">
        <update_rate>1</update_rate>
        <seed>42</seed>
        <imu>
          <linear_acceleration>
            <x><noise type="gaussian">
              <mean>0</mean><stddev>0.1</stddev>
              <bias_mean>0.5</bias_mean><bias_stddev>0</bias_stddev>
            </noise></x>
          </linear_acceleration>
        </imu>
      </sensor>
    </link>
  </model>
</sdf>)";

struct Stats {
    double mean{0.0};
    double stddev{0.0};
};

Stats stats(const std::vector<double>& values)
{
    Stats s;
    for (const double v : values) {
        s.mean += v / values.size();
    }
    for (const double v : values) {
        s.stddev += (v - s.mean) * (v - s.mean) / (values.size() - 1);
    }
    s.stddev = std::sqrt(s.stddev);
    return s;
}

/// The messages a vessel at rest's IMU publishes, one per simulated second.
std::vector<sensor_msgs::msg::Imu> measure(
    int count,
    const char* sdf_string = kSdf)
{
    auto node = rclcpp::Node::make_shared("test_imu_noise");
    std::vector<sensor_msgs::msg::Imu> received;
    auto sub = node->create_subscription<sensor_msgs::msg::Imu>(
        "vessel/imu/IMU",
        rclcpp::QoS(count),
        [&](const sensor_msgs::msg::Imu& msg) { received.push_back(msg); });

    sdf::Root root;
    EXPECT_TRUE(root.LoadSdfString(sdf_string).empty());
    const sdf::Sensor& sdf = *root.Model()->LinkByIndex(0)->SensorByIndex(0);

    IMUSensor sensor(
        std::make_shared<spdlog::logger>("test_imu_noise"),
        node,
        1,
        2,
        "vessel",
        "imu");
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

// At rest, without noise, every value is 0: what is measured is the noise.
TEST(IMUNoise, EachAxisCarriesItsOwnNoise)
{
    std::vector<double> roll, yaw, gyro_x, gyro_y, gyro_z, accel_x, accel_y;
    for (const auto& msg : measure(400)) {
        const gz::math::Quaterniond q(
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z);
        roll.push_back(q.Euler().X());
        yaw.push_back(q.Euler().Z());
        gyro_x.push_back(msg.angular_velocity.x);
        gyro_y.push_back(msg.angular_velocity.y);
        gyro_z.push_back(msg.angular_velocity.z);
        accel_x.push_back(msg.linear_acceleration.x);
        accel_y.push_back(msg.linear_acceleration.y);
    }
    auto expect =
        [](const std::vector<double>& values, double mean, double stddev) {
            const Stats s = stats(values);
            EXPECT_NEAR(s.mean, mean, 0.25 * stddev);
            EXPECT_NEAR(s.stddev, stddev, 0.2 * stddev);
        };
    expect(yaw, 0.0, 0.3);
    expect(gyro_x, 0.1, 0.2);
    expect(gyro_z, 0.0, 0.4);
    expect(accel_y, -0.5, 0.6);
    // Axes without <noise> stay exact.
    EXPECT_EQ(stats(roll).stddev, 0.0);
    EXPECT_EQ(stats(gyro_y).stddev, 0.0);
    EXPECT_EQ(stats(accel_x).stddev, 0.0);
}

TEST(IMUNoise, CovariancesGiveEachAxisVariance)
{
    const auto msg = measure(1).at(0);
    const std::array<double, 9> orientation{0, 0, 0, 0, 0, 0, 0, 0, 0.09};
    const std::array<double, 9>
        angular_velocity{0.04, 0, 0, 0, 0, 0, 0, 0, 0.16};
    const std::array<double, 9>
        linear_acceleration{0, 0, 0, 0, 0.36, 0, 0, 0, 0};
    for (int i = 0; i < 9; ++i) {
        EXPECT_NEAR(msg.orientation_covariance[i], orientation[i], 1e-12);
        EXPECT_NEAR(
            msg.angular_velocity_covariance[i],
            angular_velocity[i],
            1e-12);
        EXPECT_NEAR(
            msg.linear_acceleration_covariance[i],
            linear_acceleration[i],
            1e-12);
    }
}

// The bias shifts every measurement of the axis by the same amount, and is
// not part of the covariance.
TEST(IMUNoise, BiasShiftsTheAxis)
{
    const auto msgs = measure(200, kBiasSdf);
    std::vector<double> accel_x;
    for (const auto& msg : msgs) {
        accel_x.push_back(msg.linear_acceleration.x);
    }
    const Stats s = stats(accel_x);
    EXPECT_NEAR(std::abs(s.mean), 0.5, 0.05);
    EXPECT_NEAR(s.stddev, 0.1, 0.02);
    EXPECT_NEAR(msgs.at(0).linear_acceleration_covariance[0], 0.01, 1e-12);
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
