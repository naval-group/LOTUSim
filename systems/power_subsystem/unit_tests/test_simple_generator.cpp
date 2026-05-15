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
#include <memory>
#include <string>
#include <cmath>
#include <algorithm>

// test the logic directly without Gazebo or ROS2
// SDF and node construction are bypassed to test SimpleGenerator and Generator

#include "power_subsystem/generator.hpp"
#include "power_subsystem/power_provider.hpp"

namespace lotusim::gazebo
{
// ──────────────────────────────────────────────────────────────
// Mirrors SimpleGenerator but accepts plain constructor args 
// ──────────────────────────────────────────────────────────────
class TestGenerator : public Generator
{
public:
    TestGenerator(
        const std::string& name,
        float fuel_level_start,
        float fuel_capacity,
        float rated_output_w,
        float efficiency,
        float voltage_nominal,
        std::string fuel_type)
        : Generator(name, nullptr, fuel_level_start, 
            fuel_capacity, rated_output_w, efficiency, 
            voltage_nominal, fuel_type){}
    
    // ──────────────────────────────────────────────────────────────
    // Generator interface -> same logic as SimpleGenerator
    // ──────────────────────────────────────────────────────────────

    void receiveload(float currentA, float dt){
        if (isDepleted()){
            return;
        }

        const float powerW = currentA * m_voltage_nominal;
        const float energyDensity = getFuelEnergyDensity(m_fuel_type);
        const float fuel_consumed = (powerW * dt) / (m_efficiency * energyDensity);

        m_fuel_level = std::max(0.0f, m_fuel_level - fuel_consumed);
    }
};

// ──────────────────────────────────────────────────────────────
// Helper 
// ──────────────────────────────────────────────────────────────

static std::unique_ptr<TestGenerator> makeGenerator(
    const std::string& name = "test_generator",
    float fuel_level_start = 100.0f,
    float fuel_capacity = 150.0f,
    float rated_output_w = 5000.0f,
    float efficiency = 0.60f,
    float voltage_nominal = 48.0f,
    const std::string& fuel_type = "hydrogen")
{
    return std::make_unique<TestGenerator>(
        name, fuel_level_start, fuel_capacity, rated_output_w, efficiency, 
            voltage_nominal, fuel_type);
}

// ──────────────────────────────────────────────────────────────
// Fixture
// ──────────────────────────────────────────────────────────────
 
class SimpleGeneratorTest : public ::testing::Test
{
protected:
    std::unique_ptr<TestGenerator> gen = makeGenerator();
};

// ──────────────────────────────────────────────────────────────
// Construction tests
// ──────────────────────────────────────────────────────────────


} //namespace lotusim::gazebo