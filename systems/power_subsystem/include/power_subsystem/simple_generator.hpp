/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#pragma once

#include "power_subsystem/generator.hpp"
#include "power_subsystem/simple_generator.hpp"
#include <gz/common/Console.hh>
#include "lotusim_common/common.hpp"
#include "lotusim_common/logger.hpp"

namespace lotusim::gazebo{
/**
* @brief minimal battery for integration testing
* 
* Simple linear fuel consumption model
*
*   fuel_consumed = (currentA * voltage_nominal * dt) / (efficiency * 3600)
*   m_fuel_level -= fuel_consumed
*
* Replace receiveLoad() with FMU calls once the FMU wrapper is available.
*
* SDF format:
*   <link name="test_generator">
*        <lotusim_power>
*           <type>simple_generator</type>
*           <fuel_type>diesel</fuel_type>
*           <fuel_capacity>500</fuel_capacity>
*           <fuel_level_start>300</fuel_level_start>
*           <rated_output_w>5000</rated_output_w>
*           <efficiency>0.35</efficiency>
*           <voltage_nominal>48.0</voltage_nominal>
*       </lotusim_power>
*   </link>
*/

class SimpleGenerator : public Generator
{
public:
    /**
    * @param name   generator name from SDF
    * @param _sdf   to read fuel_capacity, fuel_level_start,
    *               rated_output_w, efficiency, voltage_nominal
    * @param node   from PowerManager
    */    
    SimpleGenerator(
        std::string name,
        const sdf::ElementPtr& _sdf,
        rclcpp::Node::SharedPtr node
    ) : Generator(
        std::move(name),
        std::move(node),
        _sdf->Get<float>("fuel_level_start", 500.0f).first,
        _sdf->Get<float>("fuel_capacity", 500.0f).first,
        _sdf->Get<float>("rated_output_w", 5000.0f).first,
        _sdf->Get<float>("efficiency", 0.35f).first,
        _sdf->Get<float>("voltage_nominal", 48.0f).first,
        _sdf->Get<std::string>("fuel_type", "diesel").first)
    {
        const std::string loggerName = "simpleGenerator_" + Generator::name();
        m_logger = logger::createConsoleAndFileLogger(
            loggerName, loggerName + ".txt");

        m_logger->info("SimpleGenerator [{}]: initialised with "
            "type = {} fuel {:.1f}/{:.1f} L, rated {:.0f} W, efficiency {:.2f}, voltage {:.1f} V",
            Generator::name(), m_fuel_type, m_fuel_level, m_fuel_capacity,
            m_rated_output_W, m_efficiency, m_voltage_nominal);
    }

    // ----------------------------------------------------------------
    // Generator interface
    // ----------------------------------------------------------------

    /**
     * @brief Consumes fuel proportional to current load over dt
     *        stub for FMU call — replace when wrapper is available -----------------
     * power_w         =   currentA * m_voltage_nominal
     * fuel_consumed   =   (power_w * dt) / (efficiency * 3_600_000)
     *                     (converts Wh -> litres via energy density stub)
     */
    void receiveLoad(float currentA, float dt) override;

private:
    std::shared_ptr<spdlog::logger> m_logger;
};
} //namespace lotusim::gazebo
