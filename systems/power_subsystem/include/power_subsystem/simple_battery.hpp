#pragma once

#include "power_subsystem/battery.hpp"

#include <gz/common/Console.hh>
#include "lotusim_common/common.hpp"
#include "lotusim_common/logger.hpp"

namespace lotusim::gazebo
{

/**
 * @brief minimal battery for integration testing
 *
 * For now, testing: stubs out FMU interaction with a simple linear voltage drain:
 *
 *   voltage -= (currentA * dt) / capacity_ah
 *
 * replace receiveLoad(), voltage() and receiveCharge() with
 * FMU calls once the FMU wrapper interface is available
 *
 * SDF format (inside <plugin> tag):
 *   <provider name="test_battery" type="simple_battery"
 *             capacity_ah="100" initial_soc="1.0"
 *             voltage_min="36.0" voltage_nominal="48.0"/>
 */
class SimpleBattery : public Battery
{
public:
    /**
     * @param name   Battery name from SDF
     * @param _sdf   reads capacity_ah, initial_soc,
     *               voltage_min, voltage_nominal
     * @param node   node from PowerManager
     */
    SimpleBattery(
        std::string name,
        const sdf::ElementPtr& _sdf,
        rclcpp::Node::SharedPtr node)
        : Battery(
            std::move(name),
            std::move(node),
            _sdf->Get<float>("capacity_ah",    100.0f).first,
            _sdf->Get<float>("initial_soc",      1.0f).first,
            _sdf->Get<float>("voltage_min",     36.0f).first)
        , m_voltage(_sdf->Get<float>("voltage_nominal", 48.0f).first)
    {
        m_logger = logger::createConsoleAndFileLogger(
            "simpleBattery_" + Battery::name(),
            "simpleBattery_" + Battery::name() + ".txt");
    }

    // ----------------------------------------------------------------
    // Battery interface
    // ----------------------------------------------------------------

    /**
     * @brief Drains voltage proportionally to current load over dt
     *        stub for FMU call — replace when wrapper is available -----------------
     */
    void receiveLoad(float currentA, float dt) override;

    /**
     * @brief returns last computed voltage (V)
     *        real implementation reads from FMU output.
     */
    float voltage() const override;

    /**
     * @brief Charges battery by increasing voltage toward nominal
     *        stub for FMU call, replace when wrapper is available     
     */
    void receiveCharge(float currentA, float dt) override;

private:
    // current battery voltage: starts at voltage_nominal, drains toward
    // voltage_min as load is applied. Updated by receiveLoad()
    float m_voltage{48.0f};

    // full-charge voltage from SDF voltage_nominal
    float m_voltageNominal{48.0f};

    std::shared_ptr<spdlog::logger> m_logger;
};

} // namespace lotusim::gazebo