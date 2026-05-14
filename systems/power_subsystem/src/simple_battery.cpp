#include "power_subsystem/simple_battery.hpp"
#include <algorithm>
#include "lotusim_common/common.hpp" 

namespace lotusim::gazebo
{

void SimpleBattery::receiveLoad(float currentA, float dt)
{
    // Drain voltage proportionally to current load over elapsed time
    // Stub for FMU call
    m_voltage -= (currentA * dt) / m_capacityAh;
    m_voltage  = std::max(m_voltage, m_voltageMin);
    m_logger->info("SimpleBattery [{}]: receiveLoad current={:.3f} A dt={:.4f} s -> voltage={:.3f} V",
        Battery::name(), currentA, dt, m_voltage);
}

float SimpleBattery::voltage() const
{
    return m_voltage;
}

void SimpleBattery::receiveCharge(float currentA, float dt)
{
    // replace when FMU wrapper is available
    m_voltage += (currentA * dt) / m_capacityAh;
    m_voltage  = std::min(m_voltage, m_voltage_nominal);

    m_logger->info("SimpleBattery [{}]: receiveCharge current={:.3f} A dt={:.4f} s -> voltage={:.3f} V",
        Battery::name(), currentA, dt, m_voltage);

    if (m_voltage >= m_voltage_nominal) {
        m_logger->info(
            "SimpleBattery [{}]: fully charged ({:.3f} V)",
            Battery::name(), m_voltage);
    }
}

float SimpleBattery::getStateOfCharge() const
{
    if (m_voltage_nominal <= m_voltageMin) { return 0.0f; }
    const float soc = (m_voltage - m_voltageMin) 
                    / (m_voltage_nominal - m_voltageMin);
    return std::clamp(soc, 0.0f, 1.0f);
}
} // namespace lotusim::gazebo