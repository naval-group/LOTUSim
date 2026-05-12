#include "power_subsystem/simple_battery.hpp"
#include <algorithm>

namespace lotusim::gazebo
{

void SimpleBattery::receiveLoad(float currentA, float dt)
{
    // Drain voltage proportionally to current load over elapsed time
    // Stub for FMU call
    m_voltage -= (currentA * dt) / m_capacityAh;
    m_voltage  = std::max(m_voltage, m_voltageMin);
}

float SimpleBattery::voltage() const
{
    return m_voltage;
}

void SimpleBattery::receiveCharge(float currentA, float dt)
{
    // replace when FMU wrapper is available
    m_voltage += (currentA * dt) / m_capacityAh;
    m_voltage  = std::min(m_voltage, m_voltageNominal);
}

} // namespace lotusim::gazebo