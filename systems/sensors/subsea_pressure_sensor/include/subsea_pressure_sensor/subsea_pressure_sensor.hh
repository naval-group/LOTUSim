// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SUBSEA_PRESSURE_ROS_PLUGIN_HH_
#define SUBSEA_PRESSURE_ROS_PLUGIN_HH_

#include "lotusim_common/common.hpp"
#include "lotusim_sensor_base/custom_sensor.hpp"
#include "lotusim_sensor_msgs/msg/pressure_depth.hpp"

namespace lotusim::sensor {
class SubseaPressureSensor : public CustomSensor {
public:
    /// \brief Class constructor
    SubseaPressureSensor(
        std::shared_ptr<spdlog::logger> logger,
        rclcpp::Node::SharedPtr node,
        const gz::sim::Entity& vessel_entity,
        const gz::sim::Entity& sensor_entity,
        const std::string& parent_name,
        const std::string& sensor_name);

    /// \brief Class destructor
    ~SubseaPressureSensor();

    /// \brief Update sensor measurement
    virtual bool UpdateSensor(
        const gz::sim::UpdateInfo& _info,
        const gz::sim::EntityComponentManager& _ecm) final;

private:
    virtual bool CustomSensorLoad(const sdf::Sensor& _sdf) final;

    /// \brief Sensor m_saturation (max. value for output pressure in Pa)
    double m_saturation;

    /// \brief If flag is set to true, estimate depth according to pressure
    /// measurement
    bool m_estimate_depth;

    /// \brief Standard pressure
    double m_standard_pressure;

    /// \brief Factor of kPa per meter
    double m_kPa_per_m;

    rclcpp::Publisher<lotusim_sensor_msgs::msg::PressureDepth>::SharedPtr
        m_sensor_pub;

    gz::sensors::NoisePtr noise{nullptr};
};

}  // namespace lotusim::sensor

#endif  // __SUBSEA_PRESSURE_ROS_PLUGIN_HH__
