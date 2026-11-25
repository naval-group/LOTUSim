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

#include <memory>

#include "lotusim_common/common.hpp"
#include "lotusim_sensor_base/custom_sensor.hpp"
#include "lotusim_sensor_base/noise_model.hpp"
#include "lotusim_sensor_msgs/msg/pressure_depth.hpp"

namespace lotusim::sensor {

/**
 * @brief Subsea pressure: the pressure at the sensor's depth, with noise,
 * and optionally the depth estimated back from it.
 *
 * Publishes lotusim_sensor_msgs/msg/PressureDepth on
 * <vessel>/<sensor name>/pressure_sensor; its stddev is the noise's (kPa),
 * 0 without noise.
 *
 * SDF, every tag optional unless noted:
 *
 * <sensor name="pressure_sensor"     <!-- required; part of the topic -->
 *         type="custom"              <!-- required -->
 *         gz:type="subsea_pressure"> <!-- required -->
 *
 *   <!-- Read by CustomSensor, as for every custom sensor -->
 *   <update_rate>10</update_rate>    <!-- Hz, default 1; 0 or less: every step -->
 *   <is_on>true</is_on>              <!-- default true; also set at runtime
 *                                         by <vessel>/<sensor>/change_state -->
 *   <seed>42</seed>                  <!-- unsigned; default: from the clock -->
 *
 *   <!-- The sensor's own -->
 *   <standard_pressure>101.325</standard_pressure>  <!-- kPa at the surface -->
 *   <kPa_per_meter>9.80638</kPa_per_meter>          <!-- kPa per m of depth -->
 *   <estimate_depth_on>false</estimate_depth_on>    <!-- publish the depth
 *                                                        estimated from the
 *                                                        measured pressure -->
 *   <saturation>3000</saturation>                   <!-- kPa, output cap,
 *                                                        noise included;
 *                                                        <m_saturation> also
 *                                                        read -->
 *
 *   <!-- Noise, in the form of Gazebo's air pressure sensor -->
 *   <pressure>
 *     <noise type="gaussian">        <!-- none | gaussian | gaussian_quantized -->
 *       <mean>0</mean>               <!-- kPa; all values default 0 -->
 *       <stddev>0.1</stddev>         <!-- kPa, white noise each measurement -->
 *       <bias_mean>0.05</bias_mean>  <!-- kPa, magnitude; sign drawn at load -->
 *       <bias_stddev>0.01</bias_stddev>
 *       <dynamic_bias_stddev>0.001</dynamic_bias_stddev>
 *       <dynamic_bias_correlation_time>600</dynamic_bias_correlation_time>
 *                                    <!-- s; with dynamic_bias_stddev, the bias
 *                                         drifts (Gauss-Markov) -->
 *       <precision>0.01</precision>  <!-- kPa; output rounded to a multiple -->
 *     </noise>
 *   </pressure>
 * </sensor>
 *
 * See lotusim_sensor_base/noise_model.hpp for the noise model.
 */
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

    /// \brief The most the sensor reads (kPa), from <saturation>
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

    /// \brief Noise on the pressure (kPa), from <pressure><noise>; nullptr
    /// for none
    std::unique_ptr<NoiseModel> m_pressure_noise;
};

}  // namespace lotusim::sensor

#endif  // __SUBSEA_PRESSURE_ROS_PLUGIN_HH__
