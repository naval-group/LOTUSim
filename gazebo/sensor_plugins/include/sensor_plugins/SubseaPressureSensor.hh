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

#ifndef __SUBSEA_PRESSURE_ROS_PLUGIN_HH__
#define __SUBSEA_PRESSURE_ROS_PLUGIN_HH__

#include "gz_liquidai_msgs/msgs/SensorPressure.pb.h"
#include "sensor_plugins/CustomSensor.hh"

#include <gz/common/Console.hh>
#include <gz/math/Vector3.hh>

namespace liquidai {
namespace gazebo {

class SubseaPressureSensor : public CustomSensor {

public:
    /// \brief Class constructor
    SubseaPressureSensor();

    /// \brief Class destructor
    ~SubseaPressureSensor();

    /// \brief Update sensor measurement
    virtual bool Update(const std::chrono::steady_clock::duration &_now) final;

private:
    virtual bool CustomSensorLoad(const sdf::Sensor &_sdf) final;

protected:
    /// \brief Sensor m_saturation (max. value for output pressure in Pa)
    double m_saturation;

    /// \brief If flag is set to true, estimate depth according to pressure
    /// measurement
    bool m_estimateDepth;

    /// \brief Standard pressure
    double m_standardPressure;

    /// \brief Factor of kPa per meter
    double m_kPaPerM;

private:
    gz::sensors::NoisePtr noise{nullptr};
    gz::transport::Node node;
    gz::transport::Node::Publisher pub;
};

} // namespace gazebo
} // namespace liquidai

#endif // __SUBSEA_PRESSURE_ROS_PLUGIN_HH__
