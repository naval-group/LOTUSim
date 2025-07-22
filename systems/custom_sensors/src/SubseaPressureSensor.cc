#include <sensor_plugins/SubseaPressureSensor.hh>

namespace lotusim::gazebo {

SubseaPressureSensor::SubseaPressureSensor() {}

SubseaPressureSensor::~SubseaPressureSensor() {}

bool SubseaPressureSensor::CustomSensorLoad(const sdf::Sensor &_sdf)
{
    SetTopic("subsea_pressure_sensor/");
    sdf::ElementPtr _sdfptr = _sdf.Element();

    GetSDFParam<double>(_sdfptr, "m_saturation", m_saturation, 3000);
    GetSDFParam<bool>(_sdfptr, "estimate_depth_on", m_estimateDepth, false);
    GetSDFParam<double>(
        _sdfptr,
        "standard_pressure",
        m_standardPressure,
        101.325);
    GetSDFParam<double>(_sdfptr, "kPa_per_meter", m_kPaPerM, 9.80638);
    m_logger->info("Loading subsea pressure at topic {}", Topic());
    m_gazeboSensorOutputPub =
        m_gazeboNode->Advertise<gz_liquidai_msgs::msgs::Pressure>(
            Topic() + "sensor_data");
    return true;
}

bool SubseaPressureSensor::Update(
    const std::chrono::steady_clock::duration &_now)
{
    // Need to rewrite
    if (!EnableMeasurement(_now))
        return false;

    double depth = std::abs(m_position.Z());
    double pressure = m_standardPressure;
    if (depth >= 0) {
        // Convert depth to pressure
        pressure += depth * m_kPaPerM;
    }

    pressure += GetGaussianNoise(m_noiseAmp);
    double inferredDepth = 0.0;
    // Estimate depth, if enabled
    if (m_estimateDepth) {
        inferredDepth = (pressure - m_standardPressure) / m_kPaPerM;
    }
    gz_liquidai_msgs::msgs::Pressure gazeboMsg;

    gazeboMsg.set_pressure(pressure);
    gazeboMsg.set_stddev(m_noiseSigma);

    if (m_estimateDepth)
        gazeboMsg.set_depth(inferredDepth);
    m_gazeboSensorOutputPub.Publish(gazeboMsg);

    m_lastMeasurementTime = _now;

    return true;
}

}  // namespace lotusim::gazebo
