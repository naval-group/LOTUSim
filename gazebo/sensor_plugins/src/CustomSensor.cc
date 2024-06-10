
#include "sensor_plugins/CustomSensor.hh"

namespace liquidai {
namespace gazebo {
CustomSensor::CustomSensor()
    : m_Namespace("")
{
    m_isOn.set_data(true);

    m_lastMeasurementTime = std::chrono::seconds(0);

    // Set seed for the noise generator
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    m_rndGen = std::default_random_engine(seed);
}

CustomSensor::~CustomSensor() {}

bool CustomSensor::Load(const sdf::Sensor &_sdf)
{
    gz::transport::NodeOptions gz_node_option;
    // TODO to look for namespace, if dont have just put the model name
    GetSDFParam<std::string>(_sdf.Element(), "namespace", m_Namespace, "");
    gz_node_option.SetNameSpace(m_Namespace);
    m_gazeboNode = std::make_unique<gz::transport::Node>(gz_node_option);

    gz::sensors::Sensor::Load(_sdf);

    CustomSensorLoad(_sdf);

    m_type = gz::sensors::customType(_sdf);

    sdf::ElementPtr _sdfptr = _sdf.Element();

    bool isSensorOn;
    GetSDFParam<bool>(_sdfptr, "is_on", isSensorOn, true);
    m_isOn.set_data(isSensorOn);

    m_gazeboNode->Advertise(
        Topic() + "/change_state", &CustomSensor::ChangeSensorState, this);

    GetSDFParam<double>(_sdfptr, "noise_sigma", m_noiseSigma, 0.0);
    GZ_ASSERT(
        m_noiseSigma >= 0.0,
        "Signal noise sigma must be greater or equal to zero");

    GetSDFParam<double>(_sdfptr, "noise_amplitude", m_noiseAmp, 0.0);
    GZ_ASSERT(
        m_noiseAmp >= 0.0,
        "Signal noise amplitude must be greater or equal to zero");

    AddNoiseModel("default", m_noiseSigma);

    return true;
}

bool CustomSensor::ChangeSensorState(
    const gz::msgs::Boolean &_req, gz::msgs::Boolean &_res)
{
    m_isOn.set_data(_req.data());
    _res.set_data(true);
    return true;
}

double CustomSensor::GetGaussianNoise(double _amp)
{
    return _amp * m_noiseModels["default"](m_rndGen);
}

double CustomSensor::GetGaussianNoise(std::string _name, double _amp)
{
    GZ_ASSERT(
        m_noiseModels.count(_name), "Gaussian noise model does not exist");
    return _amp * m_noiseModels[_name](m_rndGen);
}

bool CustomSensor::AddNoiseModel(std::string _name, double _sigma)
{
    if (m_noiseModels.count(_name))
        return false;

    m_noiseModels[_name] = std::normal_distribution<double>(0.0, _sigma);
    return true;
}

bool CustomSensor::IsOn() { return m_isOn.data(); }

void CustomSensor::NewPosition(const gz::math::Vector3d &_pos)
{
    m_position = _pos;
}

bool CustomSensor::EnableMeasurement(
    const std::chrono::steady_clock::duration &_now) const
{
    double dt = std::chrono::duration_cast<std::chrono::seconds>(
                    _now - m_lastMeasurementTime)
                    .count();
    return dt >= 1.0 / m_updateRate && m_isOn.data();
}

} // namespace gazebo
} // namespace liquidai
