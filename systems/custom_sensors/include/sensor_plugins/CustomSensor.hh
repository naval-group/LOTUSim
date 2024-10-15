#ifndef __CUSTOM_SENSOR_HH__
#define __CUSTOM_SENSOR_HH__

#include <gz/msgs/boolean.pb.h>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <chrono>
#include <gz/sensors/SensorTypes.hh>
#include <gz/sensors/Util.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <map>
#include <random>
#include <sensor_plugins/Common.hh>
#include <string>

#include "gz/sensors/Sensor.hh"
#include "logging_system/logger.hpp"

namespace lotusim::gazebo {

/*
   <pose relative_to='lidar_frame'>0 0 0 0 0 0</pose>
   <topic>lidar</topic>
   <update_rate>10</update_rate>

   <robot_namespace>${namespace}</robot_namespace>
   <noise_sigma>${noise_sigma}</noise_sigma>
   <noise_amplitude>${noise_amplitude}</noise_amplitude>

   <sensor_param> </sensor_param>
*/

class CustomSensor : public gz::sensors::Sensor {
public:
    /// \brief Class constructor
    CustomSensor();

    /// \brief Class destructor
    virtual ~CustomSensor();

    /// \brief Update callback from simulation.
    // virtual bool OnUpdate(const gz::sim::UpdateInfo &info) = 0;

    /// \brief Add noise normal distribution to the list
    bool AddNoiseModel(std::string _name, double _sigma);

    /// \brief Get the default Gaussian noise
    double GetGaussianNoise(double _amp);

    /// \brief Get the named Gaussian noise
    double GetGaussianNoise(std::string _name, double _amp);

    /// \brief Get status of the sensor
    bool IsOn();

    /// \brief Inherited function. Loading the sensor
    virtual bool Load(const sdf::Sensor &_sdf) override;

    virtual bool Update(const std::chrono::steady_clock::duration &_now) = 0;

    void NewPosition(const gz::math::Vector3d &_pos);

protected:
    virtual bool CustomSensorLoad(const sdf::Sensor &_sdf) = 0;

    bool EnableMeasurement(
        const std::chrono::steady_clock::duration &_now) const;

private:
    bool ChangeSensorState(
        const gz::msgs::Boolean &_req,
        gz::msgs::Boolean &_res);

protected:
    /**
     * @brief Spdlogger
     *
     */
    std::shared_ptr<spdlog::logger> m_logger;

    /// \brief Robot namespace
    std::string m_Namespace;

    /// \brief type of sensor loaded
    std::string m_type;

    /// \brief (Simulation) time when the last sensor measurement was
    /// generated.
    std::chrono::steady_clock::duration m_lastMeasurementTime;

    /// \brief Sensor update rate
    double m_updateRate;

    /// \brief Noise standard deviation
    double m_noiseSigma;

    /// \brief Noise amplitude
    double m_noiseAmp;

    /// \brief Pseudo random number generator
    std::default_random_engine m_rndGen;

    /// \brief Normal distribution describing the noise models
    std::map<std::string, std::normal_distribution<double>> m_noiseModels;

    /// \brief Flag to control the generation of output messages
    gz::msgs::Boolean m_isOn;

    /// \brief Gazebo's node handle for transporting measurement  messages.
    std::unique_ptr<gz::transport::Node> m_gazeboNode;

    /// \brief Gazebo's publisher for transporting measurement messages.
    gz::transport::Node::Publisher m_gazeboSensorOutputPub;

    gz::math::Vector3d m_position{std::nan(""), std::nan(""), std::nan("")};
};

}  // namespace lotusim::gazebo

#endif  // __ROS_BASE_PLUGIN_HH__
