/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_NOISE_MODEL_HPP
#define LOTUSIM_NOISE_MODEL_HPP

#include <memory>
#include <random>
#include <sdf/Noise.hh>

namespace lotusim::sensor {

/**
 * @brief Noise on one scalar measurement, read from an SDF <noise> element.
 *
 * Sensors hold one model per measured axis and pass each true value through
 * Apply(). The random engine is the sensor's: it must outlive the model.
 */
class NoiseModel {
public:
    explicit NoiseModel(std::default_random_engine& rng) : m_rng(rng) {}
    virtual ~NoiseModel() = default;

    NoiseModel(const NoiseModel&) = delete;
    NoiseModel& operator=(const NoiseModel&) = delete;

    /// @brief Read the parameters of an SDF <noise> element.
    virtual void Load(const sdf::Noise& /*sdf*/) {}

    /**
     * @brief The measured value: the true value with noise.
     *
     * @param value true value
     * @param dt time since the previous Apply() on this model (s), for the
     * noise that evolves over time
     */
    virtual double Apply(double value, double dt) = 0;

    /// @brief Variance of the noise drawn at each measurement, for the
    /// covariances a message carries.
    virtual double Variance() const = 0;

    /// @brief Current bias added to every measurement.
    virtual double Bias() const = 0;

protected:
    std::default_random_engine& m_rng;
};

/**
 * @brief Gaussian noise with a bias, as Gazebo's GaussianNoiseModel.
 *
 * measured = value + bias + N(mean, stddev), rounded to <precision> when set.
 * The bias is drawn once on Load() from N(bias_mean, bias_stddev), its sign
 * flipped with probability 1/2. With <dynamic_bias_stddev> and
 * <dynamic_bias_correlation_time> set, it then drifts as a first-order
 * Gauss-Markov process.
 */
class GaussianNoiseModel : public NoiseModel {
public:
    using NoiseModel::NoiseModel;

    void Load(const sdf::Noise& sdf) override;
    double Apply(double value, double dt) override;
    double Variance() const override;
    double Bias() const override;

private:
    double m_mean{0.0};
    double m_stddev{0.0};
    double m_bias{0.0};
    double m_dynamic_bias_stddev{0.0};
    double m_dynamic_bias_correlation_time{0.0};
    double m_precision{0.0};
    std::normal_distribution<double> m_unit_normal{0.0, 1.0};
};

/// @brief The model for an SDF <noise> element's type, loaded from it;
/// nullptr for type none.
std::unique_ptr<NoiseModel> CreateNoiseModel(
    const sdf::Noise& sdf,
    std::default_random_engine& rng);

}  // namespace lotusim::sensor

#endif  // LOTUSIM_NOISE_MODEL_HPP
