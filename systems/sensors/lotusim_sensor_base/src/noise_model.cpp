/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "lotusim_sensor_base/noise_model.hpp"

#include <algorithm>
#include <cmath>

namespace lotusim::sensor {

void GaussianNoiseModel::Load(const sdf::Noise& sdf)
{
    m_mean = sdf.Mean();
    m_stddev = std::max(sdf.StdDev(), 0.0);
    m_dynamic_bias_stddev = std::max(sdf.DynamicBiasStdDev(), 0.0);
    m_dynamic_bias_correlation_time = sdf.DynamicBiasCorrelationTime();
    m_precision = std::max(sdf.Precision(), 0.0);

    // The bias is drawn once; by convention bias_mean is its magnitude, so
    // it is negative half of the time.
    m_bias =
        sdf.BiasMean() + std::max(sdf.BiasStdDev(), 0.0) * m_unit_normal(m_rng);
    if (std::bernoulli_distribution(0.5)(m_rng)) {
        m_bias = -m_bias;
    }
}

double GaussianNoiseModel::Apply(double value, double dt)
{
    // Dynamic bias: first-order Gauss-Markov process, discretised over dt
    // (https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model).
    if (m_dynamic_bias_stddev > 0.0 && m_dynamic_bias_correlation_time > 0.0 &&
        dt > 0.0) {
        const double tau = m_dynamic_bias_correlation_time;
        const double sigma_d = std::sqrt(
            -m_dynamic_bias_stddev * m_dynamic_bias_stddev * tau / 2.0 *
            std::expm1(-2.0 * dt / tau));
        m_bias = std::exp(-dt / tau) * m_bias + sigma_d * m_unit_normal(m_rng);
    }

    double measured = value + m_bias + m_mean + m_stddev * m_unit_normal(m_rng);
    if (m_precision > 0.0) {
        measured = std::round(measured / m_precision) * m_precision;
    }
    return measured;
}

double GaussianNoiseModel::Variance() const
{
    return m_stddev * m_stddev;
}

double GaussianNoiseModel::Bias() const
{
    return m_bias;
}

std::unique_ptr<NoiseModel> CreateNoiseModel(
    const sdf::Noise& sdf,
    std::default_random_engine& rng)
{
    std::unique_ptr<NoiseModel> model;
    switch (sdf.Type()) {
        case sdf::NoiseType::GAUSSIAN:
        case sdf::NoiseType::GAUSSIAN_QUANTIZED:
            model = std::make_unique<GaussianNoiseModel>(rng);
            break;
        case sdf::NoiseType::NONE:
        default:
            return nullptr;
    }
    model->Load(sdf);
    return model;
}

}  // namespace lotusim::sensor
