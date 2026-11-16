/*
 * Copyright (c) 2026 IRL Crossing
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "physics_engine_interface/gauss_markov_current.hpp"

#include <cmath>

namespace lotusim::gazebo {

GaussMarkovCurrent::GaussMarkovCurrent(
    double mean_x,
    double mean_y,
    double tau,
    double std_dev,
    unsigned int seed)
    : m_mean_x(mean_x),
      m_mean_y(mean_y),
      m_tau(tau),
      m_std_dev(std_dev),
      m_x(mean_x),
      m_y(mean_y),
      m_rng(seed)
{
}

std::pair<double, double> GaussMarkovCurrent::update(double dt)
{
    if (dt <= 0.0 || m_tau <= 0.0) {
        return {m_x, m_y};
    }
    const double a = std::exp(-dt / m_tau);
    // Exact step of the OU process, written so that the stationary standard
    // deviation is m_std_dev for any dt: var(V) = s^2 / (1 - a^2) = m_std_dev^2.
    const double s = m_std_dev * std::sqrt(1.0 - a * a);
    m_x = m_mean_x + a * (m_x - m_mean_x) + s * m_normal(m_rng);
    m_y = m_mean_y + a * (m_y - m_mean_y) + s * m_normal(m_rng);
    return {m_x, m_y};
}

}  // namespace lotusim::gazebo
