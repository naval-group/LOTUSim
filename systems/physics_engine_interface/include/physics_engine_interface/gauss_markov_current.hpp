/*
 * Copyright (c) 2026 IRL Crossing
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_GAUSS_MARKOV_CURRENT_HH_
#define LOTUSIM_GAUSS_MARKOV_CURRENT_HH_

#include <random>
#include <utility>

namespace lotusim::gazebo {

/**
 * @brief First-order Gauss-Markov (Ornstein-Uhlenbeck) ocean current, uniform
 * over the water column: no depth-dependent shear, no rotation with depth.
 *
 * Follows Fossen's state-space current model (T. Fossen, "Handbook of Marine
 * Craft Hydrodynamics and Motion Control"), the same process conventionally
 * used for ocean current generation in Gazebo-based marine simulators such as
 * UUVSim and Project DAVE:
 *
 *   dV/dt = -(1/tau) * (V - V_mean) + driving noise
 *
 * Integrated with the exact step solution for a linear OU process, so both
 * the correlation time and the stationary spread are independent of the
 * simulation step size:
 *
 *   a = exp(-dt/tau)
 *   V(t+dt) = V_mean + a * (V(t) - V_mean) + std_dev * sqrt(1 - a^2) * N(0, 1)
 *
 * The process is parametrised by its STATIONARY STANDARD DEVIATION rather
 * than by the driving-noise intensity, because that is the quantity with a
 * direct physical meaning: it is directly comparable to the mean current, and
 * unlike the noise intensity it does not change meaning when tau changes.
 * (The two are related by std_dev = sigma * sqrt(tau/2).)
 *
 * Choosing std_dev: a current with std_dev > mean/3 reverses direction
 * regularly, since the mean lies within 3 standard deviations of zero. For a
 * persistent current, keep std_dev below a third of the mean speed.
 *
 * xdyn itself has no Gauss-Markov environment model (only "ekman current"),
 * so this current is not sent to xdyn as an environment parameter: it is
 * injected on the caller side by a Galilean frame change in
 * XdynWebsocket::getNewState.
 */
class GaussMarkovCurrent {
public:
    /**
     * @param mean_x, mean_y  mean current, NED world frame, m/s
     * @param tau             correlation time, s
     * @param std_dev         stationary standard deviation of each component,
     *                        m/s
     * @param seed            RNG seed, so a run is reproducible
     */
    GaussMarkovCurrent(
        double mean_x,
        double mean_y,
        double tau,
        double std_dev,
        unsigned int seed);

    /// Advances the process by `dt` seconds and returns the new current
    /// velocity (x, y), NED, world frame, m/s. w (vertical) is always 0.
    std::pair<double, double> update(double dt);

private:
    double m_mean_x;
    double m_mean_y;
    double m_tau;
    double m_std_dev;
    double m_x;
    double m_y;
    std::mt19937 m_rng;
    std::normal_distribution<double> m_normal{0.0, 1.0};
};

}  // namespace lotusim::gazebo

#endif
