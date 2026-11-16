/*
 * Copyright (c) 2026 IRL Crossing
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef LOTUSIM_COPERNICUS_CURRENT_HH_
#define LOTUSIM_COPERNICUS_CURRENT_HH_

#include <string>
#include <utility>
#include <vector>

namespace lotusim::gazebo {

/**
 * @brief Measured ocean current replayed from a Copernicus depth profile.
 *
 * Where GaussMarkovCurrent synthesises a current from a stochastic process and
 * xdyn's own "ekman current" synthesises one from an analytical model, this
 * class replays a profile that was *measured*: it reads the depth-resolved
 * velocities of a Copernicus Marine Service reanalysis product and returns the
 * velocity actually reported at the vehicle's depth. It exists so a controller
 * can be evaluated against a real ocean current rather than only against
 * modelled ones.
 *
 * The profile is supplied as a small CSV, one row per depth level:
 *
 *     depth_m,vx_north_ms,vy_east_ms
 *
 * produced from the Copernicus export by
 * `extract_copernicus_profile.py` (in the scenario repository's
 * bluerov_current_experiment tooling), which handles the reductions that make
 * the profile comparable to the modelled conditions: horizontal averaging over
 * the study region, selection of one time slice, and the eastward/northward to
 * NED component mapping. Keeping those in Python leaves this class with a
 * single job -- look up a depth -- and leaves the data reduction auditable
 * outside the simulator.
 *
 * Between tabulated levels the velocity is linearly interpolated; outside the
 * tabulated range it is clamped to the shallowest/deepest level rather than
 * extrapolated, since extrapolating a measured profile past its own support
 * would invent data.
 *
 * Like the Gauss-Markov current, and for the same reason (xdyn has no
 * environment model for it), this current is not sent to xdyn as an
 * environment parameter: it is injected by the Galilean frame change in
 * XdynWebsocket::getNewState.
 */
class CopernicusCurrent {
public:
    /**
     * @param profile_csv  path to the depth-profile CSV described above
     * @throws std::runtime_error if the file cannot be read or holds no
     *         usable rows -- failing loudly is deliberate, since silently
     *         falling back to zero current would make a run look like a
     *         no-current run rather than a failed one.
     */
    explicit CopernicusCurrent(const std::string& profile_csv);

    /// Current velocity at `depth_m` below the surface (NED, world frame,
    /// m/s): x north, y east. Vertical velocity is not part of the product
    /// and is treated as zero, as it is for the other current sources.
    std::pair<double, double> at(double depth_m) const;

    /// Number of depth levels read, for logging.
    std::size_t levels() const { return m_depth.size(); }

private:
    // Parallel arrays, sorted by increasing depth.
    std::vector<double> m_depth;
    std::vector<double> m_vx;  // north
    std::vector<double> m_vy;  // east
};

}  // namespace lotusim::gazebo

#endif
