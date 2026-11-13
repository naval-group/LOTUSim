/*
 * Copyright (c) 2026 IRL Crossing
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "physics_engine_interface/copernicus_current.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <tuple>

namespace lotusim::gazebo {

namespace {

/// True for a header line or a blank/comment line, i.e. anything that is not
/// a data row. The header is detected by content rather than by position so a
/// file without one still loads.
bool isNotData(const std::string& line)
{
    if (line.empty() || line[0] == '#') {
        return true;
    }
    // A data row starts with a number; the header starts with "depth_m".
    const char c = line[0];
    return !(c == '-' || c == '+' || c == '.' || (c >= '0' && c <= '9'));
}

}  // namespace

CopernicusCurrent::CopernicusCurrent(const std::string& profile_csv)
{
    std::ifstream in(profile_csv);
    if (!in) {
        throw std::runtime_error(
            "CopernicusCurrent: cannot open profile '" + profile_csv + "'");
    }

    std::vector<std::tuple<double, double, double>> rows;
    std::string line;
    std::size_t line_no = 0;
    while (std::getline(in, line)) {
        ++line_no;
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();  // tolerate CRLF
        }
        if (isNotData(line)) {
            continue;
        }
        std::istringstream ss(line);
        std::string depth_s, vx_s, vy_s;
        if (!std::getline(ss, depth_s, ',') ||
            !std::getline(ss, vx_s, ',') ||
            !std::getline(ss, vy_s, ',')) {
            throw std::runtime_error(
                "CopernicusCurrent: '" + profile_csv + "' line " +
                std::to_string(line_no) +
                ": expected depth_m,vx_north_ms,vy_east_ms");
        }
        try {
            rows.emplace_back(
                std::stod(depth_s), std::stod(vx_s), std::stod(vy_s));
        } catch (const std::exception&) {
            throw std::runtime_error(
                "CopernicusCurrent: '" + profile_csv + "' line " +
                std::to_string(line_no) + ": non-numeric value");
        }
    }

    if (rows.empty()) {
        throw std::runtime_error(
            "CopernicusCurrent: '" + profile_csv + "' holds no data rows");
    }

    // The extractor already sorts, but a hand-edited profile might not, and
    // the interpolation below assumes increasing depth.
    std::sort(rows.begin(), rows.end(),
              [](const auto& a, const auto& b) {
                  return std::get<0>(a) < std::get<0>(b);
              });

    m_depth.reserve(rows.size());
    m_vx.reserve(rows.size());
    m_vy.reserve(rows.size());
    for (const auto& [d, vx, vy] : rows) {
        m_depth.push_back(d);
        m_vx.push_back(vx);
        m_vy.push_back(vy);
    }
}

std::pair<double, double> CopernicusCurrent::at(double depth_m) const
{
    // Clamp rather than extrapolate outside the measured range: above the
    // shallowest level (including a vehicle momentarily at or above the
    // surface, depth <= 0) and below the deepest one, the nearest measured
    // value is the only defensible answer.
    if (depth_m <= m_depth.front()) {
        return {m_vx.front(), m_vy.front()};
    }
    if (depth_m >= m_depth.back()) {
        return {m_vx.back(), m_vy.back()};
    }

    const auto upper =
        std::lower_bound(m_depth.begin(), m_depth.end(), depth_m);
    const std::size_t hi = static_cast<std::size_t>(upper - m_depth.begin());
    const std::size_t lo = hi - 1;

    const double span = m_depth[hi] - m_depth[lo];
    // Coincident levels would divide by zero; fall back on the shallower one.
    const double w = (span > 0.0) ? (depth_m - m_depth[lo]) / span : 0.0;
    return {m_vx[lo] + w * (m_vx[hi] - m_vx[lo]),
            m_vy[lo] + w * (m_vy[hi] - m_vy[lo])};
}

}  // namespace lotusim::gazebo
