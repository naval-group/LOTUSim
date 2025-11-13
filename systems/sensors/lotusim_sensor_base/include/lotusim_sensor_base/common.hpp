/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */

// This source code is from rotors_simulator
//   (https://github.com/ethz-asl/rotors_simulator)
// * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
// * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
// * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
// * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
// * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
// This source code is licensed under the Apache-2.0 license found in the
// open_source_licenses.txt file in the root directory of this source tree.
//
// The original code was modified by the UUV Simulator Authors to adhere to
// Gazebo's coding standards.

#ifndef SENSOR_PLUGINS_COMMON_H_
#define SENSOR_PLUGINS_COMMON_H_

#include <Eigen/Dense>
#include <string>

namespace lotusim::sensor {

/**
 * \brief Obtains a parameter from sdf.
 * \param[in] sdf Pointer to the sdf object.
 * \param[in] name Name of the parameter.
 * \param[out] param Param Variable to write the parameter to.
 * \param[in] default_value Default value, if the parameter not available.
 * \param[in] verbose If true, gzerror if the parameter is not available.
 */
template <class T>
bool GetSDFParam(
    sdf::ElementPtr sdf,
    const std::string &name,
    T &param,
    const T &default_value,
    const bool &verbose = false)
{
    if (sdf->HasElement(name)) {
        param = sdf->GetElement(name)->Get<T>();
        return true;
    } else {
        param = default_value;
        if (verbose)
            gzerr << "[sensor_plugins] Please specify a value for "
                     "parameter \""
                  << name << "\"." << std::endl;
    }
    return false;
}
}  // namespace lotusim::sensor

template <typename T>
class FirstOrderFilter {
    /*
    This class can be used to apply a first order filter on a signal.
    It allows different acceleration and deceleration time constants.

    Short reveiw of discrete time implementation of firest order system:
    Laplace:
        X(s)/U(s) = 1/(tau*s + 1)
    continous time system:
        dx(t) = (-1/tau)*x(t) + (1/tau)*u(t)
    discretized system (ZoH):
        x(k+1) = exp(samplingTime*(-1/tau))*x(k) + (1 -
    exp(samplingTime*(-1/tau))) * u(k)
    */

public:
    FirstOrderFilter(
        double timeConstantUp,
        double timeConstantDown,
        T initialState)
        : timeConstantUp_(timeConstantUp)
        , timeConstantDown_(timeConstantDown)
        , previousState_(initialState)
    {
    }

    T updateFilter(T inputState, double samplingTime)
    {
        /*
        This method will apply a first order filter on the inputState.
        */
        T outputState;
        if (inputState > previousState_) {
            // Calcuate the outputState if accelerating.
            double alphaUp = exp(-samplingTime / timeConstantUp_);
            // x(k+1) = Ad*x(k) + Bd*u(k)
            outputState = alphaUp * previousState_ + (1 - alphaUp) * inputState;
        } else {
            // Calculate the outputState if decelerating.
            double alphaDown = exp(-samplingTime / timeConstantDown_);
            outputState =
                alphaDown * previousState_ + (1 - alphaDown) * inputState;
        }
        previousState_ = outputState;
        return outputState;
    }
    ~FirstOrderFilter() {}

protected:
    double timeConstantUp_;
    double timeConstantDown_;
    T previousState_;
};

/// Computes a quaternion from the 3-element small angle approximation theta.
template <class Derived>
Eigen::Quaternion<typename Derived::Scalar> QuaternionFromSmallAngle(
    const Eigen::MatrixBase<Derived> &theta)
{
    typedef typename Derived::Scalar Scalar;
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    const Scalar q_squared = theta.squaredNorm() / 4.0;

    if (q_squared < 1) {
        return Eigen::Quaternion<Scalar>(
            sqrt(1 - q_squared),
            theta[0] * 0.5,
            theta[1] * 0.5,
            theta[2] * 0.5);
    } else {
        const Scalar w = 1.0 / sqrt(1 + q_squared);
        const Scalar f = w * 0.5;
        return Eigen::Quaternion<Scalar>(
            w,
            theta[0] * f,
            theta[1] * f,
            theta[2] * f);
    }
}

template <class In, class Out>
void copyPosition(const In &in, Out *out)
{
    out->x = in.x;
    out->y = in.y;
    out->z = in.z;
}

#endif  // SENSOR_PLUGINS_COMMON_H_
