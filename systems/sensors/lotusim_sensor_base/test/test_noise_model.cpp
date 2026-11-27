/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "lotusim_sensor_base/noise_model.hpp"

using lotusim::sensor::CreateNoiseModel;

namespace {

sdf::Noise gaussian(double mean, double stddev)
{
    sdf::Noise noise;
    noise.SetType(sdf::NoiseType::GAUSSIAN);
    noise.SetMean(mean);
    noise.SetStdDev(stddev);
    return noise;
}

double sample_stddev(const std::vector<double>& values)
{
    double mean = 0.0;
    for (const double v : values) {
        mean += v / values.size();
    }
    double variance = 0.0;
    for (const double v : values) {
        variance += (v - mean) * (v - mean) / (values.size() - 1);
    }
    return std::sqrt(variance);
}

}  // namespace

TEST(NoiseModel, NoneGivesNoModel)
{
    std::default_random_engine rng(1);
    EXPECT_EQ(CreateNoiseModel(sdf::Noise(), rng), nullptr);
    sdf::Noise none;
    none.SetStdDev(0.2);
    EXPECT_EQ(CreateNoiseModel(none, rng), nullptr);
}

TEST(NoiseModel, GaussianMeanAndStdDev)
{
    std::default_random_engine rng(1);
    const auto model = CreateNoiseModel(gaussian(0.3, 0.2), rng);
    std::vector<double> values;
    double mean = 0.0;
    for (int i = 0; i < 5000; ++i) {
        values.push_back(model->Apply(1.0, 0.1));
        mean += values.back() / 5000;
    }
    EXPECT_NEAR(mean, 1.3, 0.02);
    EXPECT_NEAR(sample_stddev(values), 0.2, 0.01);
    EXPECT_DOUBLE_EQ(model->Variance(), 0.04);
}

// bias_mean is the bias' magnitude; its sign is drawn on load, then fixed.
TEST(NoiseModel, StaticBias)
{
    sdf::Noise noise = gaussian(0.0, 0.0);
    noise.SetBiasMean(0.5);
    bool positive = false, negative = false;
    for (unsigned seed = 0; seed < 32; ++seed) {
        std::default_random_engine rng(seed);
        const auto model = CreateNoiseModel(noise, rng);
        EXPECT_DOUBLE_EQ(std::abs(model->Bias()), 0.5);
        for (int i = 0; i < 10; ++i) {
            EXPECT_DOUBLE_EQ(model->Apply(2.0, 0.1), 2.0 + model->Bias());
        }
        (model->Bias() > 0 ? positive : negative) = true;
    }
    EXPECT_TRUE(positive);
    EXPECT_TRUE(negative);
}

TEST(NoiseModel, BiasStdDev)
{
    sdf::Noise noise = gaussian(0.0, 0.0);
    noise.SetBiasMean(1.0);
    noise.SetBiasStdDev(0.1);
    std::vector<double> biases;
    for (unsigned seed = 0; seed < 2000; ++seed) {
        std::default_random_engine rng(seed);
        biases.push_back(std::abs(CreateNoiseModel(noise, rng)->Bias()));
    }
    EXPECT_NEAR(sample_stddev(biases), 0.1, 0.01);
}

// The dynamic bias drifts with time, and only with time.
TEST(NoiseModel, DynamicBias)
{
    sdf::Noise noise = gaussian(0.0, 0.0);
    noise.SetDynamicBiasStdDev(0.1);
    noise.SetDynamicBiasCorrelationTime(10.0);
    std::default_random_engine rng(1);
    const auto model = CreateNoiseModel(noise, rng);
    const double initial = model->Bias();
    model->Apply(0.0, 0.0);
    EXPECT_EQ(model->Bias(), initial);
    model->Apply(0.0, 1.0);
    EXPECT_NE(model->Bias(), initial);
}

TEST(NoiseModel, Precision)
{
    sdf::Noise noise = gaussian(0.0, 0.3);
    noise.SetType(sdf::NoiseType::GAUSSIAN_QUANTIZED);
    noise.SetPrecision(0.25);
    std::default_random_engine rng(1);
    const auto model = CreateNoiseModel(noise, rng);
    for (int i = 0; i < 100; ++i) {
        const double steps = model->Apply(1.0, 0.1) / 0.25;
        EXPECT_NEAR(steps, std::round(steps), 1e-9);
    }
}
