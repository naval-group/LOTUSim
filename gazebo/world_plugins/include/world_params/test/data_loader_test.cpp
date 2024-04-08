#include "world_params/DataLoader.h"
#include "gtest/gtest.h"

TEST(DataLoader, YamlLoaderWellFormed)
{
    std::map<std::chrono::steady_clock::duration, liquidai::WaveParameters>
        wave_yaml;
    EXPECT_NO_THROW(liquidai::loadFile(
        "wave_yaml_data.yaml", liquidai::FileFormat::YAML, wave_yaml));
    EXPECT_EQ(wave_yaml.begin()->second.direction[0], 10.0);
    EXPECT_EQ(wave_yaml.begin()->second.direction[1], 2.0);
    EXPECT_EQ(wave_yaml.begin()->second.amplitude, 13.0);
    EXPECT_EQ(wave_yaml.begin()->second.period, 4.0);
}
