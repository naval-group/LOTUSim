/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
// Fmi2Slave and FmuInterface against test_vessel.c, a vessel moving straight
// ahead at u = gain * gear * propeller, packed into an .fmu by the tests with
// the variable names of lxdyn's FMU.
#include <gtest/gtest.h>
#include <spdlog/sinks/null_sink.h>
#include <zip.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sdf/sdf.hh>

#include "physics_engine_interface/fmi2_slave.hpp"
#include "physics_engine_interface/fmu_interface.hpp"

using namespace lotusim::gazebo;
namespace fs = std::filesystem;

namespace {

const char* const STATES[13] =
    {"x", "y", "z", "u", "v", "w", "p", "q", "r", "qr", "qi", "qj", "qk"};

std::string modelDescription(
    const std::string& version = "2.0",
    bool cosimulation = true,
    bool with_qk = true,
    bool with_reset = true)
{
    std::string vars;
    for (int i = 0; i < 13; ++i) {
        if (!with_qk && std::string(STATES[i]) == "qk") {
            continue;
        }
        vars += "<ScalarVariable name=\"Boat." + std::string(STATES[i]) +
                "\" valueReference=\"" + std::to_string(1 + i) +
                "\" causality=\"output\"><Real/></ScalarVariable>\n";
    }
    vars +=
        "<ScalarVariable name=\"gain\" valueReference=\"100\" causality=\"parameter\" variability=\"fixed\"><Real start=\"0.1\"/></ScalarVariable>\n";
    vars +=
        "<ScalarVariable name=\"gear\" valueReference=\"101\" causality=\"parameter\" variability=\"fixed\"><Enumeration declaredType=\"Gear\" start=\"1\"/></ScalarVariable>\n";
    if (with_reset) {
        vars +=
            "<ScalarVariable name=\"Boat.reset_state\" valueReference=\"200\" causality=\"input\"><Boolean start=\"false\"/></ScalarVariable>\n";
        for (int i = 0; i < 13; ++i) {
            vars +=
                "<ScalarVariable name=\"Boat.reset_" + std::string(STATES[i]) +
                "\" valueReference=\"" + std::to_string(201 + i) +
                "\" causality=\"input\"><Real start=\"0\"/></ScalarVariable>\n";
        }
    }
    vars +=
        "<ScalarVariable name=\"Boat.propeller.rpm\" valueReference=\"1000\" causality=\"input\" description=\"propeller(rpm)\"><Real start=\"0\"/></ScalarVariable>\n";
    vars +=
        "<ScalarVariable name=\"steps\" valueReference=\"1001\" causality=\"output\"><Real/></ScalarVariable>\n";
    return "<?xml version=\"1.0\"?>\n<fmiModelDescription fmiVersion=\"" +
           version + "\" modelName=\"test_vessel\" guid=\"{0}\">\n" +
           (cosimulation ? "<CoSimulation modelIdentifier=\"test_vessel\"/>\n"
                         : "") +
           "<TypeDefinitions><SimpleType name=\"Gear\"><Enumeration>"
           "<Item name=\"low\" value=\"1\"/><Item name=\"high\" value=\"2\"/>"
           "</Enumeration></SimpleType></TypeDefinitions>\n"
           "<ModelVariables>\n" + vars +
           "</ModelVariables>\n</fmiModelDescription>\n";
}

// An .fmu holding `description` and the test FMU's library.
std::string packFmu(const std::string& name, const std::string& description)
{
    const fs::path path =
        fs::temp_directory_path() / ("lotusim-test-" + name + ".fmu");
    fs::remove(path);
    int error = 0;
    zip_t* z = zip_open(path.c_str(), ZIP_CREATE | ZIP_TRUNCATE, &error);
    EXPECT_NE(z, nullptr);
    auto* text = new std::string(description);  // freed with the archive
    zip_source_t* xml = zip_source_buffer(z, text->data(), text->size(), 0);
    zip_file_add(z, "modelDescription.xml", xml, ZIP_FL_OVERWRITE);
    zip_source_t* lib = zip_source_file(z, TEST_FMU_LIBRARY, 0, -1);
    zip_file_add(z, "binaries/linux64/test_vessel.so", lib, ZIP_FL_OVERWRITE);
    EXPECT_EQ(zip_close(z), 0);
    delete text;
    return path.string();
}

std::shared_ptr<spdlog::logger> quietLogger()
{
    return std::make_shared<spdlog::logger>(
        "test",
        std::make_shared<spdlog::sinks::null_sink_mt>());
}

sdf::ElementPtr domainSdf(
    const std::string& fmu,
    const std::string& parameters = "")
{
    // Inside a <plugin>, whose content sdformat keeps as written.
    const std::string text =
        "<sdf version='1.9'><model name='m'><link name='l'/><plugin name='p' filename='f'><surface>"
        "<interface_type>FMU</interface_type><fmu>" +
        fmu + "</fmu>" +
        (parameters.empty()
             ? ""
             : "<fmu_parameters>" + parameters + "</fmu_parameters>") +
        "</surface></plugin></model></sdf>";
    auto root = std::make_shared<sdf::SDF>();
    sdf::init(root);
    sdf::readString(text, root);
    return root->Root()->GetElement("model")->GetElement("plugin")->GetElement(
        "surface");
}

class FmuInterfaceTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        FmuInterface::resetInstance();
        interface = FmuInterface::createInterface("test_world");
        interface->setLogger(quietLogger());
        commands = std::make_shared<
            std::unordered_map<gz::sim::Entity, std::string>>();
        interface->setSharedCmd(commands);
    }
    void TearDown() override
    {
        FmuInterface::resetInstance();
        // Registered by name in PhysicsInterfaceBase's constructor.
        spdlog::drop("test_world_FmuInterface_physics_engine_output");
    }

    std::shared_ptr<FmuInterface> interface;
    std::shared_ptr<std::unordered_map<gz::sim::Entity, std::string>> commands;
};

}  // namespace

TEST(Fmi2SlaveTest, LoadsSetsStepsAndReads)
{
    Fmi2Slave fmu(packFmu("ok", modelDescription()), "a", quietLogger());
    EXPECT_EQ(fmu.modelName(), "test_vessel");
    EXPECT_TRUE(fmu.hasVariable("Boat.propeller.rpm"));
    EXPECT_EQ(fmu.variable("Boat.x").causality, "output");
    EXPECT_EQ(fmu.variable("Boat.propeller.rpm").description, "propeller(rpm)");
    fmu.setFromString("gain", "0.5");
    fmu.initialize(0.0);
    fmu.setReal("Boat.propeller.rpm", 2.0);
    fmu.doStep(0.0, 1.0);
    // Facing north (identity quaternion in NED): 0.5 * 2 m/s for 1 s.
    EXPECT_DOUBLE_EQ(fmu.getReal("Boat.x"), 1.0);
    EXPECT_DOUBLE_EQ(fmu.getReal("Boat.y"), 0.0);
    EXPECT_DOUBLE_EQ(fmu.getReal("Boat.u"), 1.0);
}

// lxdyn's FMU declares xdyn.solver an Enumeration: it is set by item name.
TEST(Fmi2SlaveTest, EnumerationSetByItemName)
{
    Fmi2Slave fmu(packFmu("enum", modelDescription()), "a", quietLogger());
    fmu.setFromString("gain", "0.5");
    fmu.setFromString("gear", "high");
    fmu.initialize(0.0);
    fmu.setReal("Boat.propeller.rpm", 2.0);
    fmu.doStep(0.0, 1.0);
    EXPECT_DOUBLE_EQ(fmu.getReal("Boat.x"), 2.0);
    EXPECT_THROW(fmu.setFromString("gear", "reverse"), FmuError);
}

TEST(Fmi2SlaveTest, TwoInstancesOfOneFmuAreIndependent)
{
    const std::string path = packFmu("two", modelDescription());
    Fmi2Slave a(path, "a", quietLogger());
    Fmi2Slave b(path, "b", quietLogger());
    a.initialize(0.0);
    b.initialize(0.0);
    a.setReal("Boat.propeller.rpm", 10.0);
    a.doStep(0.0, 1.0);
    b.doStep(0.0, 1.0);
    EXPECT_DOUBLE_EQ(a.getReal("Boat.x"), 1.0);
    EXPECT_DOUBLE_EQ(b.getReal("Boat.x"), 0.0);
}

TEST(Fmi2SlaveTest, RejectsWhatItCannotRun)
{
    EXPECT_THROW(Fmi2Slave("/nonexistent.fmu", "a", quietLogger()), FmuError);
    EXPECT_THROW(
        Fmi2Slave(packFmu("v1", modelDescription("1.0")), "a", quietLogger()),
        FmuError);
    EXPECT_THROW(
        Fmi2Slave(
            packFmu("me", modelDescription("2.0", false)),
            "a",
            quietLogger()),
        FmuError);
    Fmi2Slave fmu(packFmu("vars", modelDescription()), "a", quietLogger());
    EXPECT_THROW(fmu.setReal("no_such_input", 1.0), FmuError);
    EXPECT_THROW(fmu.setReal("Boat.reset_state", 1.0), FmuError);  // a Boolean
    EXPECT_THROW(fmu.setFromString("gain", "fast"), FmuError);
}

TEST(Fmi2SlaveTest, UnpackedDirectoryIsRemovedWithTheInstance)
{
    const std::string path = packFmu("cleanup", modelDescription());
    auto unpacked = [] {
        int n = 0;
        for (const auto& e :
             fs::directory_iterator(fs::temp_directory_path())) {
            n += e.path().filename().string().rfind("lotusim-fmu-", 0) == 0;
        }
        return n;
    };
    const int before = unpacked();
    {
        Fmi2Slave fmu(path, "a", quietLogger());
        EXPECT_EQ(unpacked(), before + 1);
    }
    EXPECT_EQ(unpacked(), before);
}

TEST_F(FmuInterfaceTest, StepsTheVesselInGazeboFrame)
{
    const gz::sim::Entity entity = 7;
    ASSERT_TRUE(interface->configureInterface(
        entity,
        "boat",
        domainSdf(
            packFmu("iface", modelDescription()),
            "<parameter name=\"gain\">0.5</parameter>"),
        DomainType::Surface));
    ASSERT_TRUE(interface->activateInterface(entity, DomainType::Surface));

    // 60 rpm = 2*pi rad/s, u = 0.5 * 2*pi = pi m/s.
    (*commands)[entity] = R"j({"propeller(rpm)": 60})j";
    VesselInformation start;  // Gazebo, at the origin, facing east (ENU x)
    start.time = 3.0;
    const auto result = interface->getNewState(entity, start, 1000.0f);
    ASSERT_TRUE(result.has_value());
    const auto& [state, domain] = *result;
    EXPECT_EQ(domain, DomainType::Surface);
    EXPECT_NEAR(state.pose.Pos().X(), M_PI, 1e-9);  // east
    EXPECT_NEAR(state.pose.Pos().Y(), 0.0, 1e-9);
    EXPECT_NEAR(state.pose.Pos().Z(), 0.0, 1e-9);
    EXPECT_NEAR(state.lin_vel.X(), M_PI, 1e-9);
    EXPECT_DOUBLE_EQ(state.time, 4.0);
    EXPECT_EQ(state.entity, entity);
}

TEST_F(FmuInterfaceTest, GazeboStateIsImposedEveryStep)
{
    const gz::sim::Entity entity = 8;
    ASSERT_TRUE(interface->configureInterface(
        entity,
        "boat",
        domainSdf(packFmu("teleport", modelDescription())),
        DomainType::Surface));
    ASSERT_TRUE(interface->activateInterface(entity, DomainType::Surface));
    (*commands)[entity] = R"j({"propeller(rpm)": 0})j";

    // Moved by Gazebo, twice: reset_state must go from false to true again.
    for (const double east : {100.0, -30.0}) {
        VesselInformation teleported;
        teleported.pose = gz::math::Pose3d(east, 50, 0, 0, 0, 0);
        const auto result = interface->getNewState(entity, teleported, 200.0f);
        ASSERT_TRUE(result.has_value());
        EXPECT_NEAR(std::get<0>(*result).pose.Pos().X(), east, 1e-9);
        EXPECT_NEAR(std::get<0>(*result).pose.Pos().Y(), 50.0, 1e-9);
    }
}

TEST_F(FmuInterfaceTest, WithoutResetInputsTheFmuKeepsItsState)
{
    const gz::sim::Entity entity = 9;
    ASSERT_TRUE(interface->configureInterface(
        entity,
        "boat",
        domainSdf(
            packFmu("noreset", modelDescription("2.0", true, true, false))),
        DomainType::Surface));
    ASSERT_TRUE(interface->activateInterface(entity, DomainType::Surface));
    VesselInformation teleported;
    teleported.pose = gz::math::Pose3d(100, 50, 0, 0, 0, 0);
    const auto result = interface->getNewState(entity, teleported, 200.0f);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(std::get<0>(*result).pose.Pos().X(), 0.0, 1e-9);
}

TEST_F(FmuInterfaceTest, UnknownCommandsAreIgnored)
{
    const gz::sim::Entity entity = 10;
    ASSERT_TRUE(interface->configureInterface(
        entity,
        "boat",
        domainSdf(packFmu("unknown", modelDescription())),
        DomainType::Surface));
    ASSERT_TRUE(interface->activateInterface(entity, DomainType::Surface));
    (*commands)[entity] = R"j({"rudder(beta)": 0.3, "propeller(rpm)": 60})j";
    VesselInformation start;
    const auto result = interface->getNewState(entity, start, 1000.0f);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(std::get<0>(*result).pose.Pos().X(), 0.1 * 2 * M_PI, 1e-9);
}

TEST_F(FmuInterfaceTest, AnInputCanBeCommandedByItsOwnName)
{
    const gz::sim::Entity entity = 13;
    ASSERT_TRUE(interface->configureInterface(
        entity,
        "boat",
        domainSdf(packFmu("byname", modelDescription())),
        DomainType::Surface));
    ASSERT_TRUE(interface->activateInterface(entity, DomainType::Surface));
    // The FMU's own name, in its own unit (rad/s): no rpm conversion.
    (*commands)[entity] = R"j({"Boat.propeller.rpm": 10})j";
    VesselInformation start;
    const auto result = interface->getNewState(entity, start, 1000.0f);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(std::get<0>(*result).pose.Pos().X(), 1.0, 1e-9);
}

TEST_F(FmuInterfaceTest, ThrustersAreSeededAsForXdyn)
{
    // <thrusters> written as in the worlds: <thruster1>propeller</thruster1>.
    const std::string text =
        "<sdf version='1.9'><model name='m'><link name='l'/><plugin name='p' filename='f'><surface>"
        "<interface_type>FMU</interface_type><fmu>" +
        packFmu("seed", modelDescription()) +
        "</fmu>"
        "<thrusters><thruster1>propeller</thruster1></thrusters></surface></plugin></model></sdf>";
    auto root = std::make_shared<sdf::SDF>();
    sdf::init(root);
    sdf::readString(text, root);
    const auto sdf =
        root->Root()->GetElement("model")->GetElement("plugin")->GetElement(
            "surface");
    ASSERT_TRUE(
        interface->configureInterface(12, "boat", sdf, DomainType::Surface));
    // Only the FMU's own inputs: propeller(P/D) and (beta) do not exist here.
    EXPECT_EQ(
        nlohmann::json::parse((*commands)[12]),
        nlohmann::json::parse(R"j({"propeller(rpm)": 50.0})j"));
}

TEST_F(FmuInterfaceTest, ConfigurationFailures)
{
    EXPECT_FALSE(interface->configureInterface(
        1,
        "boat",
        domainSdf("/nonexistent.fmu"),
        DomainType::Surface));
    EXPECT_FALSE(interface->configureInterface(
        1,
        "boat",
        domainSdf(packFmu("noqk", modelDescription("2.0", true, false))),
        DomainType::Surface));
    EXPECT_FALSE(interface->configureInterface(
        1,
        "boat",
        domainSdf(packFmu("unknown_domain", modelDescription())),
        DomainType::Unknown));
    EXPECT_FALSE(interface->configureInterface(
        1,
        "boat",
        domainSdf(
            packFmu("badparam", modelDescription()),
            "<parameter name=\"nope\">1</parameter>"),
        DomainType::Surface));
}

TEST_F(FmuInterfaceTest, OnlyTheActiveDomainSteps)
{
    const gz::sim::Entity entity = 11;
    const std::string path = packFmu("domains", modelDescription());
    ASSERT_TRUE(interface->configureInterface(
        entity,
        "sub",
        domainSdf(path),
        DomainType::Surface));
    ASSERT_TRUE(interface->configureInterface(
        entity,
        "sub",
        domainSdf(path),
        DomainType::Underwater));
    VesselInformation start;
    EXPECT_FALSE(interface->getNewState(entity, start, 200.0f)
                     .has_value());  // none active
    ASSERT_TRUE(interface->activateInterface(entity, DomainType::Underwater));
    EXPECT_TRUE(interface->getNewState(entity, start, 200.0f).has_value());
    EXPECT_NE(interface->getURI(entity).find(path), std::string::npos);
    ASSERT_TRUE(interface->deactivateInterface(entity, DomainType::Underwater));
    EXPECT_FALSE(interface->getNewState(entity, start, 200.0f).has_value());
    ASSERT_TRUE(interface->removeInterface(entity));
    EXPECT_FALSE(interface->activateInterface(entity, DomainType::Surface));
}

TEST(FmuPathTest, RelativePathsResolveUnderTheModelPath)
{
    const fs::path root = fs::temp_directory_path() / "lotusim-test-models";
    fs::create_directories(root / "boat");
    std::ofstream(root / "boat" / "boat.fmu") << "x";
    setenv("LOTUSIM_MODELS_PATH", root.c_str(), 1);
    EXPECT_EQ(
        FmuInterface::resolveFmuPath("boat/boat.fmu"),
        (root / "boat" / "boat.fmu").string());
    EXPECT_EQ(FmuInterface::resolveFmuPath("boat/none.fmu"), "");
    EXPECT_EQ(
        FmuInterface::resolveFmuPath((root / "boat" / "boat.fmu").string()),
        (root / "boat" / "boat.fmu").string());
    unsetenv("LOTUSIM_MODELS_PATH");
    fs::remove_all(root);
}

// Custom main, as in test_vessel_information.cpp: gz-sim8's dependency chain
// segfaults during static teardown inside the nix build sandbox.
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    std::_Exit(result);
}
