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
#include <tinyxml2.h>

#include <cstdlib>
#include <string>

#include "lotusim_mas/scenario_parser.hpp"

using lotusim::scenario::ScenarioParser;

namespace {

std::string lotusParam(const std::string& uri, const std::string& thruster)
{
    const auto cfg = ScenarioParser::fromString(R"(
agents:
  boat:
    model: lrauv
    render_interface: {renderer_type: "a&b"}
    physics_interface:
      init_domain: underwater
      domains:
        - domain: underwater
          interface_type: XDynWebSocket
          interface_params:
            uri: ")" + uri + R"("
            thrusters: [{name: ")" + thruster + R"("}]
)");
    EXPECT_TRUE(cfg);
    if (!cfg || cfg->agents.empty()) {
        return "";
    }
    return cfg->agents.front().lotus_param;
}

}  // namespace

// An entity such as &amp; in a uri used to be decoded (the uri came back as
// "a&b"). A bare & is tolerated by tinyxml2, so it would not catch this.
TEST(ScenarioEscaping, EntityInUriKeptLiteral)
{
    const auto xml = lotusParam("ws://host:12346/?q=a&amp;b", "propeller");
    tinyxml2::XMLDocument doc;
    ASSERT_EQ(doc.Parse(xml.c_str()), tinyxml2::XML_SUCCESS) << xml;
    const auto* uri = doc.FirstChildElement("lotus_param")
                          ->FirstChildElement("physics_engine_interface")
                          ->FirstChildElement("underwater")
                          ->FirstChildElement("uri");
    ASSERT_NE(uri, nullptr);
    EXPECT_STREQ(uri->GetText(), "ws://host:12346/?q=a&amp;b");
}

// A < in a name used to let a scenario inject elements.
TEST(ScenarioEscaping, NoElementInjection)
{
    const auto xml = lotusParam(
        "ws://127.0.0.1:12346",
        "p</thrusters1><init_state>Aerial</init_state><thrusters1>");
    tinyxml2::XMLDocument doc;
    ASSERT_EQ(doc.Parse(xml.c_str()), tinyxml2::XML_SUCCESS) << xml;
    const auto* physics = doc.FirstChildElement("lotus_param")
                              ->FirstChildElement("physics_engine_interface");
    const auto* init = physics->FirstChildElement("init_state");
    ASSERT_NE(init, nullptr);
    EXPECT_STREQ(init->GetText(), "Underwater");
    EXPECT_EQ(init->NextSiblingElement("init_state"), nullptr);
}

TEST(ScenarioEscaping, RendererTypeEscaped)
{
    const auto xml = lotusParam("ws://127.0.0.1:12346", "propeller");
    EXPECT_NE(
        xml.find("<renderer_type_name>a&amp;b</renderer_type_name>"),
        std::string::npos);
}

// gz-sim's dependency chain can crash during static teardown (see
// physics_engine_interface's tests), so exit without running it.
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    const int result = RUN_ALL_TESTS();
    std::_Exit(result);
}
