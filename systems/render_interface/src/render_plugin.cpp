/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "render_interface/render_plugin.hpp"

namespace lotusim::gazebo {

RenderPlugin::RenderPlugin() {}

RenderPlugin::~RenderPlugin()
{
    m_logger->info(
        "RenderPlugin::~RenderPlugin: RenderPlugin successfully shutdown.");
}

void RenderPlugin::Configure(
    const gz::sim::Entity&,
    const std::shared_ptr<const sdf::Element>& _sdf,
    gz::sim::EntityComponentManager& _ecm,
    gz::sim::EventManager&)
{
    m_world_name = lotusim::common::getWorldName(_ecm);
    m_logger = logger::createConsoleAndFileLogger(
        "render_plugin",
        m_world_name + "_render_plugin.txt");

    auto sdfPtr = const_cast<sdf::Element*>(_sdf.get());

    std::string connection_protocol;
    if (sdfPtr->HasElement("connection_protocol")) {
        connection_protocol = sdfPtr->Get<std::string>("connection_protocol");
        m_logger->info(
            "RenderPlugin::Configure: Creating connection: {}",
            connection_protocol);
    } else {
        m_logger->error(
            "RenderPlugin::Configure: Connection protocol not found");
        return;
    }

    m_render_interface =
        CreateRenderInterface(connection_protocol, m_world_name, m_logger);
    m_render_interface->configureInterface(_sdf);

    // Cap the wall-clock rate at which poses are pushed to the renderer so a
    // high real_time_factor does not flood the bridge (see m_render_min_period_s).
    double render_rate_hz = 60.0;
    if (sdfPtr->HasElement("render_rate_hz")) {
        render_rate_hz = sdfPtr->Get<double>("render_rate_hz");
    }
    m_render_min_period_s =
        render_rate_hz > 0.0 ? 1.0 / render_rate_hz : 0.0;
    m_logger->info(
        "RenderPlugin::Configure: render send rate capped at {} Hz (wall).",
        render_rate_hz);

    m_logger->info("RenderPlugin::Configure: RenderPlugin started.");
}

void RenderPlugin::PreUpdate(
    const gz::sim::UpdateInfo& _info,
    gz::sim::EntityComponentManager& _ecm)
{
    m_render_interface->customPreUpdates(_info, _ecm);

    // Note: Once the callback return false, the callback is never called again.
    _ecm.EachNew<
        gz::sim::components::ModelSdf,
        gz::sim::components::ParentEntity>(
        [&](const gz::sim::Entity& _entity,
            const gz::sim::components::ModelSdf* _model,
            const gz::sim::components::ParentEntity*) -> bool {
            sdf::Model data = _model->Data();
            sdf::ElementPtr sdfptr = data.Element();

            auto includeptr = sdfptr->GetIncludeElement();
            // The lotus param will either be include statement or part of the
            // model
            if (!includeptr) {
                includeptr = sdfptr;
            }
            if (includeptr->HasElement("lotus_param") &&
                includeptr->GetElement("lotus_param")
                    ->HasElement("render_interface") &&
                includeptr->GetElement("lotus_param")
                    ->GetElement("render_interface")
                    ->HasElement("publish_render") &&
                includeptr->GetElement("lotus_param")
                    ->GetElement("render_interface")
                    ->Get<bool>("publish_render")) {
                auto name_opt =
                    _ecm.Component<gz::sim::components::Name>(_entity);
                if (name_opt) {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    m_vessel_entity[name_opt->Data()] = _entity;
                } else {
                    m_logger->warn(
                        "RenderPlugin::PreUpdate: Vessel entity {} name not found. Not publishing rendering.",
                        _entity);
                    return true;
                }
                m_logger->info(
                    "RenderPlugin::PreUpdate: Creation detected of vessel named {}",
                    name_opt->Data());
                gz::math::Pose3d pose =
                    _ecm.Component<gz::sim::components::Pose>(_entity)->Data();
                return m_render_interface->createVessel(
                    name_opt->Data(),
                    pose,
                    includeptr->GetElement("lotus_param")
                        ->GetElement("render_interface"));
            }
            return true;
        });
    _ecm.EachRemoved<
        gz::sim::components::ModelSdf,
        gz::sim::components::ParentEntity>(
        [&](const gz::sim::Entity& _entity,
            const gz::sim::components::ModelSdf*,
            const gz::sim::components::ParentEntity*) -> bool {
            auto name_opt = _ecm.Component<gz::sim::components::Name>(_entity);
            if (m_vessel_entity.find(name_opt->Data()) !=
                m_vessel_entity.end()) {
                m_logger->info(
                    "RenderPlugin::PreUpdate: Destruction detected of vessel named {}",
                    name_opt->Data());
                std::lock_guard<std::mutex> lock(m_mutex);
                m_vessel_entity.erase(name_opt->Data());
                return m_render_interface->destroyVessel(name_opt->Data());
            }
            return true;
        });
}

void RenderPlugin::PostUpdate(
    const gz::sim::UpdateInfo& _info,
    const gz::sim::EntityComponentManager& _ecm)
{
    if (!m_render_interface) {
        m_logger->info("RenderPlugin::PostUpdate: No render update");
        return;
    }

    // Wall-clock rate cap: skip the pose send if the last one was too recent,
    // so a high real_time_factor cannot flood the renderer bridge. customUpdates
    // still runs every step (unchanged).
    bool send_pose = true;
    if (m_render_min_period_s > 0.0) {
        auto now = std::chrono::steady_clock::now();
        if (m_render_started &&
            std::chrono::duration<double>(now - m_last_render_time).count() <
                m_render_min_period_s) {
            send_pose = false;
        } else {
            m_last_render_time = now;
            m_render_started = true;
        }
    }

    if (send_pose) {
        std::vector<std::pair<std::string, gz::math::Pose3d>> vessel_pose;
        // get a vector of vessel name, pose
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            for (auto&& entity : m_vessel_entity) {
                gz::math::Pose3d pose =
                    _ecm.Component<gz::sim::components::Pose>(entity.second)
                        ->Data();

                vessel_pose.push_back({entity.first, pose});
            }
        }
        m_render_interface->sendPosition(_info.simTime, vessel_pose);
    }
    m_render_interface->customUpdates(_info, _ecm);
}
}  // namespace lotusim::gazebo

GZ_ADD_PLUGIN(
    lotusim::gazebo::RenderPlugin,
    gz::sim::System,
    lotusim::gazebo::RenderPlugin::ISystemConfigure,
    lotusim::gazebo::RenderPlugin::ISystemPreUpdate,
    lotusim::gazebo::RenderPlugin::ISystemPostUpdate)