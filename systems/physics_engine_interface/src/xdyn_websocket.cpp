/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "physics_engine_interface/xdyn_websocket.hpp"

#include <cmath>

#include "lotusim_common/common.hpp"

namespace lotusim::gazebo {

std::shared_ptr<XdynWebsocket> XdynWebsocket::m_instance = nullptr;
std::mutex XdynWebsocket::m_instance_mutex;
std::mutex XdynWebsocket::m_variable_mutex;
std::unordered_map<gz::sim::Entity, std::string> XdynWebsocket::m_name_mapping;
std::unordered_map<std::string, gz::sim::Entity>
    XdynWebsocket::m_entity_mapping;
std::unordered_map<gz::sim::Entity, std::unordered_map<DomainType, std::string>>
    XdynWebsocket::m_uri;
std::unordered_map<gz::sim::Entity, Client::connection_ptr>
    XdynWebsocket::m_connection_mapping;
std::unordered_map<Client::connection_ptr, gz::sim::Entity>
    XdynWebsocket::m_connection_entity_mapping;
std::unordered_map<gz::sim::Entity, std::string> XdynWebsocket::m_status;
std::unordered_map<gz::sim::Entity, std::mutex> XdynWebsocket::m_msg_mutex;
std::unordered_map<gz::sim::Entity, std::condition_variable>
    XdynWebsocket::m_msg_cv;
std::unordered_map<gz::sim::Entity, bool> XdynWebsocket::m_msg_ready;
std::unordered_map<gz::sim::Entity, bool> XdynWebsocket::m_msg_ok;
std::unordered_map<gz::sim::Entity, VesselInformation>
    XdynWebsocket::m_saved_state;
std::unordered_map<gz::sim::Entity, DomainType> XdynWebsocket::m_current_domain;
std::unordered_map<gz::sim::Entity, double> XdynWebsocket::m_surface_depth;
std::unordered_map<gz::sim::Entity, std::shared_ptr<GaussMarkovCurrent>>
    XdynWebsocket::m_gauss_markov_current;
std::unordered_map<gz::sim::Entity, std::shared_ptr<CopernicusCurrent>>
    XdynWebsocket::m_copernicus_current;


XdynWebsocket::XdynWebsocket() : PhysicsInterfaceBase("XdynWebsocket")
{
    m_client.clear_access_channels(websocketpp::log::alevel::all);
    m_client.clear_error_channels(websocketpp::log::elevel::all);
    m_client.init_asio();
    m_client.start_perpetual();

    m_thread = Weblib::make_shared<Weblib::thread>(&Client::run, &m_client);

    m_engine_logger->debug("t,model_name,X,Y,Z,U,V,W,qi,qj,qk,qr,P,Q,R");
}

XdynWebsocket::~XdynWebsocket()
{
    m_client.stop_perpetual();
    websocketpp::lib::error_code ec;
    for (auto&& i : m_connection_mapping) {
        m_client
            .close(i.second, websocketpp::close::status::going_away, "", ec);
    }
    m_client.stop();
    m_thread->join();
}

std::shared_ptr<XdynWebsocket> XdynWebsocket::createInterface()
{
    std::scoped_lock lock(m_instance_mutex);
    if (m_instance == nullptr) {
        m_instance = std::make_shared<XdynWebsocket>();
    }
    return m_instance;
}

bool XdynWebsocket::configureInterface(
    const gz::sim::Entity& _entity,
    const std::string& _name,
    const sdf::ElementPtr _sdf,
    const DomainType& domain_type)
{
    std::unique_lock<std::mutex> lock(m_variable_mutex);

    if (domain_type == DomainType::Unknown) {
        m_logger->error(
            "XdynWebsocket::configureInterface: Model {} Called configure interface without a domain specified. There will not be physics update for this domain",
            _name);
        return false;
    }

    m_name_mapping[_entity] = _name;
    m_entity_mapping[_name] = _entity;

    std::string uri;
    if (_sdf->HasElement("uri")) {
        uri = _sdf->Get<std::string>("uri");
    } else {
        m_logger->error(
            "XdynWebsocket::configureInterface: No uri for {}",
            _name);
        return false;
    }
    m_uri[_entity][domain_type] = uri;

    // Surface/Underwater threshold: a vehicle property, not a universal one.
    m_surface_depth[_entity] = _sdf->HasElement("surface_depth")
        ? _sdf->Get<double>("surface_depth")
        : DEFAULT_SURFACE_DEPTH;

    // Optional Gauss-Markov current, injected on the caller side (see
    // gauss_markov_current.hpp) -- absent by default, since xdyn vessels
    // normally take their current from their own YAML environment model.
    if (_sdf->HasElement("gauss_markov_current")) {
        auto gm_sdf = _sdf->GetElement("gauss_markov_current");
        const double mean_x = gm_sdf->HasElement("mean_x")
            ? gm_sdf->Get<double>("mean_x") : 0.0;
        const double mean_y = gm_sdf->HasElement("mean_y")
            ? gm_sdf->Get<double>("mean_y") : 0.0;
        const double tau = gm_sdf->HasElement("tau")
            ? gm_sdf->Get<double>("tau") : 60.0;
        // Stationary standard deviation, not the driving-noise intensity:
        // directly comparable to the mean, and its meaning does not shift
        // when tau changes. See gauss_markov_current.hpp.
        const double std_dev = gm_sdf->HasElement("std_dev")
            ? gm_sdf->Get<double>("std_dev") : 0.0;
        const unsigned int seed = gm_sdf->HasElement("seed")
            ? gm_sdf->Get<unsigned int>("seed") : 0u;
        if (gm_sdf->HasElement("sigma")) {
            m_logger->warn(
                "XdynWebsocket::createConnection: {} declares <sigma> in "
                "<gauss_markov_current>, which is not a parameter of this model. "
                "Use <std_dev>, the stationary standard deviation in m/s "
                "(std_dev = sigma * sqrt(tau/2)). Ignoring <sigma>.",
                _name);
        }
        m_gauss_markov_current[_entity] = std::make_shared<GaussMarkovCurrent>(
            mean_x, mean_y, tau, std_dev, seed);
    }

    // Optional measured-current replay from a Copernicus depth profile (see
    // copernicus_current.hpp), the same injection slot as the Gauss-Markov
    // current above and equally absent by default. Declaring both would mean
    // two currents fighting over one slot, so that is refused rather than
    // silently resolved one way or the other.
    if (_sdf->HasElement("copernicus_current")) {
        auto cop_sdf = _sdf->GetElement("copernicus_current");
        if (m_gauss_markov_current.count(_entity)) {
            m_logger->error(
                "XdynWebsocket::createConnection: {} declares both "
                "<gauss_markov_current> and <copernicus_current>. They are "
                "alternative sources for the same injected current, not "
                "additive. Ignoring <copernicus_current>.",
                _name);
        } else if (!cop_sdf->HasElement("profile")) {
            m_logger->error(
                "XdynWebsocket::createConnection: {} declares "
                "<copernicus_current> without a <profile> path. No measured "
                "current will be applied.",
                _name);
        } else {
            const std::string profile = cop_sdf->Get<std::string>("profile");
            try {
                auto cur = std::make_shared<CopernicusCurrent>(profile);
                m_logger->info(
                    "XdynWebsocket::createConnection: {} replaying measured "
                    "current from '{}' ({} depth level(s)).",
                    _name, profile, cur->levels());
                m_copernicus_current[_entity] = std::move(cur);
            } catch (const std::exception& e) {
                // Loud, and with no current installed: a run that silently
                // fell back to zero current would be indistinguishable from
                // the no-current control condition.
                m_logger->error(
                    "XdynWebsocket::createConnection: {} could not load "
                    "<copernicus_current> profile: {}. NO current will be "
                    "applied to this vessel.",
                    _name, e.what());
            }
        }
    }

    // Pre-create every per-entity slot of the maps that send() and onMessage()
    // touch, while we are still on the single-threaded load path. Those two run
    // concurrently once the scenario has more than one xdyn model: Update fires
    // one std::async per model, and the websocket delivers replies on its own
    // thread. Reaching a missing key through operator[] INSERTS it, which can
    // rehash the map under the other threads walking it. The concurrent paths
    // must therefore use find(), never operator[].
    m_msg_mutex[_entity];
    m_msg_cv[_entity];
    m_msg_ready[_entity] = false;
    m_msg_ok[_entity] = false;
    m_saved_state[_entity] = VesselInformation{};

    // xdyn fails a whole step if any command declared by its force models is
    // missing, so the first step must already carry every key. A model whose
    // command is not a Wageningen propeller setting declares its own JSON.
    if (m_models_cmd_map_ptr && _sdf->HasElement("initial_commands")) {
        // Force-model agnostic: whatever JSON object the model needs, used
        // verbatim. Required by anything that is not Wageningen B-series,
        // e.g. `maneuvering` models whose command is a thrust in newtons.
        const std::string raw = _sdf->Get<std::string>("initial_commands");
        try {
            auto parsed = json::parse(raw);
            if (!parsed.is_object()) {
                throw std::runtime_error("not a JSON object");
            }
            (*m_models_cmd_map_ptr)[_entity] = parsed.dump();
        } catch (const std::exception& e) {
            m_logger->error(
                "XdynWebsocket::createConnection: {} has an invalid <initial_commands>: {}\n{}",
                _name,
                e.what(),
                raw);
            return false;
        }
    } else if (m_models_cmd_map_ptr) {
        auto initial_cmd = json::object();

        // Propellers: each <thrusters> entry seeds its rpm/pitch/beta signals.
        // A model with no thruster still has the tag, just with no child, so
        // the first child is tested before it is dereferenced.
        if (_sdf->HasElement("thrusters")) {
            auto sdfPtr_thruster =
                _sdf->GetElement("thrusters")->GetFirstElement();
            while (sdfPtr_thruster != sdf::ElementPtr(nullptr)) {
                std::string thruster_name = sdfPtr_thruster->Get<std::string>();
                // rpm, like every LOTUSim command: 50 rpm (5.2 rad/s) keeps the
                // Wageningen model inside its domain. Until the rpm-to-rad/s
                // conversion, this seed meant 50 rad/s (477 rpm).
                initial_cmd[thruster_name + "(rpm)"] = 50.0;
                initial_cmd[thruster_name + "(P/D)"] = 0.79;
                initial_cmd[thruster_name + "(beta)"] = 0.0;
                sdfPtr_thruster = sdfPtr_thruster->GetNextElement();
            }
        }

        // Angle-commanded actuators (rudders, sails, fins): each
        // <control_surfaces> entry is the full xdyn command signal,
        // e.g. "rudder(angle)" or "mainsail(sheet)", seeded to 0.0 so that
        // commands.at(<signal>) does not throw before the first ROS setpoint.
        // The default seeding from <thrusters> only does not cover these, so a
        // vessel driven solely by control surfaces would otherwise crash the
        // co-simulation on its first step.
        if (_sdf->HasElement("control_surfaces")) {
            auto sdfPtr_surface =
                _sdf->GetElement("control_surfaces")->GetFirstElement();
            while (sdfPtr_surface != sdf::ElementPtr(nullptr)) {
                std::string signal = sdfPtr_surface->Get<std::string>();
                initial_cmd[signal] = 0.0;
                sdfPtr_surface = sdfPtr_surface->GetNextElement();
            }
        }

        if (!initial_cmd.empty()) {
            (*m_models_cmd_map_ptr)[_entity] = initial_cmd.dump();
        } else {
            m_logger->warn(
                "XdynWebsocket::configureInterface: {} declares neither <thrusters>, "
                "<control_surfaces> nor <initial_commands>. Every step will fail until "
                "an agent publishes a complete command set on the command topic.",
                _name);
        }
    }
    return true;
}

bool XdynWebsocket::removeInterface(
    const gz::sim::Entity& _entity,
    const DomainType& domain_type)
{
    deactivateInterface(_entity);
    if (m_models_cmd_map_ptr->find(_entity) != m_models_cmd_map_ptr->end()) {
        m_models_cmd_map_ptr->erase(_entity);
    }
    m_uri.erase(_entity);
    return true;
}

bool XdynWebsocket::activateInterface(
    const gz::sim::Entity& _entity,
    const DomainType& domain_type)
{
    try {
        if (m_status.find(_entity) != m_status.end() &&
            (m_status[_entity] == "opened" ||
             m_status[_entity] == "configuring") &&
            m_current_domain.find(_entity) != m_current_domain.end() &&
            m_current_domain[_entity] == domain_type) {
            m_logger->warn(
                "XdynWebsocket::activateInterface: Called for vessel entity {} even when it is active",
                _entity);
            return true;
        }

        if (m_uri.find(_entity) == m_uri.end() ||
            m_uri[_entity].find(domain_type) == m_uri[_entity].end()) {
            m_logger->error(
                "XdynWebsocket::activateInterface: Called for vessel entity {} and domain {} but no uri found.",
                _entity,
                DomainTypeToStringMap[domain_type]);
            return false;
        }

        if (m_connection_mapping.find(_entity) != m_connection_mapping.end() &&
            m_current_domain.find(_entity) != m_current_domain.end() &&
            m_current_domain[_entity] != domain_type) {
            deactivateInterface(_entity, m_current_domain[_entity]);
        }

        websocketpp::lib::error_code ec;
        Client::connection_ptr con =
            m_client.get_connection(m_uri[_entity][domain_type], ec);
        if (ec) {
            m_logger->info(
                "XdynWebsocket::activateInterface: Connect initialization error: {}",
                ec.message());
            return false;
        }
        {
            std::unique_lock<std::mutex> lock(m_variable_mutex);
            m_connection_mapping.insert({_entity, con});
            m_connection_entity_mapping.insert({con, _entity});
            m_status.insert({_entity, "configuring"});
            m_current_domain[_entity] = domain_type;
        }
        con->set_open_handler(bind(
            &XdynWebsocket::onOpen,
            this,
            _entity,
            websocketpp::lib::placeholders::_1));
        con->set_fail_handler(bind(
            &XdynWebsocket::onFail,
            this,
            _entity,
            websocketpp::lib::placeholders::_1));
        con->set_message_handler(bind(
            &XdynWebsocket::onMessage,
            this,
            websocketpp::lib::placeholders::_1,
            websocketpp::lib::placeholders::_2));

        // Poll for the open handler rather than sleeping a flat interval per
        // try. A local xdyn answers in a few milliseconds, and this call sits
        // on Gazebo's Update thread, so a blocking wait stalls the whole
        // simulation at every domain transition.
        constexpr auto kPoll = std::chrono::milliseconds(10);
        constexpr auto kAttemptTimeout = std::chrono::seconds(3);
        auto status = [&]() {
            std::unique_lock<std::mutex> lock(m_variable_mutex);
            return m_status[_entity];
        };
        for (int retry = 0; retry < 3; ++retry) {
            m_logger->info(
                "XdynWebsocket::activateInterface: Starting connection: {}",
                m_name_mapping[_entity]);
            m_client.connect(con);
            const auto deadline =
                std::chrono::steady_clock::now() + kAttemptTimeout;
            std::string current_status = status();
            while (current_status != "opened" && current_status != "failed" &&
                   std::chrono::steady_clock::now() < deadline) {
                std::this_thread::sleep_for(kPoll);
                current_status = status();
            }
            if (current_status == "opened") {
                return true;
            }
        }
        m_logger->error(
            "XdynWebsocket::activateInterface: Called for vessel entity {} but unable to connect.",
            _entity);
        return false;
    } catch (const std::exception& e) {
        m_logger->error(
            "XdynWebsocket::activateInterface: Called for vessel entity {} but error\n{}.",
            _entity,
            e.what());

    } catch (...) {
        m_logger->error(
            "XdynWebsocket::activateInterface: Called for vessel entity {} but unkown error.",
            _entity);
    }
    return false;
}

bool XdynWebsocket::deactivateInterface(
    const gz::sim::Entity& _entity,
    const DomainType& domain_type)
{
    std::unique_lock<std::mutex> lock(m_variable_mutex);
    websocketpp::lib::error_code ec;
    m_logger->info(
        "XdynWebsocket::deactivateInterface: Deactivating connection for entity {}",
        _entity);
    if (m_connection_mapping.find(_entity) != m_connection_mapping.end() &&
        m_current_domain.find(_entity) != m_current_domain.end() &&
        (m_current_domain[_entity] == domain_type ||
         domain_type == DomainType::Unknown)) {
        m_client.close(
            m_connection_mapping[_entity]->get_handle(),
            websocketpp::close::status::going_away,
            "",
            ec);
        m_connection_entity_mapping.erase(m_connection_mapping[_entity]);
        m_connection_mapping.erase(_entity);
        m_status[_entity] = "closed";
    } else {
        m_logger->warn(
            "XdynWebsocket::deactivateInterface: No existing connection for domain {} for entity {} found.",
            DomainTypeToStringMap[domain_type],
            _entity);
    }
    return true;
}

std::string XdynWebsocket::getURI(
    const gz::sim::Entity& _entity,
    const DomainType& domain_type)
{
    std::unique_lock<std::mutex> lock(m_variable_mutex);
    if (m_uri.find(_entity) != m_uri.end() &&
        m_uri[_entity].find(domain_type) != m_uri[_entity].end()) {
        return m_uri[_entity][domain_type];
    } else {
        return "";
    }
}

void XdynWebsocket::onOpen(
    const gz::sim::Entity& _entity,
    websocketpp::connection_hdl hdl)
{
    std::unique_lock<std::mutex> lock(m_variable_mutex);
    auto uri = m_client.get_con_from_hdl(hdl)->get_uri()->str();
    m_status[_entity] = "opened";
    m_logger->info("XdynWebsocket::onOpen: Opened {}", uri);
}

void XdynWebsocket::onFail(
    const gz::sim::Entity& _entity,
    websocketpp::connection_hdl hdl)
{
    std::unique_lock<std::mutex> lock(m_variable_mutex);
    auto uri = m_client.get_con_from_hdl(hdl)->get_uri()->str();
    m_status[_entity] = "failed";
    m_logger->info("XdynWebsocket::onFail: Failed {}", uri);
}

void XdynWebsocket::onMessage(
    websocketpp::connection_hdl hdl,
    websocketpp::config::asio_client::message_type::ptr msg)
{
    gz::sim::Entity entity =
        m_connection_entity_mapping[m_client.get_con_from_hdl(hdl)];

    auto mutex_it = m_msg_mutex.find(entity);
    auto ready_it = m_msg_ready.find(entity);
    auto cv_it = m_msg_cv.find(entity);
    auto ok_it = m_msg_ok.find(entity);
    if (mutex_it == m_msg_mutex.end() || ready_it == m_msg_ready.end() ||
        cv_it == m_msg_cv.end() || ok_it == m_msg_ok.end()) {
        m_logger->error(
            "XdynWebsocket::onMessage: no message slots for entity {}.",
            entity);
        return;
    }

    // Every early return below still wakes send(), with m_msg_ok left false: a
    // reply that cannot be used fails the step at once instead of stalling it
    // for the whole websocket timeout, and m_saved_state keeps its last value.
    std::unique_lock<std::mutex> lock(mutex_it->second);
    auto release_waiter = [&]() {
        ready_it->second = true;
        cv_it->second.notify_one();
    };

    json reply;
    try {
        reply = json::parse(msg->get_payload());
    } catch (const json::parse_error& e) {
        m_logger->error("XdynWebsocket::onMessage: Failed to parse xdyn reply: {}", e.what());
        release_waiter();
        return;
    }

    // xdyn reports a refused step as an "error" field instead of a state.
    if (reply.contains("error")) {
        m_logger->error(
            "XdynWebsocket::onMessage: xdyn refused the step for entity {}: {}",
            entity,
            reply["error"].dump());
        release_waiter();
        return;
    }

    // if xdyn can't simulate, it now logs and skip instead of crashing the websocket thread
    static const std::vector<std::string> required_fields = {
        "x", "y", "z", "qi", "qj", "qk", "qr",
        "u", "v", "w", "p", "q", "r", "t"};
    for (const auto& field : required_fields) {
        if (!reply.contains(field) || reply[field].empty()) {
            m_logger->error(
                "XdynWebsocket::onMessage: Malformed reply from xdyn for entity {}, missing or empty field '{}'. Skipping update.\nRaw reply: {}",
                entity,
                field,
                reply.dump());
            release_waiter();
            return;
        }
    }

    const gz::math::Vector3d ned_position{
        reply["x"].back().get<double>(),
        reply["y"].back().get<double>(),
        reply["z"].back().get<double>()};

    const gz::math::Quaterniond ned_quat{
        reply["qr"].back().get<double>(),
        reply["qi"].back().get<double>(),
        reply["qj"].back().get<double>(),
        reply["qk"].back().get<double>()};

    const gz::math::Vector3d ned_lin_vel{
        reply["u"].back().get<double>(),
        reply["v"].back().get<double>(),
        reply["w"].back().get<double>()};

    const gz::math::Vector3d ned_angular_vel{
        reply["p"].back().get<double>(),
        reply["q"].back().get<double>(),
        reply["r"].back().get<double>()};

    VesselInformation new_state = VesselInformation::from_xdyn(
        ned_position, ned_quat, ned_lin_vel, ned_angular_vel);
    new_state.time = reply["t"].back().get<double>();
    new_state.entity = entity;

    m_saved_state[entity] = std::move(new_state);
    ok_it->second = true;
    ready_it->second = true;
    cv_it->second.notify_one();
}

std::optional<std::tuple<VesselInformation, DomainType>>
XdynWebsocket::getNewState(
    const gz::sim::Entity& _entity,
    const VesselInformation& previous_state,
    float time_diff)
{
    const double dt_s = time_diff / 1000.0;

    // A plugin-side current: Gauss-Markov, or a replayed measured profile.
    gz::math::Vector3d current_enu(0.0, 0.0, 0.0);
    auto gm_it = m_gauss_markov_current.find(_entity);
    if (gm_it != m_gauss_markov_current.end()) {
        const auto [vc_x, vc_y] = gm_it->second->update(dt_s);
        current_enu = common::kWorldEnuNed * gz::math::Vector3d(vc_x, vc_y, 0.0);
    } else {
        // Measured-current replay uses the same slot (a vessel declares at
        // most one of the two, enforced in createConnection). Unlike the
        // Gauss-Markov process it is a function of DEPTH rather than of
        // elapsed time, so it is sampled at the vessel's current depth (NED,
        // positive downwards), which is what makes the replayed profile's
        // vertical shear act on the vehicle as it changes depth.
        auto cop_it = m_copernicus_current.find(_entity);
        if (cop_it != m_copernicus_current.end()) {
            const double depth = -previous_state.pose.Pos().Z();
            const auto [vc_x, vc_y] = cop_it->second->at(depth);
            current_enu =
                common::kWorldEnuNed * gz::math::Vector3d(vc_x, vc_y, 0.0);
        }
    }

    // xdyn is stepped on the velocity RELATIVE to that current.
    //
    // A depth-resolved current (`environment models: - model: ekman current`)
    // is NOT applied here: it belongs to the vessel's own YAML, and injecting
    // a second one from the plugin would double-count it. `current_enu`
    // above is zero unless this vessel specifically requested a plugin-side
    // current, which is never combined with a YAML current on the same run.
    VesselInformation relative_state = previous_state;
    relative_state.lin_vel = previous_state.lin_vel - current_enu;
    const VesselInformation previous_state_xdyn = relative_state.to_xdyn();

    json data = json::object();
    data["Dt"] = dt_s;
    data["states"] = json::array();
    json previous_state_json = {
        // Absolute sim time in seconds: xdyn uses states.back().t as the
        // integration start time, and time-dependent forcings (waves) need the
        // true clock. The raw step duration in ms sent here previously froze
        // xdyn's clock at ~Dt, which is only harmless on calm water.
        {"t", previous_state.time},
        {"x", previous_state_xdyn.pose.Pos().X()},
        {"y", previous_state_xdyn.pose.Pos().Y()},
        {"z", previous_state_xdyn.pose.Pos().Z()},
        {"qi", previous_state_xdyn.pose.Rot().X()},
        {"qj", previous_state_xdyn.pose.Rot().Y()},
        {"qk", previous_state_xdyn.pose.Rot().Z()},
        {"qr", previous_state_xdyn.pose.Rot().W()},
        {"u", previous_state_xdyn.lin_vel.X()},
        {"v", previous_state_xdyn.lin_vel.Y()},
        {"w", previous_state_xdyn.lin_vel.Z()},
        {"p", previous_state_xdyn.ang_vel.X()},
        {"q", previous_state_xdyn.ang_vel.Y()},
        {"r", previous_state_xdyn.ang_vel.Z()}};
    data["states"].push_back(previous_state_json);

    if (m_models_cmd_map_ptr->find(_entity) != m_models_cmd_map_ptr->end()) {
        data["commands"] = json::parse((*m_models_cmd_map_ptr)[_entity]);
        // xdyn reads command values in SI units, whatever the signal is named:
        // "(rpm)" is taken as rad/s. LOTUSim speaks rpm, so convert here, once.
        // Unconverted, propellers spun ~9.5x too fast.
        for (auto& [key, value] : data["commands"].items()) {
            if (value.is_number() && key.size() >= 5 &&
                key.compare(key.size() - 5, 5, "(rpm)") == 0) {
                value = value.get<double>() * 2.0 * M_PI / 60.0;
            }
        }
    }

    data["requested_output"] = json::array();
    std::string msg_string = data.dump();

    if (send(_entity, msg_string)) {
        // Integrate the pose from the velocity rather than keeping the x, y, z
        // xdyn returned. xdyn is stepped from the state sent above and is
        // deliberately kept unaware of a plugin-side current, so the position it
        // returns advances on the RELATIVE velocity and drops that current's
        // contribution for the step. Integrating here keeps one rule for both
        // current sources. Trapezoidal, second order over the step.
        auto state_it = m_saved_state.find(_entity);
        if (state_it == m_saved_state.end()) {
            return std::nullopt;
        }

        // xdyn was sent the velocity RELATIVE to current_enu: add it back once,
        // so both the state handed out and the integration below see the true
        // ground velocity. Zero unless this model declared a plugin-side
        // current, so this is a no-op for the common case.
        state_it->second.lin_vel += current_enu;

        state_it->second.pose.Pos() = previous_state.pose.Pos() +
            0.5 * (previous_state.lin_vel + state_it->second.lin_vel) * dt_s;

        // ENU altitude, positive up, taken from the state just integrated.
        const double z = state_it->second.pose.Pos().Z();
        const double surface_depth = m_surface_depth.count(_entity)
            ? m_surface_depth.at(_entity)
            : DEFAULT_SURFACE_DEPTH;
        if (z >= surface_depth) {
            return std::make_tuple(state_it->second, DomainType::Aerial);
        } else if (z <= -surface_depth) {
            return std::make_tuple(state_it->second, DomainType::Underwater);
        } else {
            return std::make_tuple(state_it->second, DomainType::Surface);
        }
    }
    return std::nullopt;
}

bool XdynWebsocket::send(
    const gz::sim::Entity& _entity,
    const std::string& message)
{
    // Look the per-entity slots up, never operator[]: this runs on one
    // std::async thread per model, and inserting here would rehash the map
    // under the others (see configureInterface, where they are pre-created).
    auto mutex_it = m_msg_mutex.find(_entity);
    auto ready_it = m_msg_ready.find(_entity);
    auto cv_it = m_msg_cv.find(_entity);
    auto ok_it = m_msg_ok.find(_entity);
    if (mutex_it == m_msg_mutex.end() || ready_it == m_msg_ready.end() ||
        cv_it == m_msg_cv.end() || ok_it == m_msg_ok.end()) {
        m_logger->error(
            "XdynWebsocket::send: no message slots for entity {}.",
            _entity);
        return false;
    }

    // Take the message lock BEFORE sending and clear the ready flag: a reply
    // can come back on the websocket thread faster than we reach the wait, and
    // a plain wait_for() would then miss the notification and burn the full
    // timeout. Holding the lock across the send makes onMessage queue up behind
    // us until wait_for releases it.
    std::unique_lock<std::mutex> msg_lock(mutex_it->second);
    ready_it->second = false;
    ok_it->second = false;

    {
        std::unique_lock<std::mutex> lock(m_variable_mutex);
        // The connection is absent until activateInterface has succeeded, and
        // is erased again on deactivate; operator[] would hand back a default
        // null pointer and dereferencing it segfaults.
        auto con_it = m_connection_mapping.find(_entity);
        if (con_it == m_connection_mapping.end() || !con_it->second) {
            m_logger->warn("Websocket connection not ready, skipping send.");
            return false;
        }
        websocketpp::lib::error_code ec;
        m_client.send(
            con_it->second->get_handle(),
            message,
            websocketpp::frame::opcode::text,
            ec);
        if (ec) {
            m_logger->error(
                "XdynWebsocket::send: Error sending message: {}",
                ec.message());
            return false;
        }
    }

    if (!cv_it->second.wait_for(
            msg_lock,
            std::chrono::seconds(DEFAULT_WEBSOCKET_TIMEOUT),
            [&] { return ready_it->second; })) {
        m_logger->warn("XdynWebsocket::send: websocket timed out.");
        return false;
    }
    return ok_it->second;
}
}  // namespace lotusim::gazebo