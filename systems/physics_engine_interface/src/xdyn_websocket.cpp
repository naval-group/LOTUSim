#include "physics_engine_interface/xdyn_websocket.hpp"

namespace lotusim::gazebo {

gz::math::Quaterniond quatNedToEnu(const gz::math::Quaterniond& q_ned)
{
    return q_ned_to_enu * q_ned * q_ned_to_enu.Inverse();
}

gz::math::Quaterniond quatEnuToNed(const gz::math::Quaterniond& q_enu)
{
    return q_ned_to_enu.Inverse() * q_enu * q_ned_to_enu;
}

gz::math::Vector3d vecNedToEnu(const gz::math::Vector3d& v_ned)
{
    return {v_ned.Y(), v_ned.X(), -v_ned.Z()};
}

gz::math::Vector3d vecEnuToNed(const gz::math::Vector3d& v_enu)
{
    return {v_enu.Y(), v_enu.X(), -v_enu.Z()};
}

std::shared_ptr<XdynWebsocket> XdynWebsocket::m_instance = nullptr;
std::mutex XdynWebsocket::m_mutex;
std::unordered_map<gz::sim::Entity, std::string> XdynWebsocket::m_name_mapping;
std::unordered_map<std::string, gz::sim::Entity>
    XdynWebsocket::m_entity_mapping;
std::unordered_map<gz::sim::Entity, std::string> XdynWebsocket::m_uri;
std::unordered_map<gz::sim::Entity, Client::connection_ptr>
    XdynWebsocket::m_connection_mapping;
std::unordered_map<Client::connection_ptr, gz::sim::Entity>
    XdynWebsocket::m_connection_entity_mapping;
std::unordered_map<gz::sim::Entity, std::string> XdynWebsocket::m_status;
std::unordered_map<gz::sim::Entity, std::mutex> XdynWebsocket::m_msg_mutex;
std::unordered_map<gz::sim::Entity, std::condition_variable>
    XdynWebsocket::m_msg_cv;
std::unordered_map<gz::sim::Entity, VesselInformation>
    XdynWebsocket::m_saved_state;

XdynWebsocket::XdynWebsocket() : PhysicsInterfaceBase("XdynWebsocket")
{
    m_client.clear_access_channels(websocketpp::log::alevel::all);
    m_client.clear_error_channels(websocketpp::log::elevel::all);
    m_client.init_asio();
    m_client.start_perpetual();

    // know how this thread is running. To determine whether this is a
    // bottleneck
    m_thread = Weblib::make_shared<Weblib::thread>(&Client::run, &m_client);

    // Header for excel file
    m_engine_logger->debug("t,vessel_name,X,Y,Z,U,V,W,qi,qj,qk,qr,p,q,r");
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

std::shared_ptr<XdynWebsocket> XdynWebsocket::getInstance(
    const gz::sim::Entity& _entity,
    const std::string& _name)
{
    m_name_mapping[_entity] = _name;
    m_entity_mapping[_name] = _entity;
    if (m_instance == nullptr) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_instance = std::shared_ptr<XdynWebsocket>(new XdynWebsocket());
    }
    return m_instance;
}

bool XdynWebsocket::createConnection(
    const gz::sim::Entity& _entity,
    const std::string& _name,
    const sdf::ElementPtr _sdf)
{
    std::string uri;
    if (_sdf->HasElement("uri")) {
        uri = _sdf->Get<std::string>("uri");
    } else {
        m_logger->error(
            "XdynWebsocket::createConnection: No uri for {}",
            _name);
        return false;
    }
    m_uri[_entity] = uri;

    // Need to create json file and add to the command
    if (_sdf->HasElement("thrusters") && m_vessels_cmd_map_ptr) {
        auto sdfPtr_thruster = _sdf->GetElement("thrusters")->GetFirstElement();
        auto thrusters_cmd = json::object();
        do {
            std::string thruster_name = sdfPtr_thruster->Get<std::string>();
            thrusters_cmd[thruster_name + "(rpm)"] = 0.01;
            thrusters_cmd[thruster_name + "(P/D)"] = 0.79;
            thrusters_cmd[thruster_name + "(beta)"] = 0.0;
            sdfPtr_thruster = sdfPtr_thruster->GetNextElement();
        } while (sdfPtr_thruster != sdf::ElementPtr(nullptr));
        (*m_vessels_cmd_map_ptr)[_entity] = thrusters_cmd.dump();
    }
    return true;
}

bool XdynWebsocket::activateConnection(const gz::sim::Entity& _entity)
{
    websocketpp::lib::error_code ec;

    Client::connection_ptr con = m_client.get_connection(m_uri[_entity], ec);
    if (ec) {
        m_logger->info(
            "XdynWebsocket::activateConnection: Connect initialization error: {}",
            ec.message());
        return -1;
    }
    m_connection_mapping.insert({_entity, con});
    m_connection_entity_mapping.insert({con, _entity});
    m_status.insert({_entity, "configuring"});
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

    int retry = 0;
    while (m_status[_entity] != "opened" && retry < 3) {
        m_logger->info(
            "XdynWebsocket::activateConnection: Starting connection: {}",
            m_name_mapping[_entity]);
        m_client.connect(con);
        // TBD: Damn dumb implementation. You are killing the thread.
        std::this_thread::sleep_for(std::chrono::seconds(3));
        retry += 1;
    }
    if (retry > 2) {
        throw std::runtime_error(
            "XdynWebsocket::activateConnection: Failed to activate websocket");
    }
    return true;
}

bool XdynWebsocket::deactivateConnection(const gz::sim::Entity& _entity)
{
    websocketpp::lib::error_code ec;
    m_client.close(
        m_connection_mapping[_entity]->get_handle(),
        websocketpp::close::status::going_away,
        "",
        ec);
    m_connection_entity_mapping.erase(m_connection_mapping[_entity]);
    m_connection_mapping.erase(_entity);
    m_status[_entity] = "closed";
    return true;
}

std::string XdynWebsocket::getURI(const gz::sim::Entity& _entity)
{
    return m_uri[_entity];
}

void XdynWebsocket::onOpen(
    const gz::sim::Entity& _entity,
    websocketpp::connection_hdl hdl)
{
    auto uri = m_client.get_con_from_hdl(hdl)->get_uri()->str();
    m_status[_entity] = "opened";
    m_logger->info("XdynWebsocket::onOpen: Opened {}", uri);
}

void XdynWebsocket::onFail(
    const gz::sim::Entity& _entity,
    websocketpp::connection_hdl hdl)
{
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
    std::unique_lock<std::mutex> lock(m_msg_mutex[entity]);
    json reply = json::parse(msg->get_payload());

    auto ned_position = gz::math::Vector3d(
        reply["x"].back().get<double>(),
        reply["y"].back().get<double>(),
        reply["z"].back().get<double>());

    auto ned_quad = gz::math::Quaterniond(
        reply["qr"].back().get<double>(),
        reply["qi"].back().get<double>(),
        reply["qk"].back().get<double>(),
        reply["qj"].back().get<double>());

    auto gz_position = vecNedToEnu(ned_position);
    auto gz_quad = quatNedToEnu(ned_quad);

    auto ned_lin_vel = gz::math::Vector3d{
        reply["u"].back().get<double>(),
        reply["v"].back().get<double>(),
        reply["w"].back().get<double>()};

    auto ned_angular_vel = gz::math::Vector3d(
        reply["p"].back().get<double>(),
        reply["q"].back().get<double>(),
        reply["r"].back().get<double>());

    auto gz_lin_vel = vecNedToEnu(ned_lin_vel);
    auto gz_angular_vel = vecNedToEnu(ned_angular_vel);

    VesselInformation new_state;
    new_state.time = reply["t"].back().get<double>();
    new_state.entity = entity;
    new_state.pose = gz::math::Pose3d(gz_position, gz_quad);
    new_state.lin_vel = gz_lin_vel;
    new_state.ang_vel = gz_angular_vel;

    m_saved_state[entity] = std::move(new_state);
    m_msg_cv[entity].notify_one();
}

std::optional<std::tuple<VesselInformation, DomainType>>
XdynWebsocket::getNewState(
    const gz::sim::Entity& _entity,
    const VesselInformation& previous_state,
    float time_diff)
{
    gz::math::Vector3d ned_position = vecEnuToNed(previous_state.pose.Pos());
    gz::math::Quaterniond ned_quad = quatEnuToNed(previous_state.pose.Rot());
    gz::math::Vector3d ned_lin_vel = vecEnuToNed(previous_state.lin_vel);
    gz::math::Vector3d ned_angular_vel = vecEnuToNed(previous_state.ang_vel);

    json data = json::object();
    data["Dt"] = time_diff / 1000.0;
    data["states"] = json::array();
    json previous_state_json = {
        {"t", time_diff},
        {"x", ned_position.X()},
        {"y", ned_position.Y()},
        {"z", ned_position.Z()},
        {"qi", ned_quad.X()},
        {"qj", ned_quad.Y()},
        {"qk", ned_quad.Z()},
        {"qr", ned_quad.W()},
        {"u", ned_lin_vel.X()},
        {"v", ned_lin_vel.Y()},
        {"w", ned_lin_vel.Z()},
        {"p", ned_angular_vel.X()},
        {"q", ned_angular_vel.Y()},
        {"r", ned_angular_vel.Z()}};
    data["states"].push_back(previous_state_json);

    if (m_vessels_cmd_map_ptr->find(_entity) != m_vessels_cmd_map_ptr->end()) {
        data["commands"] = json::parse((*m_vessels_cmd_map_ptr)[_entity]);
    }

    data["requested_output"] = json::array();
    std::string msg_string = data.dump();
    if (send(_entity, msg_string)) {
        return std::make_tuple(m_saved_state[_entity], DomainType::Surface);
    }
    return std::nullopt;
}

bool XdynWebsocket::send(
    const gz::sim::Entity& _entity,
    const std::string& message)
{
    websocketpp::lib::error_code ec;
    m_client.send(
        m_connection_mapping[_entity]->get_handle(),
        message,
        websocketpp::frame::opcode::text,
        ec);
    if (ec) {
        m_logger->error(
            "XdynWebsocket::send: Error sending message: ",
            ec.message());
        return false;
    }
    {
        std::unique_lock<std::mutex> lock(m_msg_mutex[_entity]);
        // TODO: To be more stringent on timeout time.
        if (m_msg_cv[_entity].wait_for(
                lock,
                std::chrono::seconds(DEFAULT_WEBSOCKET_TIMEOUT)) ==
            std::cv_status::timeout) {
            m_logger->warn("XdynWebsocket::send: websocket timed out.");
            return false;
        } else {
            return true;
        }
    }
}
}  // namespace lotusim::gazebo