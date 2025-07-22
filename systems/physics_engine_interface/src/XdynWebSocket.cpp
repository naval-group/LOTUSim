#include "XdynWebSocket.h"

namespace lotusim::gazebo {

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
std::unordered_map<gz::sim::Entity, json> XdynWebsocket::m_saved_state;

XdynWebsocket::XdynWebsocket()
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
    for (auto &&i : m_connection_mapping) {
        m_client
            .close(i.second, websocketpp::close::status::going_away, "", ec);
    }
    m_client.stop();
    m_thread->join();
}

std::shared_ptr<XdynWebsocket> XdynWebsocket::getInstance(
    const gz::sim::Entity &_entity,
    const std::string &_name)
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
    const gz::sim::Entity &_entity,
    const std::string &_name,
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

bool XdynWebsocket::activateConnection(const gz::sim::Entity &_entity)
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

bool XdynWebsocket::deactivateConnection(const gz::sim::Entity &_entity)
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

std::string XdynWebsocket::getURI(const gz::sim::Entity &_entity)
{
    return m_uri[_entity];
}

void XdynWebsocket::onOpen(
    const gz::sim::Entity &_entity,
    websocketpp::connection_hdl hdl)
{
    auto uri = m_client.get_con_from_hdl(hdl)->get_uri()->str();
    m_status[_entity] = "opened";
    m_logger->info("XdynWebsocket::onOpen: Opened {}", uri);
}

void XdynWebsocket::onFail(
    const gz::sim::Entity &_entity,
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

    // Transform orientation from NED to GZ frame
    auto ned_quad = gz::math::Quaterniond(
        reply["qr"].back().get<double>(),
        reply["qi"].back().get<double>(),
        reply["qk"].back().get<double>(),
        reply["qj"].back().get<double>());

    gz::math::Quaterniond gz_quad =
        gz::math::Quaterniond(0, 1, 0, 0).Inverse() * ned_quad;

    // Transform lin vel from body frame back to NED frame
    auto body_lin_vel = gz::math::Vector3d{
        reply["u"].back().get<double>(),
        reply["v"].back().get<double>(),
        reply["w"].back().get<double>()};
    auto ned_lin_vel = ned_quad.RotateVectorReverse(body_lin_vel);

    // Transform displacement and angular speed to GZ frame
    json state = {
        {"t", reply["t"].back().get<double>()},
        {"x", reply["x"].back().get<double>()},
        {"y", -1.0 * reply["y"].back().get<double>()},
        {"z", -1.0 * reply["z"].back().get<double>()},
        {"u", ned_lin_vel.X()},
        {"v", -1.0 * ned_lin_vel.Y()},
        {"w", -1.0 * ned_lin_vel.Z()},
        {"p", reply["p"].back().get<double>()},
        {"q", -1.0 * reply["q"].back().get<double>()},
        {"r", -1.0 * reply["r"].back().get<double>()},
        {"qi", gz_quad.X()},
        {"qk", gz_quad.Y()},
        {"qj", gz_quad.Z()},
        {"qr", gz_quad.W()}};
    m_saved_state[entity] = std::move(state);
    m_msg_cv[entity].notify_one();
}

std::optional<std::tuple<json, DomainType>> XdynWebsocket::getNewState(
    const gz::sim::Entity &_entity,
    const json &previous_state,
    float time_diff)
{
    json data = json::object();
    data["Dt"] = time_diff / 1000.0;
    data["states"] = json::array();
    data["states"].push_back(previous_state);

    // frame convention transformation
    // position
    data["states"].back()["y"] =
        data["states"].back()["y"].get<double>() * -1.0;
    data["states"].back()["z"] =
        data["states"].back()["z"].get<double>() * -1.0;

    // quaternion transfrom from gz frame to NED frame

    gz::math::Quaterniond gz_quad = gz::math::Quaterniond(
        data["states"].back()["qr"].get<double>(),
        data["states"].back()["qi"].get<double>(),
        data["states"].back()["qj"].get<double>(),
        data["states"].back()["qk"].get<double>());

    gz::math::Quaterniond ned_quad =
        gz::math::Quaterniond(0, 1, 0, 0) * gz_quad;

    data["states"].back()["qi"] = ned_quad.X();
    data["states"].back()["qj"] = ned_quad.Y();
    data["states"].back()["qk"] = ned_quad.Z();
    data["states"].back()["qr"] = ned_quad.W();

    // Linear vel in NED frame velocity
    auto ned_lin_vel = gz::math::Vector3d{
        data["states"].back()["u"].get<double>(),
        data["states"].back()["v"].get<double>() * -1.0,
        data["states"].back()["w"].get<double>() * -1.0};
    // Transform lin vel from NED to Body frame in Xdyn
    auto body_lin_vel = ned_quad.RotateVector(ned_lin_vel);

    // angular speed
    data["states"].back()["q"] =
        data["states"].back()["q"].get<double>() * -1.0;
    data["states"].back()["r"] =
        data["states"].back()["r"].get<double>() * -1.0;

    std::string excelRow = fmt::format(
        "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
        data["states"].back()["t"].get<double>(),
        m_name_mapping[_entity],
        data["states"].back()["x"].get<double>(),
        data["states"].back()["y"].get<double>(),
        data["states"].back()["z"].get<double>(),
        data["states"].back()["u"].get<double>(),
        data["states"].back()["v"].get<double>(),
        data["states"].back()["w"].get<double>(),
        data["states"].back()["qi"].get<double>(),
        data["states"].back()["qj"].get<double>(),
        data["states"].back()["qk"].get<double>(),
        data["states"].back()["qr"].get<double>(),
        data["states"].back()["p"].get<double>(),
        data["states"].back()["q"].get<double>(),
        data["states"].back()["r"].get<double>());
    m_engine_logger->debug(excelRow);

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
    const gz::sim::Entity &_entity,
    const std::string &message)
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