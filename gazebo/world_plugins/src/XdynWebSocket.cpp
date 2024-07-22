#include "world_plugins/XdynWebSocket.h"

namespace liquidai {
namespace gazebo {

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
    m_thread.reset(new websocketpp::lib::thread(&Client::run, &m_client));

    // workbook = workbook_new(
    //     "/home/malcom/garden_ws/src/liquidai/physics/xdynSurface/gz.xlsx");
    // worksheet = workbook_add_worksheet(workbook, NULL);
    // lxw_format *format = workbook_add_format(workbook);
    // format_set_num_format(format, "$#,##0.00");
    // {
    //     worksheet_write_string(worksheet, 0, 0, "t", NULL);
    //     worksheet_write_string(worksheet, 0, 1, "X", NULL);
    //     worksheet_write_string(worksheet, 0, 2, "Y", NULL);
    //     worksheet_write_string(worksheet, 0, 3, "Z", NULL);
    //     worksheet_write_string(worksheet, 0, 4, "U", NULL);
    //     worksheet_write_string(worksheet, 0, 5, "v", NULL);
    //     worksheet_write_string(worksheet, 0, 6, "w", NULL);
    //     worksheet_write_string(worksheet, 0, 7, "qi", NULL);
    //     worksheet_write_string(worksheet, 0, 8, "qj", NULL);
    //     worksheet_write_string(worksheet, 0, 9, "qk", NULL);
    //     worksheet_write_string(worksheet, 0, 10, "qr", NULL);
    //     worksheet_write_string(worksheet, 0, 11, "p", NULL);
    //     worksheet_write_string(worksheet, 0, 12, "q", NULL);
    //     worksheet_write_string(worksheet, 0, 13, "r", NULL);
    // }
}

XdynWebsocket::~XdynWebsocket()
{
    m_client.stop_perpetual();
    websocketpp::lib::error_code ec;
    for (auto &&i : m_connection_mapping) {
        m_client.close(
            i.second, websocketpp::close::status::going_away, "", ec);
    }
    m_client.stop();
    m_thread->join();

    // workbook_close(workbook);
}

std::shared_ptr<XdynWebsocket> XdynWebsocket::getInstance(
    const gz::sim::Entity &_entity, const std::string &_name)
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
    }
    else {
        gzerr << "XdynWebsocket::createConnection: No uri for " << _name
              << std::endl;
        return false;
    }
    m_uri[_entity] = uri;

    if (_sdf->HasElement("thrusters")) {
        gz_liquidai_msgs::msgs::XdynCmd xdyn_cmd;
        xdyn_cmd.set_name(_name);
        auto sdfPtr_thruster = _sdf->GetElement("thrusters")->GetFirstElement();
        do {
            gz_liquidai_msgs::msgs::XdynCmd_ThrusterCmd *thruster_cmd =
                xdyn_cmd.add_cmd();
            thruster_cmd->set_name(sdfPtr_thruster->Get<std::string>());
            //  Needed 0.01 as xdyn give error at rpm 0
            thruster_cmd->set_rpm(0.01);
            // thruster_cmd->set_rpm(30);
            thruster_cmd->set_pd(0.79);
            thruster_cmd->set_beta(0.0);
            sdfPtr_thruster = sdfPtr_thruster->GetNextElement();
        } while (sdfPtr_thruster != sdf::ElementPtr(nullptr));
        m_xdyn_cmd[_entity] = std::move(xdyn_cmd);
    }

    return true;
}

bool XdynWebsocket::activateConnection(const gz::sim::Entity &_entity)
{
    websocketpp::lib::error_code ec;

    Client::connection_ptr con = m_client.get_connection(m_uri[_entity], ec);
    if (ec) {
        std::cout << "XdynWebsocket::activateConnection: Connect "
                     "initialization error: "
                  << ec.message() << std::endl;
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

    while (m_status[_entity] != "opened") {
        std::cout << "Starting connection: " << m_name_mapping[_entity]
                  << std::endl;
        m_client.connect(con);
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }
    m_gz_node.Subscribe(
        m_name_mapping[_entity] + "/cmd_vel", &XdynWebsocket::thrustCmd, this);
    return true;
}

bool XdynWebsocket::deactivateConnection(const gz::sim::Entity &_entity)
{
    m_gz_node.Unsubscribe(m_name_mapping[_entity] + "/cmd_vel");
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
    const gz::sim::Entity &_entity, websocketpp::connection_hdl hdl)
{
    auto uri = m_client.get_con_from_hdl(hdl)->get_uri()->str();
    m_status[_entity] = "opened";
    std::cout << "Opened " << uri << std::endl;
}

void XdynWebsocket::onFail(
    const gz::sim::Entity &_entity, websocketpp::connection_hdl hdl)
{
    auto uri = m_client.get_con_from_hdl(hdl)->get_uri()->str();
    m_status[_entity] = "failed";
    std::cout << "Failed " << uri << std::endl;
}

void XdynWebsocket::onMessage(
    websocketpp::connection_hdl hdl,
    websocketpp::config::asio_client::message_type::ptr msg)
{
    // Lacking info to tell us which ship this is for. to map to hdl to know
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
    const gz::sim::Entity &_entity, const json &previous_state, float time_diff)
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

    // {
    //     worksheet_write_number(
    //         worksheet, row, 0, data["states"].back()["t"].get<double>(),
    //         NULL);
    //     worksheet_write_number(
    //         worksheet, row, 1, data["states"].back()["x"].get<double>(),
    //         NULL);
    //     worksheet_write_number(
    //         worksheet, row, 2, data["states"].back()["y"].get<double>(),
    //         NULL);
    //     worksheet_write_number(
    //         worksheet, row, 3, data["states"].back()["z"].get<double>(),
    //         NULL);
    //     worksheet_write_number(
    //         worksheet, row, 4, data["states"].back()["u"].get<double>(),
    //         NULL);
    //     worksheet_write_number(
    //         worksheet, row, 5, data["states"].back()["v"].get<double>(),
    //         NULL);
    //     worksheet_write_number(
    //         worksheet, row, 6, data["states"].back()["w"].get<double>(),
    //         NULL);
    //     worksheet_write_number(
    //         worksheet, row, 7, data["states"].back()["qi"].get<double>(),
    //         NULL);
    //     worksheet_write_number(
    //         worksheet, row, 8, data["states"].back()["qj"].get<double>(),
    //         NULL);
    //     worksheet_write_number(
    //         worksheet, row, 9, data["states"].back()["qk"].get<double>(),
    //         NULL);
    //     worksheet_write_number(
    //         worksheet,
    //         row,
    //         10,
    //         data["states"].back()["qr"].get<double>(),
    //         NULL);
    //     worksheet_write_number(
    //         worksheet, row, 11, data["states"].back()["p"].get<double>(),
    //         NULL);
    //     worksheet_write_number(
    //         worksheet, row, 12, data["states"].back()["q"].get<double>(),
    //         NULL);
    //     worksheet_write_number(
    //         worksheet, row, 13, data["states"].back()["r"].get<double>(),
    //         NULL);
    //     row += 1;
    // }

    auto cmd = json::object();
    for (auto &&cmd_msg : m_xdyn_cmd[_entity].cmd()) {
        cmd[cmd_msg.name() + "(rpm)"] = cmd_msg.rpm();
        cmd[cmd_msg.name() + "(P/D)"] = cmd_msg.pd();
        cmd[cmd_msg.name() + "(beta)"] = cmd_msg.beta();
    }
    data["commands"] = cmd;
    data["requested_output"] = json::array();
    std::string msg_string = data.dump();
    if (send(_entity, msg_string)) {
        return std::make_tuple(m_saved_state[_entity], DomainType::Surface);
    }
    return std::nullopt;
}

bool XdynWebsocket::send(const gz::sim::Entity &_entity, std::string message)
{
    websocketpp::lib::error_code ec;
    m_client.send(
        m_connection_mapping[_entity]->get_handle(),
        message,
        websocketpp::frame::opcode::text,
        ec);
    if (ec) {
        std::cout << "> Error sending message: " << ec.message() << std::endl;
        return false;
    }
    {
        std::unique_lock<std::mutex> lock(m_msg_mutex[_entity]);
        // TODO: To be more stringent on timeout time.
        if (m_msg_cv[_entity].wait_for(lock, std::chrono::seconds(1000)) ==
            std::cv_status::timeout) {
            std::cout << "websocket timed out." << std::endl;
            return false;
        }
        else {
            return true;
        }
    }
}

void XdynWebsocket::thrustCmd(const gz_liquidai_msgs::msgs::XdynCmd &_msg)
{
    std::cout << "Received: " << _msg.name() << std::endl;
    if (m_entity_mapping.find(_msg.name()) != m_entity_mapping.end()) {
        m_xdyn_cmd[m_entity_mapping[_msg.name()]] = std::move(_msg);
    }
    else {
        std::cout << "Received unknown vessel: " << _msg.name() << std::endl;
    }
}

} // namespace gazebo
} // namespace liquidai