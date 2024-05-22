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
        gz_liquidai_msgs::msg::XdynCmd xdyn_cmd;
        xdyn_cmd.set_name(_name);
        auto sdfPtr_thruster = _sdf->GetElement("thrusters")->GetFirstElement();
        do {
            gz_liquidai_msgs::msg::XdynCmd_ThrusterCmd *thruster_cmd =
                xdyn_cmd.add_cmd();
            thruster_cmd->set_name(sdfPtr_thruster->Get<std::string>());
            thruster_cmd->set_rpm(0.01);
            thruster_cmd->set_pd(0.79);
            thruster_cmd->set_beta(0);
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

    // take note of frame transformation
    json state = {
        {"t", reply["t"].back()},
        {"x", reply["x"].back()},
        {"y", -1 * reply["y"].back().get<double>()},
        {"z", -1 * reply["z"].back().get<double>()},
        {"u", reply["u"].back()},
        {"v", -1 * reply["v"].back().get<double>()},
        {"w", -1 * reply["w"].back().get<double>()},
        {"p", reply["p"].back()},
        {"q", -1 * reply["q"].back().get<double>()},
        {"r", -1 * reply["r"].back().get<double>()},
        {"qr", reply["qr"].back()},
        {"qi", reply["qi"].back()},
        {"qj", reply["qj"].back()},
        {"qk", reply["qk"].back()}};

    m_saved_state[entity] = std::move(state);
    m_msg_cv[entity].notify_one();
}

std::optional<std::tuple<json, DomainType>> XdynWebsocket::getNewState(
    const gz::sim::Entity &_entity, const json &previous_state, float time_diff)
{
    json data = json::object();
    data["Dt"] = time_diff / 1000;
    data["states"] = json::array();
    data["states"].push_back(previous_state);

    // frame convention transformation
    data["states"].back()["y"] = data["states"].back()["y"].get<double>() * -1;
    data["states"].back()["z"] = data["states"].back()["z"].get<double>() * -1;
    data["states"].back()["v"] = data["states"].back()["v"].get<double>() * -1;
    data["states"].back()["w"] = data["states"].back()["w"].get<double>() * -1;
    data["states"].back()["q"] = data["states"].back()["q"].get<double>() * -1;
    data["states"].back()["r"] = data["states"].back()["r"].get<double>() * -1;

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

void XdynWebsocket::thrustCmd(const gz_liquidai_msgs::msg::XdynCmd &_msg)
{
    if (m_entity_mapping.find(_msg.name()) != m_entity_mapping.end()) {
        m_xdyn_cmd[m_entity_mapping[_msg.name()]] = std::move(_msg);
    }
    else {
        std::cout << "Received unknown vessel: " << _msg.name() << std::endl;
    }
}

} // namespace gazebo
} // namespace liquidai