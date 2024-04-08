#include "world_plugins/XdynWebSocket.h"

namespace liquidai {
namespace gazebo {

// std::weak_ptr<WaveRaoInterface> XdynWebsocket::m_instance =
//     std::shared_ptr<XdynWebsocket>(nullptr);

std::weak_ptr<XdynWebsocket> XdynWebsocket::m_instance =
    std::shared_ptr<XdynWebsocket>(nullptr);

XdynWebsocket::XdynWebsocket()
{
    m_client.clear_access_channels(websocketpp::log::alevel::all);
    m_client.clear_error_channels(websocketpp::log::elevel::all);
    m_client.init_asio();
    m_client.start_perpetual();
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
    m_thread->join();
}

// std::shared_ptr<WaveRaoInterface> XdynWebsocket::getInstance()
std::shared_ptr<XdynWebsocket> XdynWebsocket::getInstance()
{
    if (m_instance.expired()) {
        auto instance  = std::make_shared<XdynWebsocket>();
        m_instance = instance; 
        return instance;
    }
    return m_instance.lock();
}

bool XdynWebsocket::createConnection(
    const std::string &name, const std::string &uri)
{

    websocketpp::lib::error_code ec;
    Client::connection_ptr con = m_client.get_connection(uri, ec);
    if (ec) {
        std::cout << "> Connect initialization error: " << ec.message()
                  << std::endl;
        return -1;
    }
    m_connection_mapping.insert({name, con->get_handle()});
    m_status.insert({name, "configuring"});
    con->set_open_handler(bind(
        &XdynWebsocket::onOpen,
        this,
        name,
        websocketpp::lib::placeholders::_1));
    con->set_fail_handler(bind(
        &XdynWebsocket::onFail,
        this,
        name,
        websocketpp::lib::placeholders::_1));
    con->set_message_handler(bind(
        &XdynWebsocket::onMessage,
        this,
        websocketpp::lib::placeholders::_1,
        websocketpp::lib::placeholders::_2));

    while (m_status[name] != "opened") {
        std::cout << "Starting connection: " << uri << std::endl;
        m_client.connect(con);
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }

    // To add, pass the vessel name to all the plugins and have it at error
    // printout stage

    // Creating vessel command subscriber
    // ToDo, add the msg to custom binding
    m_node.Subscribe(name + "/cmd_vel", &XdynWebsocket::thrustCmd, this);
}

void XdynWebsocket::onOpen(std::string name, websocketpp::connection_hdl hdl)
{
    auto uri = m_client.get_con_from_hdl(hdl)->get_uri()->str();
    m_status[name] = "opened";
    std::cout << "Opened " << uri << std::endl;
}

void XdynWebsocket::onFail(std::string name, websocketpp::connection_hdl hdl)
{
    auto uri = m_client.get_con_from_hdl(hdl)->get_uri()->str();
    m_status[name] = "failed";
    std::cout << "Failed " << uri << std::endl;
}

void XdynWebsocket::onMessage(
    websocketpp::connection_hdl hdl,
    websocketpp::config::asio_client::message_type::ptr msg)
{
    std::unique_lock<std::mutex> lock(m_msg_mutex);
    json reply = json::parse(msg->get_payload());
    json state = {
        {"t", reply["t"].back()},
        {"x", reply["x"].back()},
        {"y", reply["y"].back()},
        {"z", reply["z"].back()},
        {"u", reply["u"].back()},
        {"v", reply["v"].back()},
        {"w", reply["w"].back()},
        {"p", reply["p"].back()},
        {"q", reply["q"].back()},
        {"r", reply["r"].back()},
        {"qr", reply["qr"].back()},
        {"qi", reply["qi"].back()},
        {"qj", reply["qj"].back()},
        {"qk", reply["qk"].back()}};
    m_resp_msg = state;
    m_msg_cv.notify_one();
}

std::optional<json> XdynWebsocket::getNewState(
    const std::string &name, json previous_state, float time_diff)
{
    json data = json::object();
    data["Dt"] = time_diff / 1000;
    data["states"] = json::array();
    data["states"].push_back(previous_state);

    if (m_xdyn_cmd.find(name) == m_xdyn_cmd.end())
        data["commands"] = json::object();
    else {
        auto cmd = json::object();
        cmd["rpm"] = m_xdyn_cmd[name].rpm();
        cmd["pd"] = m_xdyn_cmd[name].pd();
        cmd["beta"] = m_xdyn_cmd[name].beta();
        data["commands"] = cmd;
    }
    data["requested_output"] = json::array();
    std::string msg_string = data.dump();
    if (send(name, msg_string)) {
        return m_resp_msg;
    }
    else {
        return std::nullopt;
    }
}

bool XdynWebsocket::send(const std::string &name, std::string message)
{
    websocketpp::lib::error_code ec;

    m_client.send(
        m_connection_mapping[name],
        message,
        websocketpp::frame::opcode::text,
        ec);
    if (ec) {
        std::cout << "> Error sending message: " << ec.message() << std::endl;
        return false;
    }
    {
        std::unique_lock<std::mutex> lock(m_msg_mutex);
        if (m_msg_cv.wait_for(lock, std::chrono::milliseconds(100)) ==
            std::cv_status::timeout) {
            std::cout << "websocket timed out." << std::endl;
            return false;
        }
        else {
            return true;
        }
    }
}

void XdynWebsocket::thrustCmd(
    const gz_liquidai_plugins_msgs::msgs::XdynCmd &_msg)
{
    if (m_xdyn_cmd.find(_msg.name()) != m_xdyn_cmd.end())
        m_xdyn_cmd[_msg.name()] = std::move(_msg);
    else
        m_xdyn_cmd[_msg.name()] = _msg;
}

} // namespace gazebo
} // namespace liquidai