#ifndef __XDYN_WEBSOCKET_HH__
#define __XDYN_WEBSOCKET_HH__

#include "gz_liquidai_msgs/msg/xdyncmdmsg.pb.h"
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <gz/transport/Node.hh>

// #include "world_plugins/WaveRaoInterface.h"
#include <websocketpp/client.hpp>
#include <websocketpp/common/memory.hpp>
#include <websocketpp/common/thread.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>

namespace liquidai {
namespace gazebo {

namespace Weblib = websocketpp::lib;
using hdl = websocketpp::connection_hdl;
using Client = websocketpp::client<websocketpp::config::asio_client>;
using json = nlohmann::json;

enum class ConnectionType { XDynWebSocket, XDynGRPC, Manual };

class XdynWebsocket {
public:
    XdynWebsocket();
    ~XdynWebsocket();
    void operator=(const XdynWebsocket &) = delete;
    XdynWebsocket(XdynWebsocket &other) = delete;
    // static std::shared_ptr<WaveRaoInterface> getInstance();
    static std::shared_ptr<XdynWebsocket> getInstance();
    bool createConnection(const std::string &name, const std::string &uri);
    std::optional<json>
    getNewState(const std::string &name, json previous_state, float time_dif);

private:
    bool send(const std::string &name, std::string message);
    void onMessage(
        websocketpp::connection_hdl hdl,
        websocketpp::config::asio_client::message_type::ptr msg);
    void onOpen(std::string name, websocketpp::connection_hdl hdl);
    void onFail(std::string name, websocketpp::connection_hdl hdl);
    void thrustCmd(const gz_liquidai_plugins_msgs::msgs::XdynCmd &_msg);

private:
    std::string m_vessel_name;
    Client m_client;
    Weblib::shared_ptr<Weblib::thread> m_thread;

    // Vessle name is to websocket connection
    std::unordered_map<std::string, websocketpp::connection_hdl>
        m_connection_mapping;
    std::mutex m_msg_mutex;
    std::condition_variable m_msg_cv;
    json m_resp_msg;
    std::unordered_map<std::string, std::string> m_status;
    // static std::weak_ptr<WaveRaoInterface> m_instance;

    static std::weak_ptr<XdynWebsocket> m_instance;

    std::unordered_map<std::string, gz_liquidai_plugins_msgs::msgs::XdynCmd>
        m_xdyn_cmd;

    //temp fix
    gz::transport::Node m_node;

};

} // namespace gazebo
} // namespace liquidai
#endif