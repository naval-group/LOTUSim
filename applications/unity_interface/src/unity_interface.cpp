#include "unity_interface/unity_interface.hpp"

using namespace boost::asio;
using ip::tcp;
using std::cout;
using std::endl;
using std::string;

UnityInterface::UnityInterface()
    : Node("unity_interface")

{
    m_tf_subscription = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf",
        10,
        std::bind(&UnityInterface::tfCallback, this, std::placeholders::_1));
}

void UnityInterface::tfCallback(
    const tf2_msgs::msg::TFMessage::SharedPtr msg) const
{
    // https://docs.ros2.org/latest/api/tf2_msgs/msg/TFMessage.html
    for (auto &&transform_stamped : msg->transforms) {
        std::string debug = "child frame: ";
        debug += transform_stamped.child_frame_id;
        debug += "\nx:";
        debug += transform_stamped.transform.translation.x;
        debug += " y:";
        debug += transform_stamped.transform.translation.y;
        debug += " z: ";
        debug += transform_stamped.transform.translation.z;
        debug += "\nx:";
        debug += transform_stamped.transform.rotation.x;
        debug += " y:";
        debug += transform_stamped.transform.rotation.y;
        debug += " z:";
        debug += transform_stamped.transform.rotation.z;
        debug += " w:";
        debug += transform_stamped.transform.rotation.w;
        std::cout << debug << std::endl;
    }
}

int main(int argc, char *argv[])
{
    // rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<UnityInterface>());
    // rclcpp::shutdown();

    boost::asio::io_context io_context;
    // socket creation
    tcp::socket socket(io_context);
    // connection
    socket.connect(tcp::endpoint(
        boost::asio::ip::address::from_string("127.0.0.1"), 8888));

    boost::system::error_code error;
    boost::asio::streambuf receive_buffer;
    boost::asio::read(
        socket, receive_buffer, boost::asio::transfer_all(), error);
    if (error && error != boost::asio::error::eof) {
        cout << "receive failed: " << error.message() << endl;
    }
    else {
        const char *data =
            boost::asio::buffer_cast<const char *>(receive_buffer.data());
        cout << data << endl;
    }

    return 0;
}