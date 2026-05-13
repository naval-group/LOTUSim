#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>

#include "geographic_msgs/msg/geo_point.hpp"
#include "lotusim_msgs/action/mas_cmd.hpp"
#include "lotusim_msgs/action/mas_cmd_array.hpp"
#include "lotusim_msgs/msg/mas_cmd.hpp"
#include "lotusim_msgs/msg/vessel_cmd.hpp"
#include "lotusim_msgs/msg/vessel_cmd_array.hpp"
#include "lotusim_msgs/msg/vessel_position_array.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

static std::atomic<bool> g_shutdown_requested{false}; // global flag

static constexpr double SPAWN_LATITUDE = 1.2605794416293148;
static constexpr double SPAWN_LONGITUDE = 103.7516212463379;
static constexpr double SPAWN_ALTITUDE = -30.0;
int vessel_id = 0;

template <typename T>
T random_choice(const std::vector<T>& vec)
{
    static std::mt19937 gen(std::random_device{}());
    std::uniform_int_distribution<> dist(0, vec.size() - 1);
    return vec[dist(gen)];
}

class ExampleNode : public rclcpp::Node {
public:
    using VesselPositionArray = lotusim_msgs::msg::VesselPositionArray;
    using VesselCmdArray = lotusim_msgs::msg::VesselCmdArray;
    using VesselCmd = lotusim_msgs::msg::VesselCmd;
    using MASCmdArray = lotusim_msgs::action::MASCmdArray;
    using GoalHandleMASCmdArray = rclcpp_action::ClientGoalHandle<MASCmdArray>;

    ExampleNode() : Node("spawn_and_control_ships")
    {
        pose_subscription_ = this->create_subscription<VesselPositionArray>(
            "lotusim/poses",
            rclcpp::QoS(10),
            std::bind(&ExampleNode::poses_callback, this, std::placeholders::_1));

        cmd_publisher_ = this->create_publisher<VesselCmdArray>(
            "/lotusim/vessel_cmd_array", 10);

        mas_array_action_client_ =
            rclcpp_action::create_client<MASCmdArray>(
                this, "lotusim/mas_cmd_array");

        // Print positions
        timer_ = this->create_wall_timer(
            1s,
            std::bind(&ExampleNode::print_vessel_positions, this));

        // Send RPM commands continuously
        cmd_timer_ = this->create_wall_timer(
            500ms,
            std::bind(&ExampleNode::send_commands, this));
    }

    void spawn_multiple_ships(int number_of_ships)
    {
        if (!mas_array_action_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "MASCmdArray server not available");
            return;
        }

        auto goal_msg = MASCmdArray::Goal();

        for (int i = 0; i < number_of_ships; ++i) {
            lotusim_msgs::msg::MASCmd msg;

            msg.cmd_type = lotusim_msgs::msg::MASCmd::CREATE_CMD;
            msg.model_name = "lrauv";
            std::string name = "lrauv_" + std::to_string(vessel_id);
            msg.vessel_name = name;
            vessel_names_.push_back(name); // store for control

            geographic_msgs::msg::GeoPoint geo;
            double offset = 0.0001;

            geo.latitude = SPAWN_LATITUDE +
                vessel_id * offset * random_choice<int>({-1, 1});
            geo.longitude = SPAWN_LONGITUDE +
                vessel_id * offset * random_choice<int>({-1, 1});
            geo.altitude = SPAWN_ALTITUDE;
            msg.geo_point = geo;

            msg.sdf_string = R"(
            <lotus_param>
                <physics_engine_interface>
                <underwater>
                    <connection_type>XDynWebSocket</connection_type>
                    <uri>ws://127.0.0.1:12346</uri>
                    <thrusters>
                        <thrusters1>propeller</thrusters1>
                    </thrusters>
                </underwater>
                <surface>
                    <connection_type>XDynWebSocket</connection_type>
                    <uri>ws://127.0.0.1:12345</uri>
                    <thrusters>
                        <thrusters1>propeller</thrusters1>
                    </thrusters>
                </surface>
                <init_state>Underwater</init_state>
                </physics_engine_interface>
            </lotus_param>
            )";

            vessel_id++;
            goal_msg.cmd.push_back(msg);
        }

        auto send_goal_options =
            rclcpp_action::Client<MASCmdArray>::SendGoalOptions();

        mas_array_action_client_->async_send_goal(goal_msg, send_goal_options);

        RCLCPP_INFO(this->get_logger(), "Spawned ships");
    }

    void delete_all_vessels(rclcpp::Executor& exec)
    {
        if (!mas_array_action_client_->wait_for_action_server(2s)) {
            RCLCPP_WARN(this->get_logger(), "Action server not available for cleanup");
            return;
        }

        MASCmdArray::Goal goal_msg;
        for (const auto& name : vessel_names_) {
            lotusim_msgs::msg::MASCmd msg;
            msg.cmd_type = lotusim_msgs::msg::MASCmd::DELETE_CMD; 
            msg.vessel_name = name;
            goal_msg.cmd.push_back(msg);
        }

        if (goal_msg.cmd.empty()) return;
        exec.spin_some(200ms); // some time

        // leave some time
        auto goal_handle_future = mas_array_action_client_->async_send_goal(goal_msg);
        exec.spin_until_future_complete(goal_handle_future);

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Delete goal rejected");
            return;
        }

        auto result_future = mas_array_action_client_->async_get_result(goal_handle);
        exec.spin_until_future_complete(result_future);

        RCLCPP_INFO(this->get_logger(), "All vessels deleted");
        vessel_names_.clear();
    }

private:
    void poses_callback(const VesselPositionArray::SharedPtr msg)
    {
        for (const auto& vessel : msg->vessels) {
            vessel_poses_[vessel.vessel_name] =
                std::make_pair(vessel.geo_point.latitude,
                               vessel.geo_point.longitude);
        }
    }

    void print_vessel_positions()
    {
        if (vessel_poses_.empty()) {
            RCLCPP_INFO(this->get_logger(), "No vessel positions yet");
            return;
        }

        std::stringstream ss;
        for (const auto& pair : vessel_poses_) {
            ss << pair.first << ": (" << pair.second.first
               << ", " << pair.second.second << ") ";
        }

        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }

    // send RPM commands
    void send_commands()
    {
        if (vessel_names_.empty()){
            return;
        }
        
        VesselCmdArray cmd_array;
        for (const auto& name : vessel_names_) {
            VesselCmd cmd;
            cmd.vessel_name = name;
            // MATCH EXACT STRING FORMAT
            cmd.cmd_string =
                "{\"propeller(rpm)\":200,\"propeller(P/D)\":0.88}";
            cmd_array.cmds.push_back(cmd);
        }

        cmd_publisher_->publish(cmd_array);

        RCLCPP_INFO(this->get_logger(), "Published propeller commands");
    }

private:
    rclcpp::Subscription<VesselPositionArray>::SharedPtr pose_subscription_;
    rclcpp::Publisher<VesselCmdArray>::SharedPtr cmd_publisher_;
    rclcpp_action::Client<MASCmdArray>::SharedPtr mas_array_action_client_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr cmd_timer_;

    std::unordered_map<std::string, std::pair<double, double>> vessel_poses_;
    std::vector<std::string> vessel_names_; // track vessels
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ExampleNode>();

    node->spawn_multiple_ships(2);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    // install signal handler after init
    std::signal(SIGINT, [](int) {
        g_shutdown_requested = true;
    });
    std::signal(SIGTERM, [](int) {
        g_shutdown_requested = true;
    });

    // Spin manually so we can break on signal
    while (rclcpp::ok() && !g_shutdown_requested) {
        exec.spin_some(100ms);
    }
    
    // ROS is still up here — cleanup works
    RCLCPP_INFO(node->get_logger(), "Shutting down, deleting vessels...");
    node->delete_all_vessels(exec);
    
    rclcpp::shutdown();
    return 0;
}