#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <csignal> 

#include "geographic_msgs/msg/geo_point.hpp"
#include "lotusim_msgs/action/mas_cmd.hpp"
#include "lotusim_msgs/action/mas_cmd_array.hpp"
#include "lotusim_msgs/msg/mas_cmd.hpp"
#include "lotusim_msgs/msg/vessel_position_array.hpp"
#include "lotusim_msgs/srv/set_waypoints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

static std::atomic<bool> g_shutdown_requested{false}; // global flag

static constexpr double SPAWN_LATITUDE = 1.2605794416293148;
static constexpr double SPAWN_LONGITUDE = 103.7516212463379;
static constexpr double SPAWN_ALTITUDE = -30.0;
static constexpr double OFFSET = 0.0001;
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
    using MASCmd = lotusim_msgs::action::MASCmd;
    using MASCmdArray = lotusim_msgs::action::MASCmdArray;
    using GoalHandleMASCmdArray = rclcpp_action::ClientGoalHandle<MASCmdArray>;
    using SetWaypoints = lotusim_msgs::srv::SetWaypoints;
    std::vector<std::string> spawned_vessels_;

    ExampleNode() : Node("spawn_ships", "lotusim")
    {
        pose_subscription_ = this->create_subscription<VesselPositionArray>(
            "poses",
            rclcpp::QoS(10),
            [this](const VesselPositionArray::SharedPtr msg) {
                this->poses_callback(msg);
            });

        mas_array_action_client_ =
            rclcpp_action::create_client<MASCmdArray>(this, "mas_cmd_array");

        timer_ = this->create_wall_timer(
            1s,
            std::bind(&ExampleNode::print_vessel_positions, this));
    }

    void setupRosForModel(const std::string& model_name)
    {
        // Skip if we already have a client for this model
        if (waypoint_clients_.count(model_name)) {
            return;
        }

        auto client =
            this->create_client<SetWaypoints>(model_name + "/waypoints");

        for (int attempt = 1; attempt <= 3; ++attempt) {
            if (client->wait_for_service(1s)) {
                waypoint_clients_[model_name] = client;
                return;
            }
            RCLCPP_INFO(
                this->get_logger(),
                "Waiting for waypoint service... (attempt %d/3)",
                attempt);
        }

        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to connect to waypoint service for %s",
            model_name.c_str());
    }

    void send_random_waypoint_request(const std::string& model_name)
    {
        auto it = waypoint_clients_.find(model_name);
        if (it == waypoint_clients_.end()) {
            setupRosForModel(model_name);
            it = waypoint_clients_.find(model_name);
        }

        auto request = std::make_shared<SetWaypoints::Request>();
        request->header.stamp = this->get_clock()->now();
        request->header.frame_id = "world";

        geographic_msgs::msg::GeoPoint wp;
        wp.latitude =
            SPAWN_LATITUDE + OFFSET * vessel_id * random_choice<int>({-1, 1});
        wp.longitude =
            SPAWN_LONGITUDE + OFFSET * vessel_id * random_choice<int>({-1, 1});
        wp.altitude = SPAWN_ALTITUDE;
        request->path = {wp};
        request->loop = false;

        // Synchronous call — mirrors rclpy.spin_until_future_complete
        auto future = it->second->async_send_request(request);
        if (rclcpp::spin_until_future_complete(
                this->shared_from_this(),
                future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Service call failed for %s",
                model_name.c_str());
            return;
        }

        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(
                this->get_logger(),
                "Waypoint set: (%.7f, %.7f)",
                wp.latitude,
                wp.longitude);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to set waypoints");
        }
    }

    void spawn_multiple_circling_ship(int number_of_ships)
    {
        if (!mas_array_action_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(
                this->get_logger(),
                "MASCmdArray action server not available");
            return;
        }
        auto goal_msg = MASCmdArray::Goal();

        for (int i = 0; i < number_of_ships; ++i) {
            lotusim_msgs::msg::MASCmd msg;
            msg.cmd_type = lotusim_msgs::msg::MASCmd::CREATE_CMD;
            msg.model_name = "dtmb_hull";
            std::string name = "dtmb_" + std::to_string(vessel_id);
            msg.vessel_name = name;
            spawned_vessels_.push_back(name);

            geographic_msgs::msg::GeoPoint geo;
            geo.latitude = SPAWN_LATITUDE +
                           vessel_id * OFFSET * random_choice<int>({-1, 1});
            geo.longitude = SPAWN_LONGITUDE +
                            vessel_id * OFFSET * random_choice<int>({-1, 1});
            geo.altitude = SPAWN_ALTITUDE;
            msg.geo_point = geo;

            msg.sdf_string = R"(
                <lotus_param>
                    <waypoint_follower>
                        <follower>
                            <loop>true</loop>
                            <linear_accel_limit>0.5</linear_accel_limit>
                            <angular_accel_limit>0.005</angular_accel_limit>
                            <angular_velocities_limits>0.01</angular_velocities_limits>
                            <range_tolerance>2</range_tolerance>
                            <circle>
                                <radius>100</radius>
                            </circle>
                        </follower>
                    </waypoint_follower>
                </lotus_param>
            )";
            vessel_id += 1;
            goal_msg.cmd.push_back(msg);
        }

        auto send_goal_options =
            rclcpp_action::Client<MASCmdArray>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [](GoalHandleMASCmdArray::SharedPtr goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("rclcpp"),
                        "Goal rejected by server");
                } else {
                    RCLCPP_INFO(
                        rclcpp::get_logger("rclcpp"),
                        "Goal accepted by server");
                }
            };
        send_goal_options.result_callback =
            [](const GoalHandleMASCmdArray::WrappedResult& result) {
                RCLCPP_INFO(
                    rclcpp::get_logger("rclcpp"),
                    "Goal finished with status %d",
                    static_cast<int>(result.code));
            };

        mas_array_action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void delete_all_vessels(rclcpp::Executor& exec)
    {
        if (!mas_array_action_client_->wait_for_action_server(2s)) {
            RCLCPP_WARN(this->get_logger(), "Action server not available for cleanup");
            return;
        }

        MASCmdArray::Goal goal_msg;
        for (const auto& name : spawned_vessels_) {
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
        spawned_vessels_.clear();
    }

private:
    void poses_callback(
        const lotusim_msgs::msg::VesselPositionArray::SharedPtr msg)
    {
        for (const auto& vessel : msg->vessels) {
            vessel_poses_[vessel.vessel_name] = std::make_pair(
                vessel.geo_point.latitude,
                vessel.geo_point.longitude);
        }
    }

    void print_vessel_positions()
    {
        if (!vessel_poses_.empty()) {
            std::stringstream ss;
            for (const auto& pair : vessel_poses_) {
                ss << pair.first << ": (" << pair.second.first << ", "
                   << pair.second.second << ") ";
            }
            RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        }
    }

    rclcpp::Subscription<VesselPositionArray>::SharedPtr pose_subscription_;
    rclcpp_action::Client<MASCmdArray>::SharedPtr mas_array_action_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unordered_map<std::string, std::pair<double, double>> vessel_poses_;
    std::unordered_map<std::string, rclcpp::Client<SetWaypoints>::SharedPtr>
        waypoint_clients_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ExampleNode>();

    // create ships
    node->spawn_multiple_circling_ship(2);
    node->send_random_waypoint_request("dtmb_0");

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

    exec.remove_node(node);
    node.reset();
    
    rclcpp::shutdown();    
    return 0;
}
