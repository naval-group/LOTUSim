#include <cmath>
#include <csignal>
#include <memory>
#include <string>
#include <unordered_map>

#include "geographic_msgs/msg/geo_point.hpp"
#include "lotusim_msgs/action/mas_cmd.hpp"
#include "lotusim_msgs/action/mas_cmd_array.hpp"
#include "lotusim_msgs/msg/mas_cmd.hpp"
#include "lotusim_msgs/msg/vessel_position_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

static std::atomic<bool> g_shutdown_requested{false};

static constexpr double SPAWN_LATITUDE = 1.2605794416293148;
static constexpr double SPAWN_LONGITUDE = 103.7516212463379;
static constexpr double SPAWN_ALTITUDE = 0.0;

class RadarTestSpawner : public rclcpp::Node {
public:
    using VesselPositionArray = lotusim_msgs::msg::VesselPositionArray;
    using MASCmd = lotusim_msgs::action::MASCmd;
    using MASCmdArray = lotusim_msgs::action::MASCmdArray;
    using GoalHandleMASCmd = rclcpp_action::ClientGoalHandle<MASCmd>;
    using GoalHandleMASCmdArray = rclcpp_action::ClientGoalHandle<MASCmdArray>;

    RadarTestSpawner() : Node("radar_test_spawner")
    {
        pose_subscription_ = this->create_subscription<VesselPositionArray>(
            "/lotusim/poses",
            rclcpp::QoS(10),
            std::bind(
                &RadarTestSpawner::poses_callback,
                this,
                std::placeholders::_1));

        mas_action_client_ =
            rclcpp_action::create_client<MASCmd>(this, "/lotusim/mas_cmd");

        mas_array_action_client_ = rclcpp_action::create_client<MASCmdArray>(
            this,
            "/lotusim/mas_cmd_array");
    }

    void spawn_static_commando()
    {
        if (!mas_action_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(
                this->get_logger(),
                "MASCmd action server not available");
            return;
        }

        lotusim_msgs::msg::MASCmd msg;
        msg.cmd_type = lotusim_msgs::msg::MASCmd::CREATE_CMD;
        msg.model_name = "commando";
        msg.vessel_name = "commando_center";

        geographic_msgs::msg::GeoPoint geo;
        geo.latitude = SPAWN_LATITUDE;
        geo.longitude = SPAWN_LONGITUDE;
        geo.altitude = SPAWN_ALTITUDE;
        msg.geo_point = geo;

        msg.sdf_string = R"(
        <lotus_param>
            <waypoint_follower>
                <follower>
                    <loop>false</loop>
                </follower>
            </waypoint_follower>
        </lotus_param>
        )";

        auto goal_msg = MASCmd::Goal();
        goal_msg.cmd = msg;

        auto opts = rclcpp_action::Client<MASCmd>::SendGoalOptions();
        opts.result_callback =
            [this](const GoalHandleMASCmd::WrappedResult& result) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Commando ship spawned: %s",
                    result.result->name.c_str());
            };
        mas_action_client_->async_send_goal(goal_msg, opts);
    }

    void spawn_front_target()
    {
        if (!mas_action_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(
                this->get_logger(),
                "MASCmd action server not available");
            return;
        }

        lotusim_msgs::msg::MASCmd msg;
        msg.cmd_type = lotusim_msgs::msg::MASCmd::CREATE_CMD;
        msg.model_name = "lrauv";
        msg.vessel_name = "radar_target";

        geographic_msgs::msg::GeoPoint geo;
        geo.latitude = SPAWN_LATITUDE;
        geo.longitude = SPAWN_LONGITUDE + 0.00009;  // ~10 m east of center
        geo.altitude = SPAWN_ALTITUDE;
        msg.geo_point = geo;

        msg.sdf_string = R"(
        <lotus_param>
            <render_interface>
                <publish_render>false</publish_render>
            </render_interface>
        </lotus_param>
        )";

        auto goal_msg = MASCmd::Goal();
        goal_msg.cmd = msg;

        auto opts = rclcpp_action::Client<MASCmd>::SendGoalOptions();
        opts.result_callback =
            [this](const GoalHandleMASCmd::WrappedResult& result) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "LRAUV target spawned: %s",
                    result.result->name.c_str());
            };
        mas_action_client_->async_send_goal(goal_msg, opts);
    }

    void spawn_lrauv_circle(int n_ships, double radius_m)
    {
        if (!mas_array_action_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(
                this->get_logger(),
                "MASCmdArray action server not available");
            return;
        }

        auto goal_msg = MASCmdArray::Goal();

        for (int i = 0; i < n_ships; ++i) {
            double angle = (2.0 * M_PI / n_ships) * i;
            double lat_offset = (radius_m / 111111.0) * std::cos(angle);
            double lon_offset =
                (radius_m /
                 (111111.0 * std::cos(SPAWN_LATITUDE * M_PI / 180.0))) *
                std::sin(angle);

            lotusim_msgs::msg::MASCmd msg;
            msg.cmd_type = lotusim_msgs::msg::MASCmd::CREATE_CMD;
            msg.model_name = "lrauv";
            msg.vessel_name = "lrauv_" + std::to_string(i);

            geographic_msgs::msg::GeoPoint geo;
            geo.latitude = SPAWN_LATITUDE + lat_offset;
            geo.longitude = SPAWN_LONGITUDE + lon_offset;
            geo.altitude = SPAWN_ALTITUDE;
            msg.geo_point = geo;

            msg.sdf_string = R"(
            <lotus_param>
                <render_interface>
                    <publish_render>false</publish_render>
                </render_interface>
            </lotus_param>
            )";
            goal_msg.cmd.push_back(msg);
        }

        auto opts = rclcpp_action::Client<MASCmdArray>::SendGoalOptions();
        opts.result_callback =
            [this,
             n_ships](const GoalHandleMASCmdArray::WrappedResult& result) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "%d LRAUV ships spawned around commando (status %d)",
                    n_ships,
                    static_cast<int>(result.code));
            };
        mas_array_action_client_->async_send_goal(goal_msg, opts);
    }

private:
    void poses_callback(const VesselPositionArray::SharedPtr msg)
    {
        for (const auto& vessel : msg->vessels) {
            vessel_poses_[vessel.vessel_name] = std::make_pair(
                vessel.geo_point.latitude,
                vessel.geo_point.longitude);
        }
    }

    rclcpp::Subscription<VesselPositionArray>::SharedPtr pose_subscription_;
    rclcpp_action::Client<MASCmd>::SharedPtr mas_action_client_;
    rclcpp_action::Client<MASCmdArray>::SharedPtr mas_array_action_client_;

    std::unordered_map<std::string, std::pair<double, double>> vessel_poses_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RadarTestSpawner>();

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    std::signal(SIGINT, [](int) { g_shutdown_requested = true; });
    std::signal(SIGTERM, [](int) { g_shutdown_requested = true; });

    node->spawn_static_commando();
    node->spawn_front_target();
    node->spawn_lrauv_circle(30, 50.0);

    while (rclcpp::ok() && !g_shutdown_requested) {
        exec.spin_some(100ms);
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down node");
    rclcpp::shutdown();
    return 0;
}
