#include <memory>
#include <random>
#include <sstream>
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

static constexpr double SPAWN_LATITUDE = 1.2605794416293148;
static constexpr double SPAWN_LONGITUDE = 103.7516212463379;
static constexpr double SPAWN_ALTITUDE = 0.0;
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
    using GoalHandleMASCmd = rclcpp_action::ClientGoalHandle<MASCmd>;
    using GoalHandleMASCmdArray = rclcpp_action::ClientGoalHandle<MASCmdArray>;

    ExampleNode() : Node("spawn_ships")
    {
        pose_subscription_ = this->create_subscription<VesselPositionArray>(
            "lotusim/poses",
            rclcpp::QoS(10),
            [this](const VesselPositionArray::SharedPtr msg) {
                this->poses_callback(msg);
            });

        mas_action_client_ =
            rclcpp_action::create_client<MASCmd>(this, "lotusim/mas_cmd");
        mas_array_action_client_ = rclcpp_action::create_client<MASCmdArray>(
            this,
            "lotusim/mas_cmd_array");

        timer_ = this->create_wall_timer(
            1s,
            std::bind(&ExampleNode::print_vessel_positions, this));
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
            msg.vessel_name = "dtmb_" + std::to_string(vessel_id);

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
    rclcpp_action::Client<MASCmd>::SharedPtr mas_action_client_;
    rclcpp_action::Client<MASCmdArray>::SharedPtr mas_array_action_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unordered_map<std::string, std::pair<double, double>> vessel_poses_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ExampleNode>();

    node->spawn_multiple_circling_ship(2);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
