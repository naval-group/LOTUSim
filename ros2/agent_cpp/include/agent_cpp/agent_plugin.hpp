#ifndef AGENT_PLUGIN_HH_
#define AGENT_PLUGIN_HH_

#include "agent.hpp"

class AgentPlugin : public Agent{
    private:
    public:
        AgentPlugin(const string & node_name_, const rclcpp::NodeOptions & options)
            : Agent(node_name_, options) {

            };

        bool spawn() {};
        bool despawn() {};
};

#endif