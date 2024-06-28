#include "agent.hpp"
#include "agent_entity.hpp"

class AgentFactory {
public:
    virtual Agent::UniquePtr createAgent() = 0;
};

class AgentEntityFactory : public AgentFactory {
public:
    AgentEntity::UniquePtr createAgent() override
    {
        return std::make_unique<AgentEntity>();
    }
};