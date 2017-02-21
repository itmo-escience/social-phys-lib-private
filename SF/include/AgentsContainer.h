#ifndef AGENTCONTAINER_H
#define AGENTCONTAINER_H

#include <map>

namespace SF
{
	class SFSimulator;
	class Agent;

	class AgentsContainer
	{
	public:
		AgentsContainer(void);
		~AgentsContainer(void);
		Agent* GetAgentWithID(size_t ID) const;
		size_t AddAgent(Agent* agent);
		bool DeleteAgent(size_t index);

		size_t GetDeadAgentsCount() const;
		std::map<size_t, Agent*> container;
		size_t maximalID;
		std::map<size_t, Agent*>::const_iterator begin() const;
		std::map<size_t, Agent*>::const_iterator end() const;

	private:
		size_t deadAgentsCount;
	};
}
#endif

