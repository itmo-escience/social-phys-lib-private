#include "../include/AgentsContainer.h"


SF::AgentsContainer::AgentsContainer()
{
	maximalID = 0;
	deadAgentsCount = 0;
}


SF::Agent* SF::AgentsContainer::GetAgentWithID(size_t ID) const
{
	std::map<size_t, SF::Agent*>::const_iterator agent = this->container.find(ID);
	if(agent != container.end())
	{
		return agent->second;	
	}
	return NULL;
}

SF::AgentsContainer::~AgentsContainer()
{

}


std::map<size_t, SF::Agent*>::const_iterator SF::AgentsContainer::begin() const
{
	return container.begin();
}

std::map<size_t, SF::Agent*>::const_iterator SF::AgentsContainer::end() const
{
	return container.end();
}

size_t SF::AgentsContainer::AddAgent(SF::Agent* agent)
{
	container[maximalID] = agent;
	size_t agentId = maximalID;
	maximalID++;
	return agentId;
}

bool SF::AgentsContainer::DeleteAgent(size_t index)
{
	std::map<size_t, SF::Agent*>::iterator it = container.find(index);
	if(it != container.end())
	{
		container.erase(it);
		deadAgentsCount++;
		return true;
	}
	return false;
}

size_t SF::AgentsContainer::GetDeadAgentsCount() const
{
	return deadAgentsCount;
}