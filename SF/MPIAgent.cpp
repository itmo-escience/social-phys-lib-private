#include "MPIAgent.h"
#include <string>
#include <iostream>

SF::MPIAgent::MPIAgent()
{
	agent = nullptr;
}

SF::MPIAgent::MPIAgent(Agent* _agent)
{
	agent = _agent;
}

SF::MPIAgent::~MPIAgent()
{
}

unsigned char* SF::MPIAgent::SerializeAgent() const
{
	return agent->Serialize();
}

void SF::MPIAgent::DeserializeAgent(unsigned char* array)
{
	agent = Agent::Deseriaize(array);
}

void SF::MPIAgent::PrintAgentInfo() const
{
	std::string agentInfo = "";
	agentInfo += "ID: " + std::to_string(agent->id_) + "\n";
	agentInfo += "Position: " + std::to_string(agent->position_.x()) + " " + std::to_string(agent->position_.y()) + "\n";
	agentInfo += "Prev Position: " + std::to_string(agent->previosPosition_.x()) + " " + std::to_string(agent->previosPosition_.y()) + "\n";
	agentInfo += "Velocity: " + std::to_string(agent->velocity_.x()) + " " + std::to_string(agent->velocity_.y()) + "\n";
	agentInfo += "Radius: " + std::to_string(agent->radius_) + "\n";
	std::cout << agentInfo << std::endl;
}
