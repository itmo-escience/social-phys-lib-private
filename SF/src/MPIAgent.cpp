#include <string>
#include <iostream>
#include "../include/MPIAgent.h"
#include "../include/Agent.h"

SF::MPIAgent::MPIAgent()
{
	agent = NULL;
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

void SF::MPIAgent::DeleteAgent()
{
	delete agent;
}

void SF::MPIAgent::DeserializeAgent(unsigned char* array)
{
	agent = Agent::Deseriaize(array);
}

void SF::MPIAgent::PrintAgentInfo() const
{
	std::string agentInfo = "";
	agentInfo += "ID: " + patch::to_string((long double)agent->id_) + "\n";
	agentInfo += "Position: " + patch::to_string((long double)agent->position_.x()) + " " + patch::to_string((long double)agent->position_.y()) + "\n";
	agentInfo += "Prev Position: " + patch::to_string((long double)agent->previosPosition_.x()) + " " + patch::to_string((long double)agent->previosPosition_.y()) + "\n";
	agentInfo += "Velocity: " + patch::to_string((long double)agent->velocity_.x()) + " " + patch::to_string((long double)agent->velocity_.y()) + "\n";
	agentInfo += "Radius: " + patch::to_string((long double)agent->radius_) + "\n";
	std::cout << agentInfo << std::endl;
}

double SF::MPIAgent::GetToleranceValue() const
{
	return agent->TOLERANCE;
};

bool SF::MPIAgent::IsDeleted() const
{
	return agent->isDeleted_;
};

bool SF::MPIAgent::IsForced() const
{
	return agent->isForced_;
};

size_t SF::MPIAgent::ID() const
{
	return agent->id_;
};	

size_t SF::MPIAgent::MaxNeighbors() const
{
	return agent->maxNeighbors_;
};

float SF::MPIAgent::Acceleration() const
{
	return agent->acceleration_;
};

float SF::MPIAgent::RelaxationTime() const
{
	return agent->relaxationTime_;
};

float SF::MPIAgent::MaxSpeed() const
{
	return agent->maxSpeed_;
};	

float SF::MPIAgent::Force() const
{
	return agent->force_;
};

float SF::MPIAgent::NeighborDist() const
{
	return agent->neighborDist_;
};	

float SF::MPIAgent::Radius() const
{
	return agent->radius_;
};	

float SF::MPIAgent::TimeHorizonObst() const
{
	return agent->timeHorizonObst_;
};	

float SF::MPIAgent::AccelerationCoefficient() const
{
	return agent->accelerationCoefficient_;
};	

float SF::MPIAgent::RepulsiveAgent() const
{
	return agent->repulsiveAgent_;
};	

float SF::MPIAgent::RepulsiveAgentFactor() const
{
	return agent->repulsiveAgentFactor_;
};

float SF::MPIAgent::RepulsiveObstacle() const
{
	return agent->repulsiveObstacle_;
};

float SF::MPIAgent::RepulsiveObstacleFactor() const
{
	return agent->repulsiveObstacleFactor_;
};	

float SF::MPIAgent::ObstacleRadius() const
{
	return agent->obstacleRadius_;
};	

float SF::MPIAgent::PlatformFactor() const
{
	return agent->platformFactor_;
};

float SF::MPIAgent::Perception() const
{
	return agent->perception_;
};	

float SF::MPIAgent::Friction() const
{
	return agent->friction_;
};	

double SF::MPIAgent::ObstaclePressure() const
{
	return agent->obstaclePressure_;
};

double SF::MPIAgent::AgentPressure() const
{
	return agent->agentPressure_;
};

SF::Vector2 SF::MPIAgent::Correction() const
{
	return agent->correction;
};

SF::Vector2 SF::MPIAgent::NewVelocity() const
{
	return agent->newVelocity_;
};

SF::Vector2 SF::MPIAgent::Position() const
{
	return agent->position_;
};

SF::Vector2 SF::MPIAgent::PrefVelocity() const
{
	return agent->prefVelocity_;
};

SF::Vector2 SF::MPIAgent::PreviosPosition() const
{
	return agent->previosPosition_;
};

SF::Vector2 SF::MPIAgent::Velocity() const
{
	return agent->velocity_;
};

SF::Vector2 SF::MPIAgent::ObstacleForce() const
{
	return agent->obstacleForce_;
};

SF::Vector2 SF::MPIAgent::AgentForce() const
{
	return agent->agentForce_;
};

SF::Vector3 SF::MPIAgent::OldPlatformVelocity() const
{
	return agent->oldPlatformVelocity_;
};