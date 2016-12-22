#include "../include/AgentPropertyConfig.h"
#include <iostream>
#include <string>

namespace SF
{
	/// <summary> Defines an agent config </summary>
	AgentPropertyConfig::AgentPropertyConfig(
		float neighborDist,						// min distance for neighbors 
		size_t maxNeighbors,					// max count of neighbors
		float timeHorizon,						// iteration time interval
		float radius,							// range around agent defined by radius 
		float maxSpeed,							// max speed 
		float force,							// force
		float accelerationCoefficient,			// accelereation factor coefficient for acceleration term 
		float relaxationTime,					// time of approching the max speed  
		float repulsiveAgent,					// repulsive exponential agent coefficient for agent repulsive force 
		float repulsiveAgentFactor,				// repulsive factor agent coefficient for agent repulsive force 
		float repulsiveObstacle,				// repulsive exponential obstacle coefficient for obstacle repulsive force 
		float repulsiveObstacleFactor,			// repulsive factor obstacle coefficient for obstacle repulsive force 
		float obstacleRadius,					// min agent to obstacle distance 
		float platformFactor,					// factor platform coefficient for moving platform force 
		float perception,						// angle of perception	
		float friction,							// friction platform coefficient for moving platform force
		Vector2 velocity						// current result vector
		) :
		_neighborDist(neighborDist),
		_maxNeighbors(maxNeighbors),
		_timeHorizon(timeHorizon),
		_radius(radius),
		_maxSpeed(maxSpeed),
		_force(force),
		_accelerationCoefficient(accelerationCoefficient),
		_relaxationTime(relaxationTime),
		_repulsiveAgent(repulsiveAgent),
		_repulsiveAgentFactor(repulsiveAgentFactor),
		_repulsiveObstacle(repulsiveObstacle),
		_repulsiveObstacleFactor(repulsiveObstacleFactor),
		_obstacleRadius(obstacleRadius),
		_platformFactor(platformFactor),
		_perception(perception),
		_friction(friction),
		_velocity(velocity)
	{ }

	/// <summary> Defines an agent config </summary>
	AgentPropertyConfig::AgentPropertyConfig() :
		_neighborDist(0),
		_maxNeighbors(0),
		_timeHorizon(0),
		_radius(0),
		_maxSpeed(0),
		_accelerationCoefficient(0),
		_relaxationTime(0),
		_repulsiveAgent(0),
		_repulsiveAgentFactor(0),
		_repulsiveObstacle(0),
		_repulsiveObstacleFactor(0),
		_obstacleRadius(0),
		_platformFactor(0),
		_perception(0),
		_friction(0),
		_velocity(Vector2(0, 0))
	{ }

	/// <summary> Destructor </summary>
	AgentPropertyConfig::~AgentPropertyConfig(void)
	{ }

	/// <summary> Serialize agent properties </summary>
	/// <returns> Serialized agent properties </returns>
	unsigned char * AgentPropertyConfig::Serialize() const
	{
		size_t bufferSize = sizeof(size_t) + sizeof(*this);
		// The buffer we will be writing bytes into
		unsigned char* outBuf = new unsigned char[bufferSize];

		// A pointer we will advance whenever we write data
		unsigned char* p = outBuf;
		
		memcpy(p, &bufferSize, sizeof(bufferSize));
		p += sizeof(bufferSize);

		memcpy(p, &this->_neighborDist, sizeof(_neighborDist));
		p += sizeof(_neighborDist);

		memcpy(p, &this->_maxNeighbors, sizeof(_maxNeighbors));
		p += sizeof(_maxNeighbors);

		memcpy(p, &this->_timeHorizon, sizeof(_timeHorizon));
		p += sizeof(_timeHorizon);

		memcpy(p, &this->_radius, sizeof(_radius));
		p += sizeof(_radius);

		memcpy(p, &this->_maxSpeed, sizeof(_maxSpeed));
		p += sizeof(_maxSpeed);

		memcpy(p, &this->_force, sizeof(_force));
		p += sizeof(_force);

		memcpy(p, &this->_accelerationCoefficient, sizeof(_accelerationCoefficient));
		p += sizeof(_accelerationCoefficient);

		memcpy(p, &this->_relaxationTime, sizeof(_relaxationTime));
		p += sizeof(_relaxationTime);

		memcpy(p, &this->_repulsiveAgent, sizeof(_repulsiveAgent));
		p += sizeof(_repulsiveAgent);

		memcpy(p, &this->_repulsiveAgentFactor, sizeof(_repulsiveAgentFactor));
		p += sizeof(_repulsiveAgentFactor);

		memcpy(p, &this->_repulsiveObstacle, sizeof(_repulsiveObstacle));
		p += sizeof(_repulsiveObstacle);

		memcpy(p, &this->_repulsiveObstacleFactor, sizeof(_repulsiveObstacleFactor));
		p += sizeof(_repulsiveObstacleFactor);

		memcpy(p, &this->_obstacleRadius, sizeof(_obstacleRadius));
		p += sizeof(_obstacleRadius);

		memcpy(p, &this->_platformFactor, sizeof(_platformFactor));
		p += sizeof(_platformFactor);

		memcpy(p, &this->_perception, sizeof(_perception));
		p += sizeof(_perception);

		memcpy(p, &this->_friction, sizeof(_friction));
		p += sizeof(_friction);

		memcpy(p, &this->_velocity, sizeof(_velocity));
		p += sizeof(_velocity);

		return outBuf;
	}

	/// <summary> Deserialize agent properties from array </summary>
	/// <param name="angle"> array that contains serialized agent properties </param>
	/// <returns> deserialized agent properties </returns>
	AgentPropertyConfig* AgentPropertyConfig::Deseriaize(unsigned char * array)
	{
		// A pointer we will advance whenever we write data
		unsigned char* p = array;
		AgentPropertyConfig* agentPropertyConfig = new AgentPropertyConfig();

		size_t buffSize = 0;
		memcpy(&buffSize, p, sizeof(buffSize));
		p += sizeof(buffSize);

		memcpy(&agentPropertyConfig->_neighborDist, p, sizeof(float));
		p += sizeof(float);

		memcpy(&agentPropertyConfig->_maxNeighbors, p, sizeof(size_t));
		p += sizeof(size_t);

		memcpy(&agentPropertyConfig->_timeHorizon, p, sizeof(float));
		p += sizeof(float);

		memcpy(&agentPropertyConfig->_radius, p, sizeof(float));
		p += sizeof(float);

		memcpy(&agentPropertyConfig->_maxSpeed, p, sizeof(float));
		p += sizeof(float);

		memcpy(&agentPropertyConfig->_force, p, sizeof(float));
		p += sizeof(float);

		memcpy(&agentPropertyConfig->_accelerationCoefficient, p, sizeof(float));
		p += sizeof(float);

		memcpy(&agentPropertyConfig->_relaxationTime, p, sizeof(float));
		p += sizeof(float);

		memcpy(&agentPropertyConfig->_repulsiveAgent, p, sizeof(float));
		p += sizeof(float);

		memcpy(&agentPropertyConfig->_repulsiveAgentFactor, p, sizeof(float));
		p += sizeof(float);

		memcpy(&agentPropertyConfig->_repulsiveObstacle, p, sizeof(float));
		p += sizeof(float);

		memcpy(&agentPropertyConfig->_repulsiveObstacleFactor, p, sizeof(float));
		p += sizeof(float);

		memcpy(&agentPropertyConfig->_obstacleRadius, p, sizeof(float));
		p += sizeof(float);

		memcpy(&agentPropertyConfig->_platformFactor, p, sizeof(float));
		p += sizeof(float);

		memcpy(&agentPropertyConfig->_perception, p, sizeof(float));
		p += sizeof(float);

		memcpy(&agentPropertyConfig->_friction, p, sizeof(float));
		p += sizeof(float);

		memcpy(&agentPropertyConfig->_velocity, p, sizeof(float) * 2);
		p += sizeof(sizeof(float) * 2);

		return agentPropertyConfig;
	}

	/// <summary> Print default agent properties </summary>
	void AgentPropertyConfig::PrintDefaultProperties(void) const
	{
		std::string propertiesString = "";
		propertiesString += "Acceleration coeff: " + patch::to_string((long double)this->_accelerationCoefficient) + "\n";
		propertiesString += "Friction: " + patch::to_string((long double)this->_friction) + "\n";
		propertiesString += "Radius: " + patch::to_string((long double)this->_radius) + "\n";
		propertiesString += "Perception: " + patch::to_string((long double)this->_perception) + "\n";
		propertiesString += "Force: " + patch::to_string((long double)this->_force) + "\n";
		propertiesString += "Max neigh: " + patch::to_string((long double)this->_maxNeighbors) + "\n";
		propertiesString += "Max speed: " + patch::to_string((long double)this->_maxSpeed) + "\n";
		propertiesString += "Obstacle radius: " + patch::to_string((long double)this->_obstacleRadius) + "\n";
		propertiesString += "Relax time: " + patch::to_string((long double)this->_relaxationTime) + "\n";
		propertiesString += "Repulsive agent: " + patch::to_string((long double)this->_repulsiveAgent) + "\n";
		propertiesString += "Repulsive agent factor: " + patch::to_string((long double)this->_repulsiveAgentFactor) + "\n";
		propertiesString += "Repulsive obstacle: " + patch::to_string((long double)this->_repulsiveObstacle) + "\n";
		propertiesString += "Repulsive obstacle factor: " + patch::to_string((long double)this->_repulsiveObstacleFactor) + "\n";
		std::cout << propertiesString << std::endl;
	}
}
