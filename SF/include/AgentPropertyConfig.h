#pragma once

#include "SFSimulator.h"

namespace SF
{

	class AgentPropertyConfig
	{
	public:
		float _neighborDist;
		int _maxNeighbors;
		float _timeHorizon;
		float _obsHorizon;
		float _radius;
		float _maxSpeed;
		float _accelerationCoefficient;
		float _relaxationTime;
		float _repulsiveAgent;
		float _repulsiveAgentFactor;
		float _repulsiveObstacle;
		float _repulsiveObstacleFactor;
		float _platformFactor;
		float _perception;
		float _friction;
		Vector2 _velocity;

		AgentPropertyConfig(
			float neighborDist, 
			int maxNeighbors, 
			float timeHorizon,
			float obsHorizon,
			float radius, 
			float maxSpeed, 
			float accelerationCoefficient, 
			float relaxationTime,
			float repulsiveAgent, 
			float repulsiveAgentFactor, 
			float repulsiveObstacle, 
			float repulsiveObstacleFactor,
			float platformFactor,
			float perception, 
			float friction,
			Vector2 velocity
		);

		~AgentPropertyConfig(void);
	};

}