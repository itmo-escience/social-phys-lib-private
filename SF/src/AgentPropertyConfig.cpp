#include "../include/AgentPropertyConfig.h"

namespace SF
{

	AgentPropertyConfig::AgentPropertyConfig(
		float neighborDist, 
		int maxNeighbors, 
		float timeHorizon,
		float obsHorizon,
		float radius, 
		float maxSpeed, 
		float accelerationCoefficient, 
		float repulsiveAgent, 
		float repulsiveAgentFactor, 
		float repulsiveObstacle, 
		float repulsiveObstacleFactor,
		float perception, 
		float friction,
		Vector2 velocity
	) :
		_neighborDist(neighborDist),
		_maxNeighbors(maxNeighbors),
		_timeHorizon(timeHorizon),
		_obsHorizon(obsHorizon),
		_radius(radius),
		_maxSpeed(maxSpeed),
		_accelerationCoefficient(accelerationCoefficient),
		_repulsiveAgent(repulsiveAgent),
		_repulsiveAgentFactor(repulsiveAgentFactor),
		_repulsiveObstacle(repulsiveObstacle),
		_repulsiveObstacleFactor(repulsiveObstacleFactor),
		_perception(perception),
		_friction(friction),
		_velocity(velocity)
	{ }


	AgentPropertyConfig::~AgentPropertyConfig(void)
	{ }

}