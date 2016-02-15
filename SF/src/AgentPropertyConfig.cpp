#include "../include/AgentPropertyConfig.h"

namespace SF
{
	/// <summary> Defines an agent config </summary>
	AgentPropertyConfig::AgentPropertyConfig(
		float neighborDist,						// min distance for neighbors 
		size_t maxNeighbors,					// max count of neighbors
		float timeHorizon,						// iteration time interval
		float radius,							// range around agent defined by radius 
		float maxSpeed,							// max speed 
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

	/// <summary> Destructor </summary>
	AgentPropertyConfig::~AgentPropertyConfig(void)
	{ }

}