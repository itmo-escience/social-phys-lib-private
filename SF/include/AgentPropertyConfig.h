#pragma once

#include "SFSimulator.h"

namespace SF
{
	/// <summary> Defines an agent config </summary>
	class AgentPropertyConfig
	{
	public:
		size_t _maxNeighbors;				// max count of neighbors
		float _neighborDist;				// min distance for neighbors 
		float _timeHorizon;					// iteration time interval
		float _radius;						// range around agent defined by radius 
		float _maxSpeed;					// max speed 
		float _force;						// force
		float _accelerationCoefficient;		// accelereation factor coefficient for acceleration term 
		float _relaxationTime;				// time of approching the max speed  
		float _repulsiveAgent;				// repulsive exponential agent coefficient for agent repulsive force 
		float _repulsiveAgentFactor;		// repulsive factor agent coefficient for agent repulsive force 
		float _repulsiveObstacle;			// repulsive exponential obstacle coefficient for obstacle repulsive force 
		float _repulsiveObstacleFactor;		// repulsive factor obstacle coefficient for obstacle repulsive force 
		float _obstacleRadius;				// min agent to obstacle distance 
		float _platformFactor;				// factor platform coefficient for moving platform force 
		float _perception;					// angle of perception 
		float _friction;					// friction platform coefficient for moving platform force
		Vector2 _velocity;					// current result vector

		/// <summary> Defines an agent config in the simulation </summary>
		AgentPropertyConfig(
			float neighborDist,
			size_t maxNeighbors,
			float timeHorizon,
			float radius,
			float maxSpeed,
			float force,
			float accelerationCoefficient,
			float relaxationTime,
			float repulsiveAgent,
			float repulsiveAgentFactor,
			float repulsiveObstacle,
			float repulsiveObstacleFactor,
			float obstacleRadius,
			float platformFactor,
			float perception, 
			float friction,
			Vector2 velocity
		);

		/// <summary> Defines an agent config in the simulation </summary>
		AgentPropertyConfig();

		/// <summary> Serialize agent properties </summary>
		/// <returns> Serialized agent properties </returns>
		unsigned char * Serialize() const;

		/// <summary> Deserialize agent properties from array </summary>
		/// <param name="angle"> array that contains serialized agent properties </param>
		/// <returns> deserialized agent properties </returns>
		static AgentPropertyConfig* Deseriaize(unsigned char * array);

		/// <summary> Destructor </summary>
		~AgentPropertyConfig(void);

		/// <summary> Print default agent properties </summary>
		void PrintDefaultProperties(void) const;
	};

}