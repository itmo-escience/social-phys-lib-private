#pragma once
#include "Agent.h"

namespace SF
{
	class MPIAgent
	{

	public:
		Agent* agent;
		MPIAgent();
		MPIAgent(Agent* agent);
		~MPIAgent();
		void DeleteAgent();

		unsigned char* SerializeAgent() const;
		void DeserializeAgent(unsigned char*);
		void PrintAgentInfo() const;

	
		
		double GetToleranceValue() const;
		bool IsDeleted() const;
		bool IsForced() const;
		size_t ID() const;															
		size_t MaxNeighbors() const;													
		float Acceleration() const;													 
		float RelaxationTime() const;													
		float MaxSpeed() const;														
		float Force() const;															
		float NeighborDist() const;													
		float Radius() const;															
		float TimeHorizonObst() const;												
		float AccelerationCoefficient() const;										
		float RepulsiveAgent() const;													
		float RepulsiveAgentFactor() const;											
		float RepulsiveObstacle() const;												
		float RepulsiveObstacleFactor() const;										
		float ObstacleRadius() const;													
		float PlatformFactor() const;													
		float Perception() const;														
		float Friction() const;														
		double ObstaclePressure() const;												
		double AgentPressure() const;													
		Vector2 Correction() const;													
		Vector2 NewVelocity() const;													
		Vector2 Position() const;														
		Vector2 PrefVelocity() const;													
		Vector2 PreviosPosition() const;												
		Vector2 Velocity() const;														
		Vector2 ObstacleForce() const;												
		Vector2 AgentForce() const;													
		Vector3 OldPlatformVelocity() const;											
	};

}
