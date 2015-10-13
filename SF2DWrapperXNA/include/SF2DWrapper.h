// SF2DWrapper.h

#pragma once

#include "../../SF/include/SFSimulator.h"

using namespace System;

namespace SF2D {

	
	public ref class AgentProperty
	{
	public:
		float NeighborDist;
		int MaxNeighbors;
		float TimeHorizon;
		float ObsHorizon;
		float Radius;
		float MaxSpeed;
		float AccelerationCoefficient;
		float RelaxationTime;
		float RepulsiveAgent;
		float RepulsiveAgentFactor;
		float RepulsiveObstacle;
		float RepulsiveObstacleFactor;
		float ObstacleRadius;
		float PlatformFactor;
		float Perception;
		float Friction;
		Microsoft::Xna::Framework::Vector2 Velocity;

		AgentProperty();

		AgentProperty(
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
			float obstacleRadius,
			float platformFactor,
			float perception, 
			float friction,
			Microsoft::Xna::Framework::Vector2 velocity
		);
	
	private:
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
		float _obstacleRadius;
		float _platformFactor;
		float _perception;
		float _friction;
		Microsoft::Xna::Framework::Vector2 _velocity;
	};


	public ref class SFSimulator
	{
		private:
				SF::SFSimulator* _sim;
			public:
				SFSimulator();	
				~SFSimulator();

				int addAgent(Microsoft::Xna::Framework::Vector2 position);

				int addAgent(
					Microsoft::Xna::Framework::Vector2 position, 
					float neighborDist, 
					int maxNeighbors, 
					float timeHorizon,
					float timeHorizonObst, 
					float radius,
					float maxSpeed, 
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
					Microsoft::Xna::Framework::Vector2 velocity 
				);
				
				void doStep();

				int getAgentAgentNeighbor(int agentNo, int neighborNo) ;
				int getAgentMaxNeighbors(int agentNo) ;
				float getAgentMaxSpeed(int agentNo) ;
				float getAgentNeighborDist(int agentNo) ;
				int getAgentNumAgentNeighbors(int agentNo) ;
				Microsoft::Xna::Framework::Vector2  getAgentPosition(int agentNo) ;
				Microsoft::Xna::Framework::Vector2  getAgentPrefVelocity(int agentNo) ;
				void getAgentPositions(array<int>^ agentNos, array<Microsoft::Xna::Framework::Vector2>^ positions) ;
				void getAgentPrefVelocities(array<int>^ agentNos, array<Microsoft::Xna::Framework::Vector2>^ prefVelocities) ;
				float getAgentRadius(int agentNo) ;
				Microsoft::Xna::Framework::Vector2 getAgentVelocity(int agentNo) ;
				float getGlobalTime() ;
				int getNumAgents() ;
				float getTimeStep() ;
				void setAgentDefaults(AgentProperty^ ap);
				void setAgentMaxNeighbors(int agentNo, int maxNeighbors);
				void setAgentMaxSpeed(int agentNo, float maxSpeed);
				void setAgentNeighborDist(int agentNo, float neighborDist);
				void setAgentPosition(int agentNo,  Microsoft::Xna::Framework::Vector2 position);
				void setAgentPrefVelocity(int agentNo,  Microsoft::Xna::Framework::Vector2 prefVelocity);
				void setAgentPrefVelocity(array<int>^ agentNo,  array<Microsoft::Xna::Framework::Vector2>^ prefVelocity);
				void setAgentRadius(int agentNo, float radius);
				void setAgentVelocity(int agentNo,  Microsoft::Xna::Framework::Vector2 velocity);
				void setAgentVelocity(array<int>^ agentNo,  array<Microsoft::Xna::Framework::Vector2>^ velocity);
				void setTimeStep(float timeStep);	
				int addObstacle(array<Microsoft::Xna::Framework::Vector2>^ vertices);
				int addObstacle(System::Collections::Generic::List<Microsoft::Xna::Framework::Vector2>^ vertices);
				void processObstacles();
				bool queryVisibility(Microsoft::Xna::Framework::Vector2 point1, Microsoft::Xna::Framework::Vector2 point2, float radius) ;
				float getAgentTimeHorizonObst(int agentNo) ;
				void setAgentTimeHorizonObst(int agentNo, float timeHorizonObst);
				double getAgentPressure(int index);
				double getObstaclePressure(int index);
	};
}
