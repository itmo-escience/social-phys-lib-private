// SF2DWrapper.h

#pragma once

#include "../../SF/include/SFSimulator.h"
#include "../../SF/include/AgentPropertyConfig.h"
#include "SFVector2.h"
//#include "SFVector3.h"
#include "SFRotationDegreeSet.h"

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
		SFVector2 Velocity;

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
			SFVector2 velocity
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
		SFVector2 _velocity;
	};


	public ref class SFSimulator
	{
		private:
				SF::SFSimulator* _sim;
			public:
				SFSimulator();	
				~SFSimulator();

				int addAgent(SFVector2 position);
				
				int addAgent(
					SFVector2 position, 
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
					SFVector2 velocity 
				);
				
				void doStep();
				int getAgentAgentNeighbor(int agentNo, int neighborNo) ;
				int getAgentMaxNeighbors(int agentNo) ;
				float getAgentMaxSpeed(int agentNo) ;
				float getAgentNeighborDist(int agentNo) ;
				int getAgentNumAgentNeighbors(int agentNo) ;
				SFVector2  getAgentPosition(int agentNo) ;
				SFVector2  getAgentPrefVelocity(int agentNo) ;
				void getAgentPositions(array<int>^ agentNos, array<SFVector2>^ positions) ;
				void getAgentPrefVelocities(array<int>^ agentNos, array<SFVector2>^ prefVelocities) ;
				float getAgentRadius(int agentNo) ;
				SFVector2 getAgentVelocity(int agentNo) ;
				float getGlobalTime() ;
				int getNumAgents() ;
				float getTimeStep() ;
				void setAgentDefaults(AgentProperty^ ap);
				void setAgentMaxNeighbors(int agentNo, int maxNeighbors);
				void setAgentMaxSpeed(int agentNo, float maxSpeed);
				void setAgentNeighborDist(int agentNo, float neighborDist);
				void setAgentPosition(int agentNo,  SFVector2 position);
				void setAgentPrefVelocity(int agentNo,  SFVector2 prefVelocity);
				void setAgentPrefVelocity(array<int>^ agentNo,  array<SFVector2>^ prefVelocity);
				void setAgentRadius(int agentNo, float radius);
				void setAgentVelocity(int agentNo,  SFVector2 velocity);
				void setAgentVelocity(array<int>^ agentNo,  array<SFVector2>^ velocity);
				void setTimeStep(float timeStep);	
				void setAdditionalForce(SF3D::SFVector3 velocity, SF3D::SFRotationDegreeSet set);
				
				void setAttractiveForce(
					float attractiveStrength, 
					float repulsiveStrength, 
					float attractiveRange, 
					float repulsiveRange, 
					float attractiveTime_,
					float length
				);
				
				void setAttractiveIdList(
					int id,
					System::Collections::Generic::List<int>^ attractiveIds
				);

				void addAttractiveId(
					int id,
					int newId
				);

				void addAttractiveIdList(
					int id,
					System::Collections::Generic::List<int>^ attractiveIds
				);

				void deleteAttractiveId(
					int id,
					int idForDelete
				);

				void deleteAttractiveIdList(
					int id,
					System::Collections::Generic::List<int>^ attractiveIds
				);

				void setPlatformVelocity(SF3D::SFVector3 velocity);
				SF3D::SFVector3 getPlatformVelocity();
				float getAgentFriction(int agentNo);
				void setAgentFriction(int agentNo, float friction);
				void setRotationDegreeSet(SF3D::SFRotationDegreeSet set);
				SF3D::SFRotationDegreeSet getRotationDegreeSet();
				int addObstacle(array<SFVector2>^ vertices);
				int addObstacle(System::Collections::Generic::List<SFVector2>^ vertices);
				void processObstacles();
				bool queryVisibility(SFVector2 point1, SFVector2 point2, float radius) ;
				float getAgentTimeHorizonObst(int agentNo) ;
				void setAgentTimeHorizonObst(int agentNo, float timeHorizonObst);			
				System::Collections::Generic::List<int>^ getAgentNeighboursIndexList(int agentNo, float radius);
				double getObstaclePressure(int index);
				SF2D::SFVector2 getObstacleTrajectory(int index);
				double getAgentPressure(int index);
				void deleteAgent(int index);
				System::Collections::Generic::List<int>^ getDeletedIDList();
				System::Collections::Generic::List<int>^ getCountOfAliveAndDead();
				System::Collections::Generic::List<double>^ getAgentPressureList();
				System::Collections::Generic::List<double>^ getObstaclePressureList();
				System::Collections::Generic::List<SFVector2>^ getPositionList();

				void updateSFParameters(float newRepulsiveAgent, float newRepulsiveAgentFactor, float newRepulsiveObstacle, float newRepulsiveObstacleFactor);
	};
}
