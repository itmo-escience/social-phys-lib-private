#ifndef AGENT_H
#define AGENT_H

#include <map>
#include "Definitions.h"
#include "SFSimulator.h"
#include "Vector3.h"
#include "SimpleMatrix.h"

namespace SF
{
	/// <summary> Defines an agent in the simulation </summary>
	class Agent
	{
	private:
		/// <summary> Defines the ParameterType </summary>
		typedef enum
		{
			X = 1,
			Y,
			Z
		}
		ParameterType;

		/// <summary> Defines the TimeType </summary>
		typedef enum
		{
			PAST = 1,
			PAST2NOW,
			NOW,
			NOW2FUTURE,
			FUTURE
		}
		TimeType;

		/// <summary> Defines an agent in the simulation </summary>
		/// <param name="sim"> The simulator instance </param>
		explicit Agent(SFSimulator* sim);

		/// <summary> Destructor </summary>
		~Agent();

		/// <summary> Computes the neighbors of this agent </summary>
		void computeNeighbors();

		/// <summary> Search for the best new velocity </summary>
		void computeNewVelocity();

		/// <summary> Inserts an agent neighbor into the set of neighbors of this agent </summary>
		/// <param name="agent"> A pointer to the agent to be inserted </param>
		/// <param name="rangeSq"> The squared range around this agent </param>
		void insertAgentNeighbor(const Agent* agent, float& rangeSq);

		/// <summary> Inserts an neighbor agent identifier into the set of neighbors of this agent </summary>
		/// <param name="agent"> A pointer to the agent ID to be inserted </param>
		/// <param name="rangeSq"> The squared range around this agent </param>
		void insertAgentNeighborsIndex(const Agent* agent, float& rangeSq);

		/// <summary> Inserts a static obstacle neighbor into the set of neighbors of this agent </summary>
		/// <param name="agent"> A pointer to the obstacle to be inserted </param>
		/// <param name="rangeSq"> The squared range around this agent </param>
		void insertObstacleNeighbor(const Obstacle* obstacle, float rangeSq);

		/// <summary> Used for acceleration term method calling </summary>
        void update();

		/// <summary> Updates speed list containing speed values corresponding each agent  </summary>
		/// <param name="index"> Agent ID </param>
		/// <param name="value"> New speed value </param>
		void setSpeedList(size_t index, float value);

		/// <summary> Null-initialization for speed list </summary>
		/// <param name="id"> Agent ID </param>
		void setNullSpeed(size_t id);

		/// <summary> Finds perception of some point </summary>
		/// <param name="from"> Agent position </param>
		/// <param name="to"> Position of percepted point </param>
		/// <returns> The angle of perception </returns>
		float getPerception(Vector2 *from, Vector2 *to) const;

		/// <summary> Normalizing the velocity </summary>
		/// <param name="currentSpeed"> Current speed </param>
		/// <param name="maxSpeed"> Max speed </param>
		/// <returns> Normalized speed </returns>
		float getNormalizedSpeed(float currentSpeed, float maxSpeed) const;
		
		/// <summary> Gets point on line nearest to selected position  </summary>
		/// <param name="start"> Position of start of line </param>
		/// <param name="end"> Position of end of line </param>
		/// <param name="point"> Selected point </param>
		/// <returns> The nearest point </returns>
		Vector2 getNearestPoint(Vector2 *start, Vector2 *end, Vector2 *point) const;
    
		/// <summary> Has intersection computing method </summary>
		/// <param name="a"> Start of first line </param>
		/// <param name="b"> End of first line </param>
		/// <param name="c"> Start of second line </param>
		/// <param name="d"> End of second line </param>
		/// <returns> True if two lines has intersection, false elsewhere </returns>
		bool isIntersect(Vector2 a, Vector2 b, Vector2 c, Vector2 d) const;
	
		/// <summary> Gets intersection point </summary>
		/// <param name="a"> Start of first line </param>
		/// <param name="b"> End of first line </param>
		/// <param name="c"> Start of second line </param>
		/// <param name="d"> End of second line </param>
		/// <returns> Intersection point </returns>
		Vector2 getIntersection(Vector2 a, Vector2 b, Vector2 c, Vector2 d) const;

		/// <summary> Acceleration term method </summary>
		void getAccelerationTerm();
	
		/// <summary> Repulsive agent force </summary>
		void getRepulsiveAgentForce();
	
		/// <summary> Repulsive obstacle force </summary>
		void getRepulsiveObstacleForce();
	
		/// <summary> Attractive force </summary>
		void getAttractiveForce();
	
		/// <summary> Moving platform force </summary>
		void getMovingPlatformForce();

		/// <summary> Matrix cross for moving platform </summary>
		/// <param name="left"> Left matrix </param>
		/// <param name="right"> Right matrix </param>
		/// <returns> Result matrix </returns>
		Vector3 getCross(const Vector3 &left, const Vector3 &right) const;

		/// <summary> Degree-to-radian conversion </summary>
		/// <param name="degree"> Degree value </param>
		/// <returns> Radian value </returns>
		double degreesToRadians(float degree) const;

		/// <summary> Radian-to-degree conversion </summary>
		/// <param name="radian"> Radian value </param>
		/// <returns> Degree value </returns>
		double radiansToDegrees(float degree) const;

		/// <summary> Gets current roll </summary>
		/// <param name="pt"> Rotation projection type </param>
		/// <param name="tt"> Rotation time type </param>
		/// <returns> Roll </returns>
		Vector3 getRoll(ParameterType pt, TimeType tt) const;

		/// <summary> Gets Omega matrix </summary>
		/// <param name="pt"> Rotation projection type </param>
		/// <param name="tt"> Rotation time type </param>
		/// <returns> Omega matrix </returns>
		Vector3 getOmega(ParameterType pt, TimeType tt);

		/// <summary> Gets DOmega matrix </summary>
		/// <param name="pt"> Rotation projection type </param>
		/// <param name="tt"> Rotation time type </param>
		/// <returns> DOmega matrix </returns>
		Vector3 getDOmega(ParameterType pt, TimeType tt);

		/// <summary> Gets rotation X matrix </summary>
		/// <param name="angle"> Rotation angle </param>
		/// <returns> Rotation matrix </returns>
		SimpleMatrix getRotationX(float angle) const;

		/// <summary> Gets rotation Y matrix </summary>
		/// <param name="angle"> Rotation angle </param>
		/// <returns> Rotation matrix </returns>
		SimpleMatrix getRotationY(float angle) const;

		/// <summary> Gets rotation Z matrix </summary>
		/// <param name="angle"> Rotation angle </param>
		/// <returns> Rotation matrix </returns>
		SimpleMatrix getRotationZ(float angle) const;

	const size_t MULT = 1000000;
	const size_t SHIFT = 30;
	
		// TODO replace to the new parameter
		const double TOLERANCE = 0.00001f;
	
		bool isDeleted_;														// mark for deleting 
		bool isForced_;															// mark preventing high speed after meeting with the obstacle 
		size_t id_;																// unique identifier 
		size_t maxNeighbors_;													// max count of neighbors
		float acceleration_;													// acceleration buffer preventing high speed after meeting with the obstacle 
		float relaxationTime_;													// time of approching the max speed  
		float maxSpeed_;														// max speed 
		float neighborDist_;													// min distance for neighbors 
		float radius_;															// range around agent defined by radius 
		float timeHorizonObst_;													// iteration time interval
		float accelerationCoefficient_;											// accelereation factor coefficient for acceleration term 
		float repulsiveAgent_;													// repulsive exponential agent coefficient for agent repulsive force 
		float repulsiveAgentFactor_;											// repulsive factor agent coefficient for agent repulsive force 
		float repulsiveObstacle_;												// repulsive exponential obstacle coefficient for obstacle repulsive force 
		float repulsiveObstacleFactor_;											// repulsive factor obstacle coefficient for obstacle repulsive force 
		float obstacleRadius_;													// min agent to obstacle distance 
		float platformFactor_;													// factor platform coefficient for moving platform force 
		float perception_;														// angle of perception 
		float friction_;														// friction platform coefficient for moving platform force
		double obstaclePressure_;												// total pressure for obstacle repulsive force 
		double agentPressure_;													// total pressure for agent repulsive force 
		Vector2 correction;														// current correction vector
		Vector2 newVelocity_;													// new result vector
		Vector2 position_;														// current position
		Vector2 prefVelocity_;													// pre-computed velocity
		Vector2 previosPosition_;												// saved previous position
		Vector2 velocity_;														// current result vector
		Vector2 obstacleTrajectory_;											// graphic representation of result force
		Vector3 oldPlatformVelocity_;											// saved previous platform velocity
		std::vector<std::pair<float, const Obstacle*> > obstacleNeighbors_;		// list of neighbor obstacles
		std::vector<std::pair<float, const Agent*> > agentNeighbors_;			// list of neighbor agents
		std::vector<std::pair<size_t, float>> agentNeighborsIndexList_;			// list of neighbor agent identifiers
		std::vector<float> attractiveTimeList_;									
		std::vector<bool> isUsedAttractivePoint_;								
		std::vector<int> attractiveIds_;										// list of attractive agent identifiers
		std::map<size_t, float> speedList_;										// map of agent speeds
		SFSimulator* sim_;														// simulator instance
    
		friend class KdTree;
		friend class SFSimulator;
	};
}

#endif
