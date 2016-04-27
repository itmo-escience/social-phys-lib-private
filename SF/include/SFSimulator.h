#ifndef SF_SIMULATOR_H
#define SF_SIMULATOR_H

#include <limits>
#include <vector>

#include "Vector2.h"
#include "Vector3.h"
#include "AgentPropertyConfig.h"
#include "RotationDegreeSet.h"

namespace SF
{
	static const size_t SF_ERROR = std::numeric_limits<size_t>::max();	// error value
  
	/// <summary> Defines a directed line </summary>
	struct Line 
	{
		/// <summary> A point on the directed line </summary>
		Vector2 point;
       
		/// <summary> The direction of the directed line </summary>
		Vector2 direction;
	};

	class Agent;
	class KdTree;
	class Obstacle;
	class AgentPropertyConfig;
	class RotationDegreeSet;

	/// <summary> The main class of the library that contains all simulation functionality </summary>
	class SFSimulator
	{
	public:
		/// <summary> Constructs a simulator instance </summary>
		SFSimulator();

		/// <summary> Destroys this simulator instance </summary>
		~SFSimulator();

		/// <summary> Adds a new agent with default properties to the simulation </summary>
		/// <param name="position"> The two-dimensional starting position of this agent </param>
		/// <returns> The number of the agent, or SF::SF_ERROR when the agent defaults have not been set </returns>
		size_t addAgent(const Vector2& position);

		/// <summary> Adds a new agent to the simulation </summary>
		/// <param name="position"> The two-dimensional starting position of this agent </param>
		/// <param name="neighborDist"> The maximal distance (center point to center point) to other agents this agent takes into account in the navigation. The larger this number, the longer the running time of the simulation. If the number is too low, the simulation will not be safe. Must be non - negative </param>
		/// <param name="maxNeighbors"> The maximal number of other agents this agent takes into account in the navigation. The larger this number, the longer the running time of the simulation. If the number is too low, the simulation will not be safe </param>
		/// <param name="timeHorizon"> The minimal amount of time for which this agent's velocities that are computed by the simulation are safe with respect to other agents.The larger this number, the sooner this agent will respond to the presence of other agents, but the less freedom this agent has in choosing its velocities. Must be positive </param>
		/// <param name="radius"> The radius of this agent </param>
		/// <param name="maxSpeed"> The maximal speed of this agent </param>
		/// <param name="velocity"> The initial two-dimensional linear velocity of this agent (optional) </param>
		/// <param name="accelerationCoefficient"> Accelereation factor coefficient for acceleration term </param>
		/// <param name="relaxationTime"> Time of approching the max speed </param>
		/// <param name="repulsiveAgent"> Repulsive exponential agent coefficient for agent repulsive force </param>
		/// <param name="repulsiveAgentFactor"> Repulsive factor agent coefficient for agent repulsive force  </param>
		/// <param name="repulsiveObstacle"> Repulsive exponential obstacle coefficient for obstacle repulsive force </param>
		/// <param name="repulsiveObstacleFactor"> Repulsive factor obstacle coefficient for obstacle repulsive force </param>
		/// <param name="obstacleRadius"> Min agent to obstacle distance </param>
		/// <param name="platformFactor"> Factor platform coefficient for moving platform force  </param>
		/// <param name="perception"> Angle of perception  </param>
		/// <param name="friction"> Friction platform coefficient for moving platform force </param>
		/// <returns> The number of the agent</returns>
		size_t addAgent(
			const Vector2& position, 
			float neighborDist,
			size_t maxNeighbors, 
			float timeHorizon,
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
			const Vector2& velocity = Vector2()
		);
				
		/// <summary> Adds a new obstacle to the simulation </summary>
		/// <param name="vertices"> List of the vertices of the polygonal obstacle in counterclockwise order </param>
		/// <returns> The number of the first vertex of the obstacle, or SF::SF_ERROR when the number of vertices is less than two</returns>
		size_t addObstacle(const std::vector<Vector2>& vertices);

		/// <summary> Lets the simulator perform a simulation step and updates the two - dimensional position and two - dimensional velocity of each agent </summary>
		void doStep();

		/// <summary> Returns the specified agent neighbor of the specified agent </summary>
		/// <param name="agentNo"> The number of the agent whose agent neighbor is to be retrieved </param>
		/// <param name="neighborNo"> The number of the agent neighbor to be retrieved </param>
		/// <returns> The number of the neighboring agent </returns>
		size_t getAgentAgentNeighbor(size_t agentNo, size_t neighborNo) const;
    
		/// <summary> Returns the maximum neighbor count of a specified agent </summary>
		/// <param name="agentNo"> The number of the agent whose maximum neighbor count is to be retrieved </param>
		/// <returns> The present maximum neighbor count of the agent </returns>
		size_t getAgentMaxNeighbors(size_t agentNo) const;

		/// <summary> Returns the maximum speed of a specified agent </summary>
		/// <param name="agentNo"> The number of the agent whose maximum speed is to be retrieved </param>
		/// <returns> The present maximum speed of the agent </returns>
		float getAgentMaxSpeed(size_t agentNo) const;

		/// <summary> Returns the maximum neighbor distance of a specified agent </summary>
		/// <param name="agentNo"> The number of the agent whose maximum neighbor distance is to be retrieved </param>
		/// <returns> The present maximum neighbor distance of the agent</returns>
		float getAgentNeighborDist(size_t agentNo) const;

		/// <summary> Returns the count of agent neighbors taken into account to compute the current velocity for the specified agent </summary>
		/// <param name="agentNo"> The number of the agent whose count of agent neighbors is to be retrieved </param>
		/// <returns> The count of agent neighbors taken into account to compute the current velocity for the specified agent </returns>
		size_t getAgentNumAgentNeighbors(size_t agentNo) const;

		/// <summary> Returns the count of obstacle neighbors taken into account to compute the current velocity for the specified agent </summary>
		/// <param name="agentNo"> The number of the agent whose count of obstacle neighbors is to be retrieved </param>
		/// <returns> The count of obstacle neighbors taken into account to compute the current velocity for the specified agent </returns>
		size_t getAgentNumObstacleNeighbors(size_t agentNo) const;

		/// <summary> Returns the specified obstacle neighbor of the specified agent </summary>
		/// <param name="agentNo"> The number of the agent whose obstacle neighbor is to be retrieved </param>
		/// <param name="neighborNo"> The number of the obstacle neighbor to be retrieved </param>
		/// <returns> The number of the first vertex of the neighboring obstacle edge </returns>
		size_t getAgentObstacleNeighbor(size_t agentNo, size_t neighborNo) const;

		/// <summary> Returns the two-dimensional position of a specified agent </summary>
		/// <param name="agentNo"> The number of the agent whose two - dimensional position is to be retrieved </param>
		/// <returns> The present two-dimensional position of the (center of the) agent </returns>
		const Vector2& getAgentPosition(size_t agentNo) const;

		/// <summary> Returns the two-dimensional preferred velocity  of a specified agent </summary>
		/// <param name="agentNo"> The number of the agent whose two-dimensional preferred velocity is to be retrieved </param>
		/// <returns> The present two-dimensional of the agent </returns>
		const Vector2& getAgentPrefVelocity(size_t agentNo) const;

		/// <summary> Returns the radius of a specified agent </summary>
		/// <param name="agentNo"> The number of the agent whose radius is to be retrieved </param>
		/// <returns> The present radius of the agent </returns>
		float getAgentRadius(size_t agentNo) const;

		/// <summary> Returns the time horizon with respect to obstacles of a specified agent </summary>
		/// <param name="agentNo"> The number of the agent whose time horizon with respect to obstacles is to be retrieved </param>
		/// <returns> The present time horizon with respect to obstacles of the agent </returns>
		float getAgentTimeHorizonObst(size_t agentNo) const;

		/// <summary> Returns the two-dimensional linear velocity of a specified agent </summary>
		/// <param name="agentNo"> The number of the agent whose two - dimensional linear velocity is to be retrieved </param>
		/// <returns> The present two-dimensional linear velocity of the agent </returns>
		const Vector2& getAgentVelocity(size_t agentNo) const;

		/// <summary> Returns the global time of the simulation </summary>
		/// <returns> The present global time of the simulation (zero initially) </returns>
		float getGlobalTime() const;

		/// <summary> Returns the count of agents in the simulation </summary>
		/// <returns> The count of agents in the simulation </returns>
		size_t getNumAgents() const;

		/// <summary> Returns the count of obstacle vertices in the simulation </summary>
		/// <returns> The count of obstacle vertices in the simulation </returns>
		size_t getNumObstacleVertices() const;

		/// <summary> Returns the two-dimensional position of a specified obstacle vertex </summary>
		/// <param name="vertexNo"> The number of the obstacle vertex to be retrieved </param>
		/// <returns> The two-dimensional position of the specified obstacle vertex </returns>
		const Vector2& getObstacleVertex(size_t vertexNo) const;

		/// <summary> Returns the number of the obstacle vertex succeeding the specified obstacle vertex in its polygon </summary>
		/// <param name="vertexNo"> The number of the obstacle vertex whose successor is to be retrieved </param>
		/// <returns> The number of the obstacle vertex succeeding the specified obstacle vertex in its polygon</returns>
		size_t getNextObstacleVertexNo(size_t vertexNo) const;

		/// <summary> Returns the number of the obstacle vertex preceding the specified obstacle vertex in its polygon </summary>
		/// <param name="agentNo"> The number of the obstacle vertex whose predecessor is to be retrieved </param>
		/// <returns> The number of the obstacle vertex preceding the specified obstacle vertex in its polygon </returns>
		size_t getPrevObstacleVertexNo(size_t vertexNo) const;

		/// <summary> Returns the time step of the simulation </summary>
		/// <returns> The present time step of the simulation </returns>
		float getTimeStep() const;

		/// <summary> Processes the obstacles that have been added so that they are accounted for in the simulation </summary>
		void processObstacles() const;

		/// <summary> Performs a visibility query between the two specified points with respect to the obstacles </summary>
		/// <param name="point1"> The first point of the query </param>
		/// <param name="point2"> The second point of the query </param>
		/// <param name="radius"> The minimal distance between the line connecting the two points and the obstacles in order for the points to be mutually visible(optional). Must be non - negative </param>
		/// <returns> A boolean specifying whether the two points are mutually visible. Returns true when the obstacles have not been processed </returns>
		bool queryVisibility(
			const Vector2& point1, 
			const Vector2& point2, 
			float radius = 0.0f
		) const;

		/// <summary> Sets default property of agent</summary>
		/// <param name="apc"> Property </param>
		void setAgentDefaults(AgentPropertyConfig & apc);

		/// <summary> Sets the maximum neighbor count of a specified agent </summary>
		/// <param name="agentNo"> The number of the agent whose maximum neighbor count is to be modified </param>
		/// <param name="maxNeighbors"> The replacement maximum neighbor count </param>
		void setAgentMaxNeighbors(size_t agentNo, size_t maxNeighbors);

		/// <summary> Sets the maximum speed of a specified agent </summary>
		/// <param name="agentNo"> The number of the agent whose maximum speed is to be modified </param>
		/// <param name="maxSpeed"> The replacement maximum speed. Must be non - negative </param>
		void setAgentMaxSpeed(size_t agentNo, float maxSpeed);

		/// <summary> Sets the maximum neighbor distance of a specified agent </summary>
		/// <param name="agentNo"> The number of the agent whose maximum neighbor distance is to be modified </param>
		/// <param name="neighborDist"> The replacement maximum neighbor distance. Must be non - negative</param>
		void setAgentNeighborDist(size_t agentNo, float neighborDist);

		/// <summary> Sets the two-dimensional position of a specified agent </summary>
		/// <param name="agentNo"> The number of the agent whose two - dimensional position is to be modified </param>
		/// <param name="position"> The replacement of the two-dimensional position </param>
		void setAgentPosition(size_t agentNo, const Vector2& position);

		/// <summary> Sets the two-dimensional preferred velocity of a specified agent </summary>
		/// <param name="agentNo"> The number of the agent whose two - dimensional preferred velocity is to be modified</param>
		/// <param name="prefVelocity"> The replacement of the two-dimensional preferred velocity </param>
		void setAgentPrefVelocity(size_t agentNo, const Vector2& prefVelocity);

		/// <summary> Sets the radius of a specified agent </summary>
		/// <param name="agentNo"> The number of the agent whose radius is to be modified </param>
		/// <param name="radius"> The replacement radius. Must be non - negative </param>
		void setAgentRadius(size_t agentNo, float radius);

		/// <summary> Sets the time horizon of a specified agent with respect to obstacles </summary>
		/// <param name="agentNo"> The number of the agent whose time horizon with respect to obstacles is to be modified </param>
		/// <param name="timeHorizonObst"> The replacement time horizon with respect to obstacles. Must be positive </param>
		void setAgentTimeHorizonObst(size_t agentNo, float timeHorizonObst);

		/// <summary> Sets the two-dimensional linear velocity of a specified agent </summary>
		/// <param name="agentNo"> The number of the agent whose two - dimensional linear velocity is to be modified </param>
		/// <param name="velocity"> The replacement two-dimensional linear velocity </param>
		void setAgentVelocity(size_t agentNo, const Vector2& velocity);

		/// <summary> Sets the time step of the simulation</summary>
		/// <param name="timeStep"> The time step of the simulation. Must be positive </param>
		void setTimeStep(float timeStep);

		/// <summary> Sets the additional force </summary>
		/// <param name="velocity"> New value of velocity </param>
		/// <param name="set"> Value of rotation set </param>
		void setAdditionalForce(const Vector3 &velocity, const RotationDegreeSet &set);

		/// <summary> Sets the attractive force </summary>
		/// <param name="attractiveStrength"> Attractive Strength coefficient </param>
		/// <param name="repulsiveStrength"> Repulsive Strength coefficient </param>
		/// <param name="attractiveRange"> Attractive Range coefficient </param>
		/// <param name="repulsiveRange"> Repulsive Range coefficient </param>
		void setAttractiveForce(
			float attractiveStrength, 
			float repulsiveStrength, 
			float attractiveRange, 
			float repulsiveRange
		);

		/// <summary> Sets the list of attractive agents to specified agent</summary>
		/// <param name="id"> The number of the agent </param>
		/// <param name="attractiveIds"> The list of attractive agent ID</param>
		void setAttractiveIdList(int id, const std::vector<int> &attractiveIds);

		/// <summary> Sets the attractive agent to specified agent</summary>
		/// <param name="id"> The number of the agent </param>
		/// <param name="newID"> The attractive agent ID </param>
		void addAttractiveId(int id, int newId);
		
		/// <summary> Adds the list of attractive agents to specified agent </summary>
		/// <param name="id"> The number of the agent </param>
		/// <param name="attractiveIds"> The list of attractive agent ID</param>
		void addAttractiveIdList(int id, const std::vector<int> &attractiveIds);

		/// <summary> The deleting of special attractive agent </summary>
		/// <param name="id"> The number of the agent </param>
		/// <param name="idFoeDelete"> The attractive agent ID </param>
		void deleteAttractiveId(int id, int idForDelete);

		/// <summary> The deleting of the list of attractive agents </summary>
		/// <param name="id"> The number of the agent </param>
		/// <param name="attractiveIds"> The list of attractive agent ID</param>
		void deleteAttractiveIdList(int id, const std::vector<int> &attractiveIds);

		/// <summary> Sets the velocity of platform </summary>
		/// <param name="velocity"> New value of velocit </param>
		void setPlatformVelocity(const Vector3 &velocity);

		/// <summary> Returns the velocity of platform </summary>
		/// <returns> The platform velocity </returns>
		Vector3 getPlatformVelocity() const;

		/// <summary> Returns the agent friction of platform </summary>
		/// <param name="agentNo"> The number of the agent whose friction is to be retrieved </param>
		/// <returns> The friction of agent </returns>
		float getAgentFriction(size_t agentNo) const;

		/// <summary> Sets the friction of platform </summary>
		/// <param name="agentNo"> The number of the agent </param>
		/// <param name="friction"> New value of friction </param>
		void setAgentFriction(size_t agentNo, float friction);

		/// <summary> Sets the angle set </summary>
		/// <param name="set"> Angle set </param>
		void setRotationDegreeSet(const RotationDegreeSet &set);

		/// <summary> Returns the angle set </summary>
		/// <returns> The angle set </returns>
		RotationDegreeSet getRotationDegreeSet() const;

		/// <summary> Adds the platform rotation on XY axis </summary>
		/// <param name="value"> The new rotation value </param>
		void addPlatformRotationXY(float value);

		/// <summary> Adds the platform rotation on XZ axis </summary>
		/// <param name="value"> The new rotation value </param>
		void addPlatformRotationXZ(float value);

		/// <summary> Adds the platform rotation on YZ axis </summary>
		/// <param name="value"> The new rotation value </param>
		void addPlatformRotationYZ(float value);

		/// <summary> Returns the platform rotation on XY axis </summary>
		/// <returns> The platform rotation on XY axis </returns>
		double getPlatformRotationXY() const;

		/// <summary> Returns the platform rotation on XY axis </summary>
		/// <returns> The platform rotation on XZ axis </returns>
		double getPlatformRotationXZ() const;

		/// <summary> Returns the platform rotation on YZ axis </summary>
		/// <returns> The platform rotation on XY axis </returns>
		double getPlatformRotationYZ() const;

		/// <summary> Returns the agent pressure</summary>
		/// <param name="index"> The number of the agent </param>
		/// <returns> The agent pressure </returns>
		double getAgentPressure(size_t index);

		/// <summary> Returns the obstacle pressure </summary>
		/// <param name="index"> The number of the agent </param>
		/// <returns> The obstacle pressure </returns>
		double getObstaclePressure(size_t index);
		
		/// <summary> Returns the obstacle trajectory </summary>
		/// <param name="index"> The number of the agent </param>
		/// <returns> The the obstacle trajectory vector </returns>
		Vector2 getObstacleTrajectory(size_t index);

		/// <summary> Returns a list of indices into a specified radius agents </summary>
		/// <param name="index"> The number of the agent </param>
		/// <param name="radius"> The specified radius </param>
		/// <returns> A list of indices into a specified radius agents </returns>
		std::vector<size_t> getAgentNeighboursIndexList(size_t index, float radius);

		/// <summary> Deleting the specified agent </summary>
		/// <param name="index"> The number of the agent </param>
		void deleteAgent(size_t index);

		/// <summary> Returns the list containing counts of alive and dead agents respectively </summary>
		/// <returns> The list containing counts of alive and dead agents </returns>
		std::vector<size_t> getCountOfAliveAndDead();

		/// <summary> Returns the list containing IDs of deleted agents </summary>
		/// <returns> The list containing IDs of deleted agents </returns>
		std::vector<size_t> getDeletedIDList();

		/// <summary> Sets the new SF parameters </summary>
		/// <param name="newRepulsiveAgent_"> New RepulsiveAgent value </param>
		/// <param name="newRepulsiveAgentFactor_"> New RepulsiveAgentFactor value </param>
		/// <param name="newRepulsiveObstacle_"> New RepulsiveObstacle value </param>
		/// <param name="newRepulsiveObstacleFactor_"> New RepulsiveObstacleFactor value </param>
		void updateSFParameters(
			float newRepulsiveAgent_,
			float newRepulsiveAgentFactor_,
			float newRepulsiveObstacle_,
			float newRepulsiveObstacleFactor_
		);

		/// <summary> Set of rotation values in different simple time inretval </summary>
		Vector3 rotationPast_;				
		Vector3 rotationPast2Now_;
		Vector3 rotationNow_;
		Vector3 rotationNow2Future_;
		Vector3 rotationFuture_;

		/// <summary> Attractive Force section </summary>
		float attractiveStrength_;
		float attractiveRange_;
		float repulsiveStrength_;
		float repulsiveRange_;
	
		bool IsMovingPlatform;				// mark if platform if moving

		std::vector<size_t> deleteIDs;		// list of deleted agents

	private:
		std::vector<Agent*> agents_;		// all agents list
		Agent* defaultAgent_;				// default setting
		float globalTime_;					// the global timer
		KdTree* kdTree_;					// the global tree 
		std::vector<Obstacle*> obstacles_;	// all obstacles list
		float timeStep_;					// time step
		Vector3 platformVelocity_;			// the velocity of platform
		RotationDegreeSet angleSet_;		// the rotation set
		double platformRotationXY_;			// the rotaion component of XY axis
		double platformRotationXZ_;			// the rotaion component of XZ axis
		double platformRotationYZ_;			// the rotaion component of YZ axis

		friend class Agent;
		friend class KdTree;
		friend class Obstacle;
	};
}

#endif
