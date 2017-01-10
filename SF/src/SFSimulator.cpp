#include "../include/SFSimulator.h"

#include "../include/Agent.h"
#include "../include/KdTree.h"
#include "../include/Obstacle.h"
#include "../include/AgentPropertyConfig.h"
#include "../include/RotationDegreeSet.h"
#include <iostream>
#include <algorithm>


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#if HAVE_OPENMP || _OPENMP
#include <omp.h>
#endif

namespace SF
{
	/// <summary> Constructs a simulator instance </summary>
	SFSimulator::SFSimulator() :
		rotationPast_(),
		rotationPast2Now_(),
		rotationNow_(),
		rotationNow2Future_(),
		rotationFuture_(),
		agents_(),
		defaultAgent_(NULL),
		globalTime_(0.0f),
		kdTree_(NULL),
		obstacles_(),
		timeStep_(1.0f),
		platformVelocity_(),
		platformRotationXY_(0),
		platformRotationXZ_(0),
		platformRotationYZ_(0),
		IsMovingPlatform(false)
	{
		kdTree_ = new KdTree(this);
	}

	/// <summary> Destroys this simulator instance </summary>
	SFSimulator::~SFSimulator()
	{
		delete defaultAgent_;

//#pragma omp parallel
//		{ 

//#pragma omp for
			for (int i = 0; i < agents_.size(); ++i)
				delete agents_[i];

//#pragma omp for
			for(int i = 0; i < tmpAgents_.size(); ++i)
				delete tmpAgents_[i];

//#pragma omp for
			for (int i = 0; i < obstacles_.size(); ++i)
				delete obstacles_[i];

			delete kdTree_;
		//}
	}

	Agent* SFSimulator::getAgent(size_t agentId)
	{
		for(size_t i = 0; i < agents_.size(); i++)
		{
			if(agents_[i]->id_ == agentId)
			{
				return agents_[i];
			}
		}
		return NULL;
	}

	/// <summary> Get list of agents who are not in modeling areas </summary>
	/// <param name="leftBotPoint"> Pointer to external agent </param>
	/// <param name="rightTopPoint"> Pointer to external agent </param>
	/// <returns> Vector of agent who are not in modeling area</returns>
	std::vector<Agent* > SFSimulator::getAgentsWhoNotInArea(Vector2 leftBotPoint, Vector2 rightTopPoint)
	{
		std::vector<Agent* > agentsOutsideOfArea;
		for(int i = 0; i < agents_.size(); i++)
		{
			if(agents_[i]->position_.x() < leftBotPoint.x() || agents_[i]->position_.x() > leftBotPoint.x() //If outside of modeling area
			|| agents_[i]->position_.y() < leftBotPoint.y() || agents_[i]->position_.y() > leftBotPoint.y()) 
			{
				agentsOutsideOfArea.push_back(agents_[i]);
			}
		}

		return agentsOutsideOfArea; 
	}

	/// <summary> Returns the count of agent neighbors taken into account to compute the current velocity for the specified agent </summary>
	/// <param name="agentNo"> The number of the agent whose count of agent neighbors is to be retrieved </param>
	/// <returns> The count of agent neighbors taken into account to compute the current velocity for the specified agent </returns>
	size_t SFSimulator::getAgentNumAgentNeighbors(size_t agentNo) const
	{
		return agents_[agentNo]->agentNeighbors_.size();
	}

	/// <summary> Returns the specified agent neighbor of the specified agent </summary>
	/// <param name="agentNo"> The number of the agent whose agent neighbor is to be retrieved </param>
	/// <param name="neighborNo"> The number of the agent neighbor to be retrieved </param>
	/// <returns> The number of the neighboring agent </returns>
	size_t SFSimulator::getAgentAgentNeighbor(size_t agentNo, size_t neighborNo) const
	{
		return agents_[agentNo]->agentNeighbors_[neighborNo].second->id_;
	}

	/// <summary> Returns the specified obstacle neighbor of the specified agent </summary>
	/// <param name="agentNo"> The number of the agent whose obstacle neighbor is to be retrieved </param>
	/// <param name="neighborNo"> The number of the obstacle neighbor to be retrieved </param>
	/// <returns> The number of the first vertex of the neighboring obstacle edge </returns>
	size_t SFSimulator::getAgentObstacleNeighbor(size_t agentNo, size_t neighborNo) const
	{
		return agents_[agentNo]->obstacleNeighbors_[neighborNo].second->id_;
	}

	/// <summary> Returns the count of obstacle neighbors taken into account to compute the current velocity for the specified agent </summary>
	/// <param name="agentNo"> The number of the agent whose count of obstacle neighbors is to be retrieved </param>
	/// <returns> The count of obstacle neighbors taken into account to compute the current velocity for the specified agent </returns>
	size_t SFSimulator::getAgentNumObstacleNeighbors(size_t agentNo) const
	{
		return agents_[agentNo]->obstacleNeighbors_.size();
	}

	/// <summary> Adds a new agent with default properties to the simulation </summary>
	/// <param name="position"> The two-dimensional starting position of this agent </param>
	/// <returns> The number of the agent, or SF::SF_ERROR when the agent defaults have not been set </returns>
	size_t SFSimulator::addAgent(const Vector2& position)
	{
		if (defaultAgent_ == 0)
			return SF_ERROR;

		Agent* agent = new Agent(this);

		agent->position_ = position;
		agent->maxNeighbors_ = defaultAgent_->maxNeighbors_;
		agent->maxSpeed_ = defaultAgent_->maxSpeed_;
		agent->force_ = defaultAgent_->force_;
		agent->neighborDist_ = defaultAgent_->neighborDist_;
		agent->radius_ = defaultAgent_->radius_;
		agent->timeHorizonObst_ = defaultAgent_->timeHorizonObst_;
		agent->velocity_ = defaultAgent_->velocity_;
		agent->accelerationCoefficient_ = defaultAgent_->accelerationCoefficient_;
		agent->relaxationTime_ = defaultAgent_->relaxationTime_;
		agent->repulsiveAgent_ = defaultAgent_->repulsiveAgent_;
		agent->repulsiveAgentFactor_ = defaultAgent_->repulsiveAgentFactor_;
		agent->repulsiveObstacle_ = defaultAgent_->repulsiveObstacle_;
		agent->repulsiveObstacleFactor_ = defaultAgent_->repulsiveObstacleFactor_;
		agent->obstacleRadius_ = defaultAgent_->obstacleRadius_;
		agent->platformFactor_ = defaultAgent_->platformFactor_;
		agent->perception_ = defaultAgent_->perception_;
		agent->friction_ = defaultAgent_->friction_;

		agent->id_ = agents_.size();

		agents_.push_back(agent);

		return agents_.size() - 1;
	}

	/// <summary> Print agents info </summary>
	void SFSimulator::printAgentsInfo() const
	{
		std::cout << "Agents count: " << agents_.size() << std::endl;
		for(int i = 0; i < agents_.size(); i++)
		{
			std::cout << "ID: " << agents_[i]->id_ << " X: " << agents_[i]->position_.x() << " Y: " << agents_[i]->position_.y() << " VelX: " << agents_[i]->velocity_.x() << " VelY: " << agents_[i]->velocity_.y() <<std::endl;
		}
	}

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
	size_t SFSimulator::addAgent(
		const Vector2& position,
		float neighborDist,
		size_t maxNeighbors,
		float timeHorizonObst,
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
		const Vector2& velocity
		)
	{
		Agent* agent = new Agent(this);

		agent->position_ = position;
		agent->maxNeighbors_ = maxNeighbors;
		agent->maxSpeed_ = maxSpeed;
		agent->force_ = force;
		agent->neighborDist_ = neighborDist;
		agent->radius_ = radius;
		agent->timeHorizonObst_ = timeHorizonObst;
		agent->velocity_ = velocity;
		agent->accelerationCoefficient_ = accelerationCoefficient;
		agent->relaxationTime_ = relaxationTime;
		agent->repulsiveAgent_ = repulsiveAgent;
		agent->repulsiveAgentFactor_ = repulsiveAgentFactor;
		agent->repulsiveObstacle_ = repulsiveObstacle;
		agent->repulsiveObstacleFactor_ = repulsiveObstacleFactor;
		agent->obstacleRadius_ = obstacleRadius;
		agent->platformFactor_ = platformFactor;
		agent->perception_ = perception;
		agent->friction_ = friction;

		agent->id_ = agents_.size();

		agents_.push_back(agent);

		return agents_.size() - 1;
	}

	/// <summary> Adds an external agent to the simulation </summary>
	/// <param name="newAgent"> Pointer to external agent </param>
	/// <returns> The number of the agent</returns>
	size_t SFSimulator::addAgent(Agent* newAgent)
	{
		newAgent->id_ = agents_.size();
		agents_.push_back(newAgent);
		newAgent->sim_ = this;
		return newAgent->id_;
	}

	/// <summary> Adds an external agent to the simulation that will be deleted after a step</summary>
	/// <param name="newAgent"> Pointer to external agent </param>
	/// <returns> The number of the agent</returns>
	size_t SFSimulator::addTempAgent( Agent* newAgent )
	{
		newAgent->id_ = tmpAgents_.size();
		tmpAgents_.push_back(newAgent);
		newAgent->sim_ = this;
		return newAgent->id_;
	}

	/// <summary> Adds a new obstacle to the simulation </summary>
	/// <param name="vertices"> List of the vertices of the polygonal obstacle in counterclockwise order </param>
	/// <returns> The number of the first vertex of the obstacle, or SF::SF_ERROR when the number of vertices is less than two</returns>
	size_t SFSimulator::addObstacle(const std::vector<Vector2>& vertices)
	{
		if (vertices.size() < 2)
			return SF_ERROR;

		size_t obstacleNo = obstacles_.size();

		for (size_t i = 0; i < vertices.size(); ++i) {
			Obstacle* obstacle = new Obstacle();
			obstacle->point_ = vertices[i];

			if (i != 0)
			{
				obstacle->prevObstacle = obstacles_.back();
				obstacle->prevObstacle->nextObstacle = obstacle;
			}
			if (i == vertices.size() - 1)
			{
				obstacle->nextObstacle = obstacles_[obstacleNo];
				obstacle->nextObstacle->prevObstacle = obstacle;
			}

			obstacle->unitDir_ = normalize(vertices[(i == vertices.size() - 1 ? 0 : i + 1)] - vertices[i]);

			if (vertices.size() == 2)
				obstacle->isConvex_ = true;
			else
				obstacle->isConvex_ = (leftOf(vertices[(i == 0 ? vertices.size() - 1 : i - 1)], vertices[i], vertices[(i == vertices.size() - 1 ? 0 : i + 1)]) >= 0);


			obstacle->id_ = obstacles_.size();

			obstacles_.push_back(obstacle);
		}

		return obstacleNo;
	}

	bool SFSimulator::deleteObstacle(size_t objectId)
	{
		bool isFound = false;

		for (int i = 0; i < obstacles_.size(); i++)
		{
			if (obstacles_[i]->id_ == objectId)
			{
				int pointsInObstacle = 1;
				Obstacle* currentObstacle = obstacles_[i]->nextObstacle;
				while (currentObstacle != obstacles_[i])
				{
					pointsInObstacle++;
					currentObstacle = currentObstacle->nextObstacle;
				}

				isFound = true;
				obstacles_.erase(obstacles_.begin() + i, obstacles_.begin() + i + pointsInObstacle);
				break;
			}
		}

		return isFound;
	}

	/// <summary> Lets the simulator perform a simulation step and updates the two - dimensional position and two - dimensional velocity of each agent </summary>
	void SFSimulator::doStep()
	{
		kdTree_->buildAgentTree();

		if (agents_.size() > 0)
		{
			addPlatformRotationXZ(getRotationDegreeSet().getRotationOY());
			addPlatformRotationYZ(getRotationDegreeSet().getRotationOX());
		}

		//#pragma omp parallel
		//{ 
			//#pragma omp for
			for (int i = 0; i < static_cast<size_t>(agents_.size()); ++i)
			{
				if (!(agents_[i]->isDeleted_))
				{
					agents_[i]->computeNeighbors();
					agents_[i]->computeNewVelocity();
				}
			}

			//#pragma omp for
			for(int i = 0; i < static_cast<size_t>(agents_.size()); ++i)
			{
				if(!(agents_[i]->isDeleted_))
					agents_[i]->update();
			}
		//}

		for(size_t i = 0; i < tmpAgents_.size(); i++)
		{
			delete tmpAgents_[i];
		}

		tmpAgents_.clear();

		globalTime_ += timeStep_;
	}

	/// <summary> Returns the maximum neighbor count of a specified agent </summary>
	/// <param name="agentNo"> The number of the agent whose maximum neighbor count is to be retrieved </param>
	/// <returns> The present maximum neighbor count of the agent </returns>
	size_t SFSimulator::getAgentMaxNeighbors(size_t agentNo) const
	{
		return agents_[agentNo]->maxNeighbors_;
	}

	/// <summary> Returns the maximum speed of a specified agent </summary>
	/// <param name="agentNo"> The number of the agent whose maximum speed is to be retrieved </param>
	/// <returns> The present maximum speed of the agent </returns>
	float SFSimulator::getAgentMaxSpeed(size_t agentNo) const
	{
		return agents_[agentNo]->maxSpeed_;
	}

	/// <summary> Returns the maximum neighbor distance of a specified agent </summary>
	/// <param name="agentNo"> The number of the agent whose maximum neighbor distance is to be retrieved </param>
	/// <returns> The present maximum neighbor distance of the agent</returns>
	float SFSimulator::getAgentNeighborDist(size_t agentNo) const
	{
		return agents_[agentNo]->neighborDist_;
	}

	/// <summary> Returns the two-dimensional position of a specified agent </summary>
	/// <param name="agentNo"> The number of the agent whose two - dimensional position is to be retrieved </param>
	/// <returns> The present two-dimensional position of the (center of the) agent </returns>
	const Vector2& SFSimulator::getAgentPosition(size_t agentNo) const
	{
		return agents_[agentNo]->position_;
	}

	/// <summary> Returns the two-dimensional preferred velocity  of a specified agent </summary>
	/// <param name="agentNo"> The number of the agent whose two-dimensional preferred velocity is to be retrieved </param>
	/// <returns> The present two-dimensional of the agent </returns>
	const Vector2& SFSimulator::getAgentPrefVelocity(size_t agentNo) const
	{
		return agents_[agentNo]->prefVelocity_;
	}

	/// <summary> Returns the radius of a specified agent </summary>
	/// <param name="agentNo"> The number of the agent whose radius is to be retrieved </param>
	/// <returns> The present radius of the agent </returns>
	float SFSimulator::getAgentRadius(size_t agentNo) const
	{
		return agents_[agentNo]->radius_;
	}

	/// <summary> Returns the time horizon with respect to obstacles of a specified agent </summary>
	/// <param name="agentNo"> The number of the agent whose time horizon with respect to obstacles is to be retrieved </param>
	/// <returns> The present time horizon with respect to obstacles of the agent </returns>
	float SFSimulator::getAgentTimeHorizonObst(size_t agentNo) const
	{
		return agents_[agentNo]->timeHorizonObst_;
	}

	/// <summary> Returns the two-dimensional linear velocity of a specified agent </summary>
	/// <param name="agentNo"> The number of the agent whose two - dimensional linear velocity is to be retrieved </param>
	/// <returns> The present two-dimensional linear velocity of the agent </returns>
	const Vector2& SFSimulator::getAgentVelocity(size_t agentNo) const
	{
		return agents_[agentNo]->velocity_;
	}

	/// <summary> Returns the global time of the simulation </summary>
	/// <returns> The present global time of the simulation (zero initially) </returns>
	float SFSimulator::getGlobalTime() const
	{
		return globalTime_;
	}

	/// <summary> Returns the count of agents in the simulation </summary>
	/// <returns> The count of agents in the simulation </returns>
	size_t SFSimulator::getNumAgents() const
	{
		return agents_.size();
	}

	/// <summary> Returns the count of obstacle vertices in the simulation </summary>
	/// <returns> The count of obstacle vertices in the simulation </returns>
	size_t SFSimulator::getNumObstacleVertices() const
	{
		return obstacles_.size();
	}

	/// <summary> Returns the two-dimensional position of a specified obstacle vertex </summary>
	/// <param name="vertexNo"> The number of the obstacle vertex to be retrieved </param>
	/// <returns> The two-dimensional position of the specified obstacle vertex </returns>
	const Vector2& SFSimulator::getObstacleVertex(size_t vertexNo) const
	{
		return obstacles_[vertexNo]->point_;
	}

	/// <summary> Returns the number of the obstacle vertex succeeding the specified obstacle vertex in its polygon </summary>
	/// <param name="vertexNo"> The number of the obstacle vertex whose successor is to be retrieved </param>
	/// <returns> The number of the obstacle vertex succeeding the specified obstacle vertex in its polygon</returns>
	size_t SFSimulator::getNextObstacleVertexNo(size_t vertexNo) const
	{
		return obstacles_[vertexNo]->nextObstacle->id_;
	}

	/// <summary> Returns the number of the obstacle vertex preceding the specified obstacle vertex in its polygon </summary>
	/// <param name="agentNo"> The number of the obstacle vertex whose predecessor is to be retrieved </param>
	/// <returns> The number of the obstacle vertex preceding the specified obstacle vertex in its polygon </returns>
	size_t SFSimulator::getPrevObstacleVertexNo(size_t vertexNo) const
	{
		return obstacles_[vertexNo]->prevObstacle->id_;
	}

	/// <summary> Returns the time step of the simulation </summary>
	/// <returns> The present time step of the simulation </returns>
	float SFSimulator::getTimeStep() const
	{
		return timeStep_;
	}

	/// <summary> Processes the obstacles that have been added so that they are accounted for in the simulation </summary>
	void SFSimulator::processObstacles() const
	{
		kdTree_->buildObstacleTree();
	}

	/// <summary> Performs a visibility query between the two specified points with respect to the obstacles </summary>
	/// <param name="point1"> The first point of the query </param>
	/// <param name="point2"> The second point of the query </param>
	/// <param name="radius"> The minimal distance between the line connecting the two points and the obstacles in order for the points to be mutually visible(optional). Must be non - negative </param>
	/// <returns> A boolean specifying whether the two points are mutually visible. Returns true when the obstacles have not been processed </returns>
	bool SFSimulator::queryVisibility(const Vector2& point1, const Vector2& point2, float radius) const
	{
		return kdTree_->queryVisibility(point1, point2, radius);
	}

	/// <summary> Sets default property of agent</summary>
	/// <param name="apc"> Property </param>
	void SFSimulator::setAgentDefaults(AgentPropertyConfig & apc)
	{
		if (defaultAgent_ == NULL)
			defaultAgent_ = new Agent(this);


		defaultAgent_->maxNeighbors_ = apc._maxNeighbors;
		defaultAgent_->maxSpeed_ = apc._maxSpeed;
		defaultAgent_->force_ = apc._force;
		defaultAgent_->neighborDist_ = apc._neighborDist;
		defaultAgent_->radius_ = apc._radius;
		defaultAgent_->timeHorizonObst_ = apc._timeHorizon;
		defaultAgent_->accelerationCoefficient_ = apc._accelerationCoefficient;
		defaultAgent_->relaxationTime_ = apc._relaxationTime;
		defaultAgent_->repulsiveAgent_ = apc._repulsiveAgent;
		defaultAgent_->repulsiveAgentFactor_ = apc._repulsiveAgentFactor;
		defaultAgent_->repulsiveObstacle_ = apc._repulsiveObstacle;
		defaultAgent_->repulsiveObstacleFactor_ = apc._repulsiveObstacleFactor;
		defaultAgent_->obstacleRadius_ = apc._obstacleRadius;
		defaultAgent_->platformFactor_ = apc._platformFactor;
		defaultAgent_->perception_ = apc._perception;
		defaultAgent_->friction_ = apc._friction;
		defaultAgent_->velocity_ = apc._velocity;
	}

	/// <summary> Sets the maximum neighbor count of a specified agent </summary>
	/// <param name="agentNo"> The number of the agent whose maximum neighbor count is to be modified </param>
	/// <param name="maxNeighbors"> The replacement maximum neighbor count </param>
	void SFSimulator::setAgentMaxNeighbors(size_t agentNo, size_t maxNeighbors)
	{
		agents_[agentNo]->maxNeighbors_ = maxNeighbors;
	}

	/// <summary> Sets the maximum speed of a specified agent </summary>
	/// <param name="agentNo"> The number of the agent whose maximum speed is to be modified </param>
	/// <param name="maxSpeed"> The replacement maximum speed. Must be non - negative </param>
	void SFSimulator::setAgentMaxSpeed(size_t agentNo, float maxSpeed)
	{
		agents_[agentNo]->maxSpeed_ = maxSpeed;
	}


	void SFSimulator::setAgentForce(size_t agentNo, float force)
	{
		agents_[agentNo]->force_ = force;
	}

	/// <summary> Sets the maximum neighbor distance of a specified agent </summary>
	/// <param name="agentNo"> The number of the agent whose maximum neighbor distance is to be modified </param>
	/// <param name="neighborDist"> The replacement maximum neighbor distance. Must be non - negative</param>
	void SFSimulator::setAgentNeighborDist(size_t agentNo, float neighborDist)
	{
		agents_[agentNo]->neighborDist_ = neighborDist;
	}

	/// <summary> Sets the two-dimensional position of a specified agent </summary>
	/// <param name="agentNo"> The number of the agent whose two - dimensional position is to be modified </param>
	/// <param name="position"> The replacement of the two-dimensional position </param>
	void SFSimulator::setAgentPosition(size_t agentNo, const Vector2& position)
	{
		agents_[agentNo]->position_ = position;
	}

	/// <summary> Sets the two-dimensional preferred velocity of a specified agent </summary>
	/// <param name="agentNo"> The number of the agent whose two - dimensional preferred velocity is to be modified</param>
	/// <param name="prefVelocity"> The replacement of the two-dimensional preferred velocity </param>
	void SFSimulator::setAgentPrefVelocity(size_t agentNo, const Vector2& prefVelocity)
	{
		agents_[agentNo]->prefVelocity_ = prefVelocity;
	}

	/// <summary> Sets the radius of a specified agent </summary>
	/// <param name="agentNo"> The number of the agent whose radius is to be modified </param>
	/// <param name="radius"> The replacement radius. Must be non - negative </param>
	void SFSimulator::setAgentRadius(size_t agentNo, float radius)
	{
		agents_[agentNo]->radius_ = radius;
	}

	/// <summary> Sets the time horizon of a specified agent with respect to obstacles </summary>
	/// <param name="agentNo"> The number of the agent whose time horizon with respect to obstacles is to be modified </param>
	/// <param name="timeHorizonObst"> The replacement time horizon with respect to obstacles. Must be positive </param>
	void SFSimulator::setAgentTimeHorizonObst(size_t agentNo, float timeHorizonObst)
	{
		agents_[agentNo]->timeHorizonObst_ = timeHorizonObst;
	}

	/// <summary> Sets the two-dimensional linear velocity of a specified agent </summary>
	/// <param name="agentNo"> The number of the agent whose two - dimensional linear velocity is to be modified </param>
	/// <param name="velocity"> The replacement two-dimensional linear velocity </param>
	void SFSimulator::setAgentVelocity(size_t agentNo, const Vector2& velocity)
	{
		agents_[agentNo]->velocity_ = velocity;
	}

	/// <summary> Sets the time step of the simulation</summary>
	/// <param name="timeStep"> The time step of the simulation. Must be positive </param>
	void SFSimulator::setTimeStep(float timeStep)
	{
		timeStep_ = timeStep;
	}

	/// <summary> Sets the velocity of platform </summary>
	/// <param name="velocity"> New value of velocit </param>
	void SFSimulator::setPlatformVelocity(const Vector3 &velocity)
	{
		platformVelocity_ = velocity;
	}

	/// <summary> Returns the velocity of platform </summary>
	/// <returns> The platform velocity </returns>
	Vector3 SFSimulator::getPlatformVelocity() const
	{
		return platformVelocity_;
	}

	/// <summary> Sets the friction of platform </summary>
	/// <param name="agentNo"> The number of the agent </param>
	/// <param name="friction"> New value of friction </param>
	void SFSimulator::setAgentFriction(size_t agentNo, float friction)
	{
		agents_[agentNo]->friction_ = friction;
	}

	/// <summary> Returns the agent friction of platform </summary>
	/// <param name="agentNo"> The number of the agent whose friction is to be retrieved </param>
	/// <returns> The friction of agent </returns>
	float SFSimulator::getAgentFriction(size_t agentNo) const
	{
		return agents_[agentNo]->friction_;
	}

	/// <summary> Returns the angle set </summary>
	/// <returns> The angle set </returns>
	RotationDegreeSet SFSimulator::getRotationDegreeSet() const
	{
		return angleSet_;
	}

	/// <summary> Sets the angle set </summary>
	/// <param name="set"> Angle set </param>
	void SFSimulator::setRotationDegreeSet(const RotationDegreeSet &set)
	{
		IsMovingPlatform = true;

		if ((fabs(set.getRotationOX()) > SF_EPSILON) || (fabs(set.getRotationOY()) > SF_EPSILON) || (fabs(set.getRotationOZ()) > SF_EPSILON))
		{
			if (rotationPast_ == Vector3())
			{
				rotationPast_ = Vector3(set.getRotationOY(), set.getRotationOX(), set.getRotationOZ());
			}
			else if (rotationNow_ == Vector3())
			{
				rotationNow_ = Vector3(set.getRotationOY(), set.getRotationOX(), set.getRotationOZ());
				rotationPast2Now_ = (rotationPast_ + rotationNow_) / 2;
			}
			else if (rotationFuture_ == Vector3())
			{
				rotationFuture_ = Vector3(set.getRotationOY(), set.getRotationOX(), set.getRotationOZ());
				rotationNow2Future_ = (rotationNow_ + rotationFuture_) / 2;
			}
			else
			{
				rotationPast_ = rotationNow_;
				rotationNow_ = rotationFuture_;
				rotationFuture_ = Vector3(set.getRotationOY(), set.getRotationOX(), set.getRotationOZ());
				rotationPast2Now_ = (rotationPast_ + rotationNow_) / 2;
				rotationNow2Future_ = (rotationNow_ + rotationFuture_) / 2;
			}
		}
	}

	/// <summary> Sets the additional force </summary>
	/// <param name="velocity"> New value of velocity </param>
	/// <param name="set"> Value of rotation set </param>
	void SFSimulator::setAdditionalForce(const Vector3 &velocity, const RotationDegreeSet &set)
	{
		setPlatformVelocity(velocity);
		setRotationDegreeSet(set);
	}

	/// <summary> Sets the attractive force </summary>
	/// <param name="attractiveStrength"> Attractive Strength coefficient </param>
	/// <param name="repulsiveStrength"> Repulsive Strength coefficient </param>
	/// <param name="attractiveRange"> Attractive Range coefficient </param>
	/// <param name="repulsiveRange"> Repulsive Range coefficient </param>
	void SFSimulator::setAttractiveForce(
		float attractiveStrength, 
		float repulsiveStrength, 
		float attractiveRange, 
		float repulsiveRange
	)
	{
		attractiveStrength_ = attractiveStrength;
		repulsiveStrength_ = repulsiveStrength;
		attractiveRange_ = attractiveRange;
		repulsiveRange_ = repulsiveRange;
	}

	/// <summary> Sets the list of attractive agents to specified agent</summary>
	/// <param name="id"> The number of the agent </param>
	/// <param name="attractiveIds"> The list of attractive agent ID</param>
	void SFSimulator::setAttractiveIdList(int id, const std::vector<int>& attractiveIds)
	{
		agents_[id]->attractiveIds_ = attractiveIds;
	}

	/// <summary> Sets the attractive agent to specified agent</summary>
	/// <param name="id"> The number of the agent </param>
	/// <param name="newID"> The attractive agent ID </param>
	void SFSimulator::addAttractiveId(int id, int newId)
	{
		std::vector<int> ail = agents_[id]->attractiveIds_;
		if(std::find(ail.begin(), ail.end(), newId) == ail.end())
			agents_[id]->attractiveIds_.push_back(newId);
	}

	/// <summary> Adds the list of attractive agents to specified agent </summary>
	/// <param name="id"> The number of the agent </param>
	/// <param name="attractiveIds"> The list of attractive agent ID</param>
	void SFSimulator::addAttractiveIdList(int id, const std::vector<int>& attractiveIds)
	{
		//for (auto ai : attractiveIds)
		for( int i = 0; i < attractiveIds.size(); i++)
		{
			addAttractiveId(id, attractiveIds[i]);
		}

	}

	/// <summary> The deleting of special attractive agent </summary>
	/// <param name="id"> The number of the agent </param>
	/// <param name="idFoeDelete"> The attractive agent ID </param>
	void SFSimulator::deleteAttractiveId(int id, int idForDelete)
	{
		std::vector<int> aais = agents_[id]->attractiveIds_;
		for (std::vector<int>::iterator i = aais.begin(); i != aais.end(); ++i)
		{
			if (*i == idForDelete)
			{
				aais.erase(i);
				break;
			}
		}
	}

	/// <summary> The deleting of the list of attractive agents </summary>
	/// <param name="id"> The number of the agent </param>
	/// <param name="attractiveIds"> The list of attractive agent ID</param>
	void SFSimulator::deleteAttractiveIdList(int id, const std::vector<int>& attractiveIds)
	{
		//for(auto ai: attractiveIds)
		for(int i = 0; i < attractiveIds.size(); i++)
		{
			deleteAttractiveId(id, attractiveIds[i]);	
		}
	}

	/// <summary> Adds the platform rotation on XY axis </summary>
	/// <param name="value"> The new rotation value </param>
	void SFSimulator::addPlatformRotationXY(float value)
	{
		double futureSum = platformRotationXY_ + value;

		if (futureSum >= 2 * M_PI)
			platformRotationXY_ = futureSum - 2 * M_PI;
		else
			platformRotationXY_ = futureSum;
	}

	/// <summary> Adds the platform rotation on XZ axis </summary>
	/// <param name="value"> The new rotation value </param>
	void SFSimulator::addPlatformRotationXZ(float value)
	{
		double futureSum = platformRotationXZ_ + value;

		if (futureSum >= 2 * M_PI)
			platformRotationXZ_ = futureSum - 2 * M_PI;
		else
			platformRotationXZ_ = futureSum;
	}

	/// <summary> Adds the platform rotation on YZ axis </summary>
	/// <param name="value"> The new rotation value </param>
	void SFSimulator::addPlatformRotationYZ(float value)
	{
		double futureSum = platformRotationYZ_ + value;

		if (futureSum >= 2 * M_PI)
			platformRotationYZ_ = futureSum - 2 * M_PI;
		else
			platformRotationYZ_ = futureSum;
	}

	/// <summary> Returns the platform rotation on XY axis </summary>
	/// <returns> The platform rotation on XY axis </returns>
	double SFSimulator::getPlatformRotationXY() const
	{
		return platformRotationXY_;
	}

	/// <summary> Returns the platform rotation on XY axis </summary>
	/// <returns> The platform rotation on XZ axis </returns>
	double SFSimulator::getPlatformRotationXZ() const
	{
		return platformRotationXZ_;
	}

	/// <summary> Returns the platform rotation on YZ axis </summary>
	/// <returns> The platform rotation on XY axis </returns>
	double SFSimulator::getPlatformRotationYZ() const
	{
		return platformRotationYZ_;
	}

	/// <summary> Returns a list of indices into a specified radius agents </summary>
	/// <param name="index"> The number of the agent </param>
	/// <param name="radius"> The specified radius </param>
	/// <returns> A list of indices into a specified radius agents </returns>
	std::vector<size_t> SFSimulator::getAgentNeighboursIndexList(size_t index, float radius)
	{
		std::vector<size_t> result;
		if (agents_.size() > 0)
		{
			if (index >= agents_.size())
				result.push_back(0);
			else
			{
				Agent* agent = agents_[index];
				float rangeSq = sqr(radius);

				agent->agentNeighborsIndexList_.clear();
				this->kdTree_->computeAgentNeighborsIndexList(agent, rangeSq);

				//for (auto an : agent->agentNeighborsIndexList_)
				for(int i = 0; i < agent->agentNeighborsIndexList_.size(); i++)
				{
					result.push_back(agent->agentNeighborsIndexList_[i].first);	
				}
			}
		}
		else
			result.push_back(0);

		return result;
	}

	/// <summary> Deleting the specified agent </summary>
	/// <param name="index"> The number of the agent </param>
	void SFSimulator::deleteAgent(size_t index)
	{
		agents_[index]->isDeleted_ = true;
	}

	/// <summary> Returns the list containing IDs of deleted agents </summary>
	/// <returns> The list containing IDs of deleted agents </returns>
	std::vector<size_t> SFSimulator::getDeletedIDList()
	{
		std::vector<size_t> result = std::vector<size_t>();

		//for (auto a : agents_)
		for(int i = 0; i < agents_.size(); i++)
		{
			if (agents_[i]->isDeleted_)
				result.push_back(agents_[i]->id_);
		}


		return result;
	}

	/// <summary> Returns the list containing counts of alive and dead agents respectively </summary>
	/// <returns> The list containing counts of alive and dead agents </returns>
	std::vector<size_t> SFSimulator::getCountOfAliveAndDead()
	{
		std::vector<size_t> result = std::vector<size_t>();

		size_t
			alive = 0,
			dead = 0;

		//for (auto a : agents_)
		for(int i = 0; i < agents_.size(); i++)
		{
			agents_[i]->isDeleted_ ? dead++ : alive++;
		}
		//a->isDeleted_ ? dead++ : alive++;
					
		result.push_back(agents_.size());
		result.push_back(alive);
		result.push_back(dead);

		return result;
	}

	/// <summary> Returns the list containing IDs of agents </summary>
	/// <returns> The list containing IDs of agents </returns>
	std::vector<size_t> SFSimulator::getAliveAgentIdsList()
	{
		std::vector<size_t> agentIdsList = std::vector<size_t>();
		for(int i = 0; i < agents_.size(); i++)
		{
			if(!agents_[i]->isDeleted_)
			{
				agentIdsList.push_back(agents_[i]->id_);	
			}
		}

		return agentIdsList;
	}

	/// <summary> Returns the list containing alive agents </summary>
	/// <returns> The list containing pointers of agents </returns>
	std::vector<Agent*> SFSimulator::getAliveAgents()
	{
		std::vector<Agent*> agentIdsList = std::vector<Agent*>();
		for(int i = 0; i < agents_.size(); i++)
		{
			if(!agents_[i]->isDeleted_)
			{
				agentIdsList.push_back(agents_[i]);	
			}
		}

		return agentIdsList;
	}

	/// <summary> Sets the new SF parameters </summary>
	/// <param name="newRepulsiveAgent_"> New RepulsiveAgent value </param>
	/// <param name="newRepulsiveAgentFactor_"> New RepulsiveAgentFactor value </param>
	/// <param name="newRepulsiveObstacle_"> New RepulsiveObstacle value </param>
	/// <param name="newRepulsiveObstacleFactor_"> New RepulsiveObstacleFactor value </param>
	void SFSimulator::updateSFParameters(float newRepulsiveAgent_, float newRepulsiveAgentFactor_, float newRepulsiveObstacle_, float newRepulsiveObstacleFactor_)
	{
#pragma omp parallel for

		for (int i = 0; i < static_cast<size_t>(agents_.size()); ++i)
		{
			if (!(agents_[i]->isDeleted_))
			{
				agents_[i]->repulsiveAgent_ = newRepulsiveAgent_;
				agents_[i]->repulsiveAgentFactor_ = newRepulsiveAgentFactor_;
				agents_[i]->repulsiveObstacle_ = newRepulsiveObstacle_;
				agents_[i]->repulsiveObstacleFactor_ = newRepulsiveObstacleFactor_;
			}
		}

		defaultAgent_->repulsiveAgent_ = newRepulsiveAgent_;
		defaultAgent_->repulsiveAgentFactor_ = newRepulsiveAgentFactor_;
		defaultAgent_->repulsiveObstacle_ = newRepulsiveObstacle_;
		defaultAgent_->repulsiveObstacleFactor_ = newRepulsiveObstacleFactor_;
	}

	/// <summary> Returns the agent pressure</summary>
	/// <param name="index"> The number of the agent </param>
	/// <returns> The agent pressure </returns>
	double SFSimulator::getAgentPressure(size_t index)
	{
		return agents_[index]->agentPressure_;
	}

	/// <summary> Returns the obstacle pressure </summary>
	/// <param name="index"> The number of the agent </param>
	/// <returns> The obstacle pressure </returns>
	double SFSimulator::getObstaclePressure(size_t index)
	{
		return agents_[index]->obstaclePressure_;
	}

	/// <summary> Returns the obstacle trajectory </summary>
	/// <param name="index"> The number of the agent </param>
	/// <returns> The the obstacle trajectory vector </returns>
	Vector2 SFSimulator::getObstacleTrajectory(size_t index)
	{
		return agents_[index]->obstacleForce_;
	}

	/// <summary> Returns the agent repulsive force </summary>
	/// <param name="index"> The number of the agent </param>
	/// <returns> The the agent repulsive force vector </returns>
	Vector2 SFSimulator::getAgentRepulsiveForce(size_t index)
	{
		return agents_[index]->agentForce_;
	}
}
