/*
 *  SFSimulator.cpp
 *  SF Library.
 */

#include "../include/SFSimulator.h"

#include "../include/Agent.h"
#include "../include/KdTree.h"
#include "../include/Obstacle.h"
#include "../include/AgentPropertyConfig.h"
#include "../include/RotationDegreeSet.h"


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#if HAVE_OPENMP || _OPENMP
#include <omp.h>
#endif

namespace SF
{

  SFSimulator::SFSimulator() : 
	agents_(), 
	defaultAgent_(0), 
	globalTime_(0.0f), 
	kdTree_(0), 
	obstacles_(), 
	timeStep_(1.0f),
	platformRotationXY_(0),
	platformRotationXZ_(0),
	platformRotationYZ_(0),
	platformVelocity_()
	{
		kdTree_ = new KdTree(this);
	}

  SFSimulator::~SFSimulator()
  {
    if (defaultAgent_ != 0) {
      delete defaultAgent_;
    }

    for (size_t i = 0; i < agents_.size(); ++i) {
      delete agents_[i];
    }

    for (size_t i = 0; i < obstacles_.size(); ++i) {
      delete obstacles_[i];
    }

    delete kdTree_;
  }

  size_t SFSimulator::getAgentNumAgentNeighbors(size_t agentNo) const
  {
    return agents_[agentNo]->agentNeighbors_.size();
  }

  size_t SFSimulator::getAgentAgentNeighbor(size_t agentNo, size_t neighborNo) const
  {
    return agents_[agentNo]->agentNeighbors_[neighborNo].second->id_;
  }

  size_t SFSimulator::getAgentObstacleNeighbor(size_t agentNo, size_t neighborNo) const
  {
    return agents_[agentNo]->obstacleNeighbors_[neighborNo].second->id_;
  }

  size_t SFSimulator::getAgentNumObstacleNeighbors(size_t agentNo) const
  {
    return agents_[agentNo]->obstacleNeighbors_.size();
  }

  size_t SFSimulator::addAgent(const Vector2& position)
  {
    if (defaultAgent_ == 0) {
      return SF_ERROR;
    }

    Agent* agent = new Agent(this);

    agent->position_ = position;
    agent->maxNeighbors_ = defaultAgent_->maxNeighbors_;
    agent->maxSpeed_ = defaultAgent_->maxSpeed_;
    agent->neighborDist_ = defaultAgent_->neighborDist_;
    agent->radius_ = defaultAgent_->radius_;
    agent->timeHorizonObst_ = defaultAgent_->timeHorizonObst_;
    agent->velocity_ = defaultAgent_->velocity_;
	agent->accelerationCoefficient_ = defaultAgent_->accelerationCoefficient_;
	agent->repulsiveAgent_ = defaultAgent_->repulsiveAgent_;
	agent->repulsiveAgentFactor_ = defaultAgent_->repulsiveAgentFactor_;
	agent->repulsiveObstacle_ = defaultAgent_->repulsiveObstacle_;
	agent->repulsiveObstacleFactor = defaultAgent_->repulsiveObstacleFactor;
	agent->perception_ = defaultAgent_->perception_;
	agent->friction_ = defaultAgent_->friction_;
	
    agent->id_ = agents_.size();

    agents_.push_back(agent);

    return agents_.size() - 1;
  }

  size_t SFSimulator::addAgent(
	  const Vector2& position, 
	  float neighborDist, 
	  size_t maxNeighbors, 
	  float timeHorizon, 
	  float timeHorizonObst, 
	  float radius, 
	  float maxSpeed, 
	  float accelerationCoefficient, 
	  float repulsiveAgent, 
	  float repulsiveAgentFactor, 
	  float repulsiveObstacle, 
	  float repulsiveObstacleFactor,
	  float perception, 
	  float friction,
	  const Vector2& velocity
  )
  {
    Agent* agent = new Agent(this);
    
    agent->position_ = position;
    agent->maxNeighbors_ = maxNeighbors;
    agent->maxSpeed_ = maxSpeed;
    agent->neighborDist_ = neighborDist;
    agent->radius_ = radius;
    agent->timeHorizonObst_ = timeHorizonObst;
    agent->velocity_ = velocity;
	agent->accelerationCoefficient_ = accelerationCoefficient;
	agent->repulsiveAgent_ = repulsiveAgent;
	agent->repulsiveAgentFactor_ = repulsiveAgentFactor;
	agent->repulsiveObstacle_ = repulsiveObstacle;
	agent->repulsiveObstacleFactor = repulsiveObstacleFactor;
	agent->perception_ = perception;
	agent->friction_ = friction;

    agent->id_ = agents_.size();
    
    agents_.push_back(agent);

    return agents_.size() - 1;
  }

  size_t SFSimulator::addObstacle(const std::vector<Vector2>& vertices)
  {
    if (vertices.size() < 2) {
      return SF_ERROR;
    }

    size_t obstacleNo = obstacles_.size();

    for (size_t i = 0; i < vertices.size(); ++i) {
      Obstacle* obstacle = new Obstacle();
      obstacle->point_ = vertices[i];
      if (i != 0) {
        obstacle->prevObstacle = obstacles_.back();
        obstacle->prevObstacle->nextObstacle = obstacle;
      }
      if (i == vertices.size() - 1) {
        obstacle->nextObstacle = obstacles_[obstacleNo];
        obstacle->nextObstacle->prevObstacle = obstacle;
      } 
      obstacle->unitDir_ = normalize(vertices[(i == vertices.size() - 1 ? 0 : i+1)] - vertices[i]);

      if (vertices.size() == 2) {
        obstacle->isConvex_ = true;
      } else {
        obstacle->isConvex_ = (leftOf(vertices[(i == 0 ? vertices.size() - 1 : i-1)], vertices[i], vertices[(i == vertices.size() - 1 ? 0 : i+1)]) >= 0);
      }

      obstacle->id_ = obstacles_.size();

      obstacles_.push_back(obstacle);
    }

    return obstacleNo;
  }

  void SFSimulator::doStep()
  {
    kdTree_->buildAgentTree();

	if (agents_.size() > 0)
	{
		addPlatformRotationXZ(getRotationDegreeSet().getRotationOY());
	    addPlatformRotationYZ(getRotationDegreeSet().getRotationOX());
	}

#pragma omp parallel for

    for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
      agents_[i]->computeNeighbors();
      agents_[i]->computeNewVelocity();
    }

#pragma omp parallel for

    for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
      agents_[i]->update();
    }

    globalTime_ += timeStep_;
  }

  size_t SFSimulator::getAgentMaxNeighbors(size_t agentNo) const
  {
    return agents_[agentNo]->maxNeighbors_;
  }

  float SFSimulator::getAgentMaxSpeed(size_t agentNo) const
  {
    return agents_[agentNo]->maxSpeed_;
  }

  float SFSimulator::getAgentNeighborDist(size_t agentNo) const
  {
    return agents_[agentNo]->neighborDist_;
  }

  const Vector2& SFSimulator::getAgentPosition(size_t agentNo) const
  {
    return agents_[agentNo]->position_;
  }

  const Vector2& SFSimulator::getAgentPrefVelocity(size_t agentNo) const
  {
    return agents_[agentNo]->prefVelocity_;
  }

  float SFSimulator::getAgentRadius(size_t agentNo) const
  {
    return agents_[agentNo]->radius_;
  }

  float SFSimulator::getAgentTimeHorizonObst(size_t agentNo) const
  {
    return agents_[agentNo]->timeHorizonObst_;
  }

  const Vector2& SFSimulator::getAgentVelocity(size_t agentNo) const
  {
    return agents_[agentNo]->velocity_;
  }

  float SFSimulator::getGlobalTime() const
  {
    return globalTime_;
  }

  size_t SFSimulator::getNumAgents() const
  {
    return agents_.size();
  }

  size_t SFSimulator::getNumObstacleVertices() const
  {
    return obstacles_.size();
  }

  const Vector2& SFSimulator::getObstacleVertex(size_t vertexNo) const
  {
    return obstacles_[vertexNo]->point_;
  }

  size_t SFSimulator::getNextObstacleVertexNo(size_t vertexNo) const
  {
    return obstacles_[vertexNo]->nextObstacle->id_;
  }

  size_t SFSimulator::getPrevObstacleVertexNo(size_t vertexNo) const
  {
    return obstacles_[vertexNo]->prevObstacle->id_;
  }

  float SFSimulator::getTimeStep() const
  {
    return timeStep_;
  }

  void SFSimulator::processObstacles()
  {
    kdTree_->buildObstacleTree();
  }

  bool SFSimulator::queryVisibility(const Vector2& point1, const Vector2& point2, float radius) const
  {
    return kdTree_->queryVisibility(point1, point2, radius);
  }
  
  void SFSimulator::setAgentDefaults(AgentPropertyConfig & apc)
  {
	 if (defaultAgent_ == 0) {
		defaultAgent_ = new Agent(this);
	 }
	 
	defaultAgent_->maxNeighbors_ = apc._maxNeighbors;
    defaultAgent_->maxSpeed_ = apc._maxSpeed;
    defaultAgent_->neighborDist_ = apc._neighborDist;
    defaultAgent_->radius_ = apc._radius;
	defaultAgent_->timeHorizonObst_ = apc._timeHorizon;
	defaultAgent_->accelerationCoefficient_ = apc._accelerationCoefficient;
	defaultAgent_->repulsiveAgent_ = apc._repulsiveAgent;
	defaultAgent_->repulsiveAgentFactor_ = apc._repulsiveAgentFactor;
	defaultAgent_->repulsiveObstacle_ = apc._repulsiveObstacle;
	defaultAgent_->repulsiveObstacleFactor = apc._repulsiveObstacleFactor;
	defaultAgent_->perception_ = apc._perception;
	defaultAgent_->friction_ = apc._friction;
    defaultAgent_->velocity_ = apc._velocity;
  }

  void SFSimulator::setAgentMaxNeighbors(size_t agentNo, size_t maxNeighbors)
  {
    agents_[agentNo]->maxNeighbors_ = maxNeighbors;
  }

  void SFSimulator::setAgentMaxSpeed(size_t agentNo, float maxSpeed)
  {
    agents_[agentNo]->maxSpeed_ = maxSpeed;
  }

  void SFSimulator::setAgentNeighborDist(size_t agentNo, float neighborDist)
  {
    agents_[agentNo]->neighborDist_ = neighborDist;
  }

  void SFSimulator::setAgentPosition(size_t agentNo, const Vector2& position)
  {
    agents_[agentNo]->position_ = position;
  }

  void SFSimulator::setAgentPrefVelocity(size_t agentNo, const Vector2& prefVelocity)
  {
    agents_[agentNo]->prefVelocity_ = prefVelocity;
  }

  void SFSimulator::setAgentRadius(size_t agentNo, float radius)
  {
    agents_[agentNo]->radius_ = radius;
  }

  void SFSimulator::setAgentTimeHorizonObst(size_t agentNo, float timeHorizonObst)
  {
    agents_[agentNo]->timeHorizonObst_ = timeHorizonObst;
  }

  void SFSimulator::setAgentVelocity(size_t agentNo, const Vector2& velocity)
  {
    agents_[agentNo]->velocity_ = velocity;
  }

  void SFSimulator::setTimeStep(float timeStep)
  {
    timeStep_ = timeStep;
  }

  void SFSimulator::setPlatformVelocity(Vector3 velocity)
  {
	  platformVelocity_ = velocity;
  }

  Vector3 SFSimulator::getPlatformVelocity()
  {
	  return platformVelocity_;
  }

  void SFSimulator::setAgentFriction(size_t agentNo, float friction)
  {
	  agents_[agentNo]->friction_ = friction;
  }

  float SFSimulator::getAgentFriction(size_t agentNo) const
  {
	  return agents_[agentNo]->friction_;
  }

  RotationDegreeSet SFSimulator::getRotationDegreeSet()
  {
	  return angleSet_;
  }

  void SFSimulator::setRotationDegreeSet(RotationDegreeSet set)
  {
	  angleSet_ = set;
  }

  void SFSimulator::setAdditionalForce(Vector3 velocity, RotationDegreeSet set)
  {
	  setPlatformVelocity(velocity);
	  setRotationDegreeSet(set);
  }


  void SFSimulator::addPlatformRotationXY(float value)
  {
	  float futureSum = platformRotationXY_ + value;

	  if (futureSum >= 2 * M_PI)
		  platformRotationXY_ = futureSum - 2 * M_PI;
	  else
		  platformRotationXY_ = futureSum;
  }

  void SFSimulator::addPlatformRotationXZ(float value)
  {
	  float futureSum = platformRotationXZ_ + value;

	  if (futureSum >= 2 * M_PI)
		  platformRotationXZ_ = futureSum - 2 * M_PI;
	  else
		  platformRotationXZ_ = futureSum;
  }

  void SFSimulator::addPlatformRotationYZ(float value)
  {
	  float futureSum = platformRotationYZ_ + value;

	  if (futureSum >= 2 * M_PI)
		  platformRotationYZ_ = futureSum - 2 * M_PI;
	  else
		  platformRotationYZ_ = futureSum;
  }


  float SFSimulator::getPlatformRotationXY()
  {
	  return platformRotationXY_;
  }

  float SFSimulator::getPlatformRotationXZ()
  {
	  return platformRotationXZ_;
  }

  float SFSimulator::getPlatformRotationYZ()
  {
	  return platformRotationYZ_;
  }

}
