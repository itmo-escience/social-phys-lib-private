/*
* SFSimulator.cpp
* SF Library
*
* Copyright (c) 2008-2010 University of North Carolina at Chapel Hill.
* All rights reserved.
*
* Permission to use, copy, modify, and distribute this software and its
* documentation for educational, research, and non-profit purposes, without
* fee, and without a written agreement is hereby granted, provided that the
* above copyright notice, this paragraph, and the following four paragraphs
* appear in all copies.
*
* Permission to incorporate this software into commercial products may be
* obtained by contacting the Office of Technology Development at the University
* of North Carolina at Chapel Hill <otd@unc.edu>.
*
* This software program and documentation are copyrighted by the University of
* North Carolina at Chapel Hill. The software program and documentation are
* supplied "as is," without any accompanying services from the University of
* North Carolina at Chapel Hill or the authors. The University of North
* Carolina at Chapel Hill and the authors do not warrant that the operation of
* the program will be uninterrupted or error-free. The end-user understands
* that the program was developed for research purposes and is advised not to
* rely exclusively on the program for any reason.
*
* IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE
* AUTHORS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
* CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS
* SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT
* CHAPEL HILL OR THE AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
* DAMAGE.
*
* THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY
* DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY
* STATUTORY WARRANTY OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS ON
* AN "AS IS" BASIS, AND THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE
* AUTHORS HAVE NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, UPDATES,
* ENHANCEMENTS, OR MODIFICATIONS.
*
* Please send all bug reports to <geom@cs.unc.edu>.
*
* The authors may be contacted via:
*
* Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
* Dept. of Computer Science
* 201 S. Columbia St.
* Frederick P. Brooks, Jr. Computer Science Bldg.
* Chapel Hill, N.C. 27599-3175
* United States of America
*
* <http://gamma.cs.unc.edu/RVO2/>
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
		rotationPast_(),
		rotationPast2Now_(),
		rotationNow_(),
		rotationNow2Future_(),
		rotationFuture_(),
		IsMovingPlatform(false),
		agents_(),
		defaultAgent_(nullptr),
		globalTime_(0.0f),
		kdTree_(nullptr),
		obstacles_(),
		timeStep_(1.0f),
		platformVelocity_(),
		platformRotationXY_(0),
		platformRotationXZ_(0),
		platformRotationYZ_(0),
		ID()
	{
		kdTree_ = new KdTree(this);
	}

	SFSimulator::~SFSimulator()
	{
		delete defaultAgent_;

		for (auto id : ID)
			if (id != -1)
				delete agents_[id];

		for (size_t i = 0; i < obstacles_.size(); ++i)
			delete obstacles_[i];

		delete kdTree_;
	}

	size_t SFSimulator::getAgentNumAgentNeighbors(size_t agentNo) const
	{
		auto id = ID[agentNo];

		if(id != -1)
			return agents_[id]->agentNeighbors_.size();
	}

	size_t SFSimulator::getAgentAgentNeighbor(size_t agentNo, size_t neighborNo) const
	{
		auto id = ID[agentNo];

		if(id != -1)
			return agents_[id]->agentNeighbors_[neighborNo].second->id_;
	}

	size_t SFSimulator::getAgentObstacleNeighbor(size_t agentNo, size_t neighborNo) const
	{
		auto id = ID[agentNo];

		if(id != -1)
			return agents_[id]->obstacleNeighbors_[neighborNo].second->id_;
	}

	size_t SFSimulator::getAgentNumObstacleNeighbors(size_t agentNo) const
	{
		auto id = ID[agentNo];

		if(id != -1)
			return agents_[id]->obstacleNeighbors_.size();
	}

	size_t SFSimulator::addAgent(const Vector2& position)
	{
		if (defaultAgent_ == nullptr)
			return SF_ERROR;

		auto agent = new Agent(this);

		agent->position_ = position;
		agent->maxNeighbors_ = defaultAgent_->maxNeighbors_;
		agent->maxSpeed_ = defaultAgent_->maxSpeed_;
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
		ID.push_back(agent->id_);

		return agents_.size() - 1;
	}

	size_t SFSimulator::addAgent(
		const Vector2& position,
		float neighborDist,
		size_t maxNeighbors,
		float timeHorizon,		// never used
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
		const Vector2& velocity
		)
	{
		auto agent = new Agent(this);

		agent->position_ = position;
		agent->maxNeighbors_ = maxNeighbors;
		agent->maxSpeed_ = maxSpeed;
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
		ID.push_back(agent->id_);

		return agents_.size() - 1;
	}

	size_t SFSimulator::addObstacle(const std::vector<Vector2>& vertices)
	{
		if (vertices.size() < 2)
			return SF_ERROR;

		auto obstacleNo = obstacles_.size();

		for (size_t i = 0; i < vertices.size(); ++i) {
			auto obstacle = new Obstacle();
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

	void SFSimulator::doStep()
	{
		auto s = agents_.size();

		kdTree_->buildAgentTree();

		if (agents_.size() > 0)
		{
			addPlatformRotationXZ(getRotationDegreeSet().getRotationOY());
			addPlatformRotationYZ(getRotationDegreeSet().getRotationOX());
		}

#pragma omp parallel for

		for (int i = 0; i < static_cast<size_t>(agents_.size()); ++i)
		{
			auto id = ID[i];

			if(id != -1)
			{
				agents_[id]->computeNeighbors();
				agents_[id]->computeNewVelocity();
			}
		}

#pragma omp parallel for

		for (int i = 0; i < static_cast<size_t>(agents_.size()); ++i)
		{
			auto id = ID[i];

			if (id != -1)
				agents_[id]->update();
		}

		globalTime_ += timeStep_;
	}

	size_t SFSimulator::getAgentMaxNeighbors(size_t agentNo) const
	{
		auto id = ID[agentNo];

		if(id != -1)
			return agents_[id]->maxNeighbors_;
	}

	float SFSimulator::getAgentMaxSpeed(size_t agentNo) const
	{
		auto id = ID[agentNo];

		if(id != -1)
			return agents_[id]->maxSpeed_;
	}

	float SFSimulator::getAgentNeighborDist(size_t agentNo) const
	{
		auto id = ID[agentNo];

		if(id != -1)
			return agents_[id]->neighborDist_;
	}

	const Vector2& SFSimulator::getAgentPosition(size_t agentNo) const
	{
		auto id = ID[agentNo];

		if(id != -1)
			return agents_[id]->position_;
	}

	const Vector2& SFSimulator::getAgentPrefVelocity(size_t agentNo) const
	{
		auto id = ID[agentNo];

		if(id != -1)
		return agents_[id]->prefVelocity_;
	}

	float SFSimulator::getAgentRadius(size_t agentNo) const
	{
		auto id = ID[agentNo];

		if(id != -1)
			return agents_[id]->radius_;
	}

	float SFSimulator::getAgentTimeHorizonObst(size_t agentNo) const
	{
		auto id = ID[agentNo];

		if(id != -1)
			return agents_[id]->timeHorizonObst_;
	}

	const Vector2& SFSimulator::getAgentVelocity(size_t agentNo) const
	{
		auto id = ID[agentNo];

		if(id != -1)
			return agents_[id]->velocity_;
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

	void SFSimulator::processObstacles() const
	{
		kdTree_->buildObstacleTree();
	}

	bool SFSimulator::queryVisibility(const Vector2& point1, const Vector2& point2, float radius) const
	{
		return kdTree_->queryVisibility(point1, point2, radius);
	}

	void SFSimulator::setAgentDefaults(AgentPropertyConfig & apc)
	{
		if (defaultAgent_ == nullptr)
			defaultAgent_ = new Agent(this);


		defaultAgent_->maxNeighbors_ = apc._maxNeighbors;
		defaultAgent_->maxSpeed_ = apc._maxSpeed;
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

	void SFSimulator::setAgentMaxNeighbors(size_t agentNo, size_t maxNeighbors)
	{
		auto id = ID[agentNo];

		if(id != -1)
			agents_[id]->maxNeighbors_ = maxNeighbors;
	}

	void SFSimulator::setAgentMaxSpeed(size_t agentNo, float maxSpeed)
	{
		auto id = ID[agentNo];

		if(id != -1)
			agents_[id]->maxSpeed_ = maxSpeed;
	}

	void SFSimulator::setAgentNeighborDist(size_t agentNo, float neighborDist)
	{
		auto id = ID[agentNo];

		if(id != -1)
			agents_[id]->neighborDist_ = neighborDist;
	}

	void SFSimulator::setAgentPosition(size_t agentNo, const Vector2& position)
	{
		auto id = ID[agentNo];

		if(id != -1)
			agents_[id]->position_ = position;
	}

	void SFSimulator::setAgentPrefVelocity(size_t agentNo, const Vector2& prefVelocity)
	{
		auto id = ID[agentNo];
			
		if(id != -1)
			agents_[id]->prefVelocity_ = prefVelocity;
	}

	void SFSimulator::setAgentRadius(size_t agentNo, float radius)
	{
		auto id = ID[agentNo];

		if(id != -1)
			agents_[id]->radius_ = radius;
	}

	void SFSimulator::setAgentTimeHorizonObst(size_t agentNo, float timeHorizonObst)
	{
		auto id = ID[agentNo];

		if(id != -1)
			agents_[id]->timeHorizonObst_ = timeHorizonObst;
	}

	void SFSimulator::setAgentVelocity(size_t agentNo, const Vector2& velocity)
	{
		auto id = ID[agentNo];

		if(id != -1)
			agents_[id]->velocity_ = velocity;
	}

	void SFSimulator::setTimeStep(float timeStep)
	{
		timeStep_ = timeStep;
	}

	void SFSimulator::setPlatformVelocity(const Vector3 &velocity)
	{
		platformVelocity_ = velocity;
	}

	Vector3 SFSimulator::getPlatformVelocity()
	{
		return platformVelocity_;
	}

	void SFSimulator::setAgentFriction(size_t agentNo, float friction)
	{
		auto id = ID[agentNo];

		if(id != -1)
			agents_[id]->friction_ = friction;
	}

	float SFSimulator::getAgentFriction(size_t agentNo) const
	{
		auto id = ID[agentNo];

		if(id != -1)
			return agents_[id]->friction_;
	}

	RotationDegreeSet SFSimulator::getRotationDegreeSet()
	{
		return angleSet_;
	}

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

	void SFSimulator::setAdditionalForce(const Vector3 &velocity, const RotationDegreeSet &set)
	{
		setPlatformVelocity(velocity);
		setRotationDegreeSet(set);
	}

	void SFSimulator::setAttractiveForce(const std::vector<Vector2> &pointList, float attractiveStrength, float repulsiveStrength, float attractiveRange, float repulsiveRange, float attractiveTime, float length)
	{
		attractivePointList_ = pointList;

		attractiveTime_ = attractiveTime;
		attractiveLength_ = length;

		attractiveStrength_ = attractiveStrength;
		repulsiveStrength_ = repulsiveStrength;
		attractiveRange_ = attractiveRange;
		repulsiveRange_ = repulsiveRange;
	}

	void SFSimulator::addPlatformRotationXY(float value)
	{
		auto futureSum = platformRotationXY_ + value;

		if (futureSum >= 2 * M_PI)
			platformRotationXY_ = futureSum - 2 * M_PI;
		else
			platformRotationXY_ = futureSum;
	}

	void SFSimulator::addPlatformRotationXZ(float value)
	{
		auto futureSum = platformRotationXZ_ + value;

		if (futureSum >= 2 * M_PI)
			platformRotationXZ_ = futureSum - 2 * M_PI;
		else
			platformRotationXZ_ = futureSum;
	}

	void SFSimulator::addPlatformRotationYZ(float value)
	{
		auto futureSum = platformRotationYZ_ + value;

		if (futureSum >= 2 * M_PI)
			platformRotationYZ_ = futureSum - 2 * M_PI;
		else
			platformRotationYZ_ = futureSum;
	}


	double SFSimulator::getPlatformRotationXY() const
	{
		return platformRotationXY_;
	}

	double SFSimulator::getPlatformRotationXZ() const
	{
		return platformRotationXZ_;
	}

	double SFSimulator::getPlatformRotationYZ() const
	{
		return platformRotationYZ_;
	}

	std::vector<size_t> SFSimulator::getAgentNeighboursIndexList(size_t index, float radius)
	{
		std::vector<size_t> result;
		if (agents_.size() > 0)
		{
			auto id = ID[index];

			if (index >= agents_.size() || id == -1)
				result.push_back(0);
			else
			{
				auto agent = agents_[id];
				auto rangeSq = sqr(radius);

				agent->agentNeighborsIndexList_.clear();
				this->kdTree_->computeAgentNeighborsIndexList(agent, rangeSq);

				for (size_t i = 0; i < agent->agentNeighborsIndexList_.size(); i++)
					result.push_back(agent->agentNeighborsIndexList_[i].first);
			}
		}
		else
			result.push_back(0);

		return result;
	}

	void SFSimulator::deleteAgent(size_t index)
	{
		agents_[index]->isDeleted_ = true;

		ID[index] = -1;
	}

	std::vector<size_t> SFSimulator::getDeletedIDList() const
	{
		auto result = std::vector<size_t>();

		for (auto a : agents_)
			if (a->isDeleted_)
				result.push_back(a->id_);

		return result;
	}

	std::vector<size_t> SFSimulator::getCountOfAliveAndDead() const
	{
		auto result = std::vector<size_t>();

		size_t
			alive = 0,
			dead = 0;

		for (auto a : agents_)
			a->isDeleted_ ? dead++ : alive++;
					
		result.push_back(agents_.size());
		result.push_back(alive);
		result.push_back(dead);

		return result;
	}

	void SFSimulator::updateSFParameters(float newRepulsiveAgent_, float newRepulsiveAgentFactor_, float newRepulsiveObstacle_, float newRepulsiveObstacleFactor_)
	{
#pragma omp parallel for

		for (int i = 0; i < static_cast<size_t>(agents_.size()); ++i)
		{
			auto id = ID[i];
			if (id != -1)
			{
				agents_[id]->repulsiveAgent_ = newRepulsiveAgent_;
				agents_[id]->repulsiveAgentFactor_ = newRepulsiveAgentFactor_;
				agents_[id]->repulsiveObstacle_ = newRepulsiveObstacle_;
				agents_[id]->repulsiveObstacleFactor_ = newRepulsiveObstacleFactor_;
			}
		}

	}

	double SFSimulator::getAgentPressure(size_t index)
	{
		auto id = ID[index];

		if(id != -1)
			return agents_[id]->agentPressure_;
	}

	double SFSimulator::getObstaclePressure(size_t index)
	{
		auto id = ID[index];

		if(id != -1)
			return agents_[id]->obstaclePressure_;
	}

	Vector2 SFSimulator::getObstacleTrajectory(size_t index)
	{
		auto id = ID[index];

		if(id != -1)
			return agents_[id]->obstacleTrajectory_;
	}
}
