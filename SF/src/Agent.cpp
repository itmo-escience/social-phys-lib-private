/*
 *  Agent.cpp
 *  SF Library.
 */

#include <algorithm>

#include "../include/Agent.h"
#include "../include/Obstacle.h"
#include "../include/KdTree.h"

namespace SF
{
	Agent::Agent(SFSimulator* sim) :
		id_(0),
		maxNeighbors_(0),
		accelerationBuffer_(0.0f),
		maxSpeed_(0.0f),
		neighborDist_(0.0f),
		radius_(0.0f),
		timeHorizonObst_(0.0f),
		obstaclePressure_(),
		repulsiveObstacle_(1 / repulsiveObstacle_),
		agentPressure_(),
		correction(),
		newVelocity_(),
		position_(),
		prefVelocity_(),
		previosPosition_(INT_MIN, INT_MIN),
		velocity_(),
		oldPlatformVelocity_(),
		obstacleNeighbors_(),
		agentNeighbors_(),
		attractiveTimeList_(),
		obstacleRadius_(0.1f),
		isForced_(false),
		sim_(sim)

	{ 
	  setNullSpeed(id_); 

	  // attractive section
	  for (size_t i = 0; i < sim->attractivePointList_.size(); i++)
	  {
		  attractiveTimeList_.push_back(0);
		  isUsedAttractivePoint_.push_back(false);
	  }
	}

	Agent::~Agent() { }

	void Agent::computeNeighbors()
	{
		// obstacle section
		obstacleNeighbors_.clear();
		auto rangeSq = sqr(timeHorizonObst_ * maxSpeed_ + radius_);
		sim_->kdTree_->computeObstacleNeighbors(this, rangeSq);

		// agent section
		agentNeighbors_.clear();
		if (maxNeighbors_ > 0) 
		{
			rangeSq = sqr(neighborDist_);
			sim_->kdTree_->computeAgentNeighbors(this, rangeSq);
		}
	}

	void Agent::setSpeedList(size_t index, float value)
	{
		if (speedList_.count(index) < 1)
			speedList_.insert(std::make_pair(index, value));
		else
			speedList_[index] = value;
	}

	void Agent::setNullSpeed(size_t id)
	{
		if (speedList_.count(id) < 1)
			setSpeedList(id, 0.0f);
	}

	float Agent::getPerception(Vector2 *arg1, Vector2 *arg2) const
	{
		if (getLength(*arg1) * getLength(*arg2) * getCos(*arg1, *arg2) > 0)
			return 1;

		return perception_;
	}


	float Agent::getNormalizedSpeed(float currentSpeed, float maxSpeed) const
	{
        if (currentSpeed <= maxSpeed)
            return 1;

        return maxSpeed / currentSpeed;
    }

	void Agent::getAccelerationTerm()
	{
		setNullSpeed(id_);

		if (previosPosition_.x() == INT_MIN && previosPosition_.y() == INT_MIN)
			previosPosition_ = position_;

		velocity_ = newVelocity_;

		if (fabs(prefVelocity_.x()) < TOLERANCE && fabs(prefVelocity_.y()) < TOLERANCE)
		{
			acceleration_ = 0.0f;
			setSpeedList(id_, 0.0f);
		}

		auto speed = speedList_[id_];
		auto mult = getNormalizedSpeed(speedList_[id_], maxSpeed_);
		auto tempAcceleration = 1 / relaxationTime_ * (maxSpeed_ - speedList_[id_]) * mult;

		if (!isForced_)
			acceleration_ += tempAcceleration;
		else acceleration_ = 0;

		position_ += velocity_ * sim_->timeStep_ * acceleration_;

		auto minLength = DBL_MAX;
		auto p = Vector2();
		auto hasIntersection = false;

		for (size_t i = 0; i < obstacleNeighbors_.size(); i++)
		{
			if (isIntersect(
				position_,
				previosPosition_,
				obstacleNeighbors_[i].second->point_,
				obstacleNeighbors_[i].second->nextObstacle->point_
				))
			{
				if (!hasIntersection)
					hasIntersection = true;

				auto intersection = getIntersection(
					position_,
					previosPosition_,
					obstacleNeighbors_[i].second->point_,
					obstacleNeighbors_[i].second->nextObstacle->point_
					);

				auto l = getLength(intersection - previosPosition_);
				p = intersection;

				if (l < minLength)
				{
					minLength = l;
					p = intersection;
				}
			}
		}

		if (hasIntersection)
		{
			auto difference = p - previosPosition_;
			auto m = (getLength(difference) - obstacleRadius_) / getLength(difference);

			if (getLength(difference) > obstacleRadius_ && getLength(difference) <= 2)
			{
				if (m >= 0 && m <= 1)
					position_ = previosPosition_ + difference * m;
				else
					position_ = previosPosition_;
			}

			if (getLength(difference) > obstacleRadius_ && getLength(difference) > 2)
				position_ = previosPosition_;

			if (getLength(difference) <= obstacleRadius_)
				position_ = previosPosition_;

			isForced_ = true;
		}
		else
			isForced_ = false;

		setSpeedList(id_, static_cast<float>(sqrt(pow((position_ - previosPosition_).x(), 2) + pow((position_ - previosPosition_).y(), 2))) / sim_->timeStep_);

		previosPosition_ = position_;
	}

	void Agent::getRepulsiveAgentForce()
	{
		for (size_t i = 0; i < agentNeighbors_.size(); i++)
		{
			auto agent = agentNeighbors_[i].second;
			setNullSpeed(agent->id_);
			auto pos = agent->position_;

			if (pos == position_)
				continue;

			auto velocity = agent->velocity_;

			auto y = agent->velocity_ * speedList_[agent->id_] * sim_->timeStep_;
			auto d = position_ - pos;
			auto radius = speedList_[agent->id_] * sim_->timeStep_;
			auto b = sqrt(sqr(getLength(d) + getLength(d - y)) - sqr(radius)) / 2;
			auto potential = repulsiveAgent_ * exp(-b / repulsiveAgent_);
			auto ratio = (getLength(d) + getLength(d - y)) / 2 * b;
			auto sum = (d / getLength(d) + (d - y) / getLength(d - y));
			auto force = potential * ratio * sum * getPerception(&position_, &pos) * repulsiveAgentFactor_;
			agentPressure_ = getLength(force);

			correction += force;
		}
	}

	void Agent::getRepulsiveObstacleForce()
	{
		auto forceSum = Vector2();
		auto maxForceLength = FLT_MIN;
		auto minDistanceToObstacle = FLT_MAX;

		std::vector<Vector2> nearestObstaclePointList;
		nearestObstaclePointList.clear();

		Vector2 sum;
		
		std::vector<Vector2> forces;
		forces.clear();

		for (size_t i = 0; i < obstacleNeighbors_.size(); i++)
		{
			setNullSpeed(id_);

			auto obstacle = obstacleNeighbors_[i].second;
			auto start = obstacle->point_;
			auto end = obstacle->nextObstacle->point_;
			auto closestPoint = getNearestPoint(&start, &end, &position_);

			auto hasSuchClosestPoint = false;

			for (size_t j = 0; j < nearestObstaclePointList.size(); j++)
			{
				auto l = nearestObstaclePointList[j] - closestPoint;
				if (fabsf(l.GetLengthSquared()) < TOLERANCE)
				{
					hasSuchClosestPoint = true;
					break;
				}
			}

			if (hasSuchClosestPoint)
				continue;

			nearestObstaclePointList.push_back(closestPoint);
		}

		for (size_t i = 0; i < obstacleNeighbors_.size(); i++)
		{
			auto obstacle = obstacleNeighbors_[i].second;
			auto start = obstacle->point_;
			auto end = obstacle->nextObstacle->point_;
			auto closestPoint = getNearestPoint(&start, &end, &position_);

			for (size_t j = 0; j < nearestObstaclePointList.size(); j++)
			{
				auto l = nearestObstaclePointList[j] - closestPoint;
				if (l.GetLengthSquared() > TOLERANCE)
				{
					if ((nearestObstaclePointList[j] - start).GetLengthSquared() < TOLERANCE || (nearestObstaclePointList[j] - end).GetLengthSquared() < TOLERANCE)
					nearestObstaclePointList.erase(nearestObstaclePointList.begin() + j);
				}
			}
		}

		for (size_t i = 0; i < nearestObstaclePointList.size(); i++)
		{
			auto closestPoint = nearestObstaclePointList[i];

			auto diff = position_ - closestPoint;
			auto distanceSquared = diff.GetLengthSquared();
			auto absoluteDistanceToObstacle = sqrt(distanceSquared);
			auto distance = absoluteDistanceToObstacle - radius_;
		
			if (absoluteDistanceToObstacle < minDistanceToObstacle)
				minDistanceToObstacle = absoluteDistanceToObstacle;

			auto forceAmount = repulsiveObstacleFactor_ * exp(-distance / repulsiveObstacle_);
			auto force = forceAmount * diff.normalized();

			forces.push_back(force);
			forceSum += force;
			
			auto length = getLength(force);

			if (maxForceLength < length)
				maxForceLength = length;
		}

		auto size = forces.size();
		float lengthSum = 0;
		for (size_t i = 0; i < size; i++)
			lengthSum += getLength(forces[i]);
		
		std::vector<float> forceWeightList;
		forceWeightList.clear();
		for (size_t i = 0; i < size; i++)
			forceWeightList.push_back(getLength(forces[i]) / lengthSum);
		
		auto total = Vector2();
		for (size_t i = 0; i < size; i++)
			total += forces[i] * forceWeightList[i];
		
		obstaclePressure_ = getLength(total);
		correction += total;

		if (size > 0)
			obstacleTrajectory_ = position_ + total * 10;
		else
			obstacleTrajectory_ = position_;
	}

	void Agent::getAttractiveForce()
	{
		auto time = sim_->attractiveTime_;
		auto attractivePointList = sim_->attractivePointList_;

		for (size_t i = 0; i < attractivePointList.size(); i++)
		{
			if (!isUsedAttractivePoint_[i])
			{
				if (getLength(attractivePointList[i] - position_) <= sim_->attractiveLength_)
					attractiveTimeList_[i] += sim_->timeStep_;

				if (attractiveTimeList_[i] <= time && attractiveTimeList_[i] > 0)
				{
					auto difference = normalize(position_ - attractivePointList[i]);

					auto first = sim_->repulsiveStrength_ * exp((2 * radius_ - getLength(difference)) / sim_->repulsiveRange_);;
					auto second = sim_->attractiveStrength_ * exp((2 * radius_ - getLength(difference)) / sim_->attractiveRange_);;

					auto add =  (first - second) * getPerception(&position_, &attractivePointList[i]) * difference;

					correction += add;
				}

				if (attractiveTimeList_[i] > time)
					isUsedAttractivePoint_[i] = true;
			}
		}
	}

	void Agent::getMovingPlatformForce()
	{
		if (sim_->rotationFuture_ != Vector3())
		{
			Vector3
				omega,
				dOmega,
				R = Vector3(position_.x(), position_.y(), 0),
				V = Vector3(velocity_.x(), velocity_.y(), 0),
				A = Vector3(),
				fixedOmega = Vector3(),
				fixedR = Vector3(),
				fixedV = Vector3(),
				fixedA = Vector3();

			float
				determinantPrefixCentralForceX,
				determinantPrefixCentralForceY,
				determinantCentralForceX,
				determinantCentralForceY,
				determinantTangentialForceX,
				determinantTangentialForceY,
				determinantCoriolisForceX,
				determinantCoriolisForceY;

			Vector2
				newVX = Vector2(),
				newVY = Vector2();

			if (fabs(sim_->rotationNow_.x()) > 0.001f)
			{
				auto parameterType = X;
				omega = getOmega(parameterType, NOW);
				dOmega = getDOmega(parameterType, NOW);

				Vector3
					prefixCentralForce,
					centralForce,
					tangentialForce,
					CoriolisForce;

				fixedR = Vector3(
					R.x() * cos(omega.y()) + R.z() * sin(omega.y()),
					R.y() * cos(omega.x()) + R.z() * sin(omega.y()),
					R.z() * cos(omega.x()) - R.y() * sin(omega.x()) + R.z() * cos(omega.y()) + R.x() * sin(omega.y()));

				fixedV = Vector3(
					V.x() * cos(omega.y()) + V.z() * sin(omega.y()),
					V.y() * cos(omega.x()) + V.z() * sin(omega.x()),
					V.z() * cos(omega.x()) - V.y() * sin(omega.x()) + V.z() * cos(omega.y()) + V.x() * sin(omega.y()));

				determinantPrefixCentralForceX = omega.y() * R.z() - omega.z() * R.y() - omega.x() * R.z() + omega.z() * R.x() + omega.x() * R.y() - omega.y() * R.x();
				prefixCentralForce =
					(determinantPrefixCentralForceX > 0) ?
					getCross(omega, R) :
					getCross(R, omega);

				determinantCentralForceX = omega.y() * prefixCentralForce.z() - omega.z() * prefixCentralForce.y() - omega.x() * prefixCentralForce.z() + omega.z() * prefixCentralForce.x() + omega.x() * prefixCentralForce.y() - omega.y() * prefixCentralForce.x();
				centralForce =
					(determinantCentralForceX > 0) ?
					getCross(omega, prefixCentralForce) :
					getCross(prefixCentralForce, omega);

				determinantTangentialForceX = dOmega.y() * R.z() - dOmega.z() * R.y() - dOmega.x() * R.z() + dOmega.z() * R.x() + dOmega.x() * R.y() - dOmega.y() * R.x();
				tangentialForce =
					(determinantTangentialForceX > 0) ?
					getCross(dOmega, R) :
					getCross(R, dOmega);

				determinantCoriolisForceX = omega.y() * V.z() - omega.z() * V.y() - omega.x() * V.z() + omega.z() * V.x() + omega.x() * V.y() - omega.y() * V.x();
				CoriolisForce =
					(determinantCoriolisForceX > 0) ?
					2 * getCross(omega, V) :
					2 * getCross(V, omega);

				fixedA = centralForce + tangentialForce - CoriolisForce;

				A = Vector3(fixedA.x() / cos(omega.x()), fixedA.y() / cos(omega.y()), 0);

				newVX = Vector2(A.x(), A.y());
			}

			if (fabs(sim_->rotationNow_.y()) > 0.001f)
			{
				auto parameterType = Y;
				omega = getOmega(parameterType, NOW);
				dOmega = getDOmega(parameterType, NOW);
				
				Vector3
					prefixCentralForce,
					centralForce,
					tangentialForce,
					CoriolisForce;

				fixedR = Vector3(
					R.x() * cos(omega.y()) + R.z() * sin(omega.y()),
					R.y() * cos(omega.x()) + R.z() * sin(omega.y()),
					R.z() * cos(omega.x()) - R.y() * sin(omega.x()) + R.z() * cos(omega.y()) - R.x() * sin(omega.y()));

				fixedV = Vector3(
					V.x() * cos(omega.y()) + V.z() * sin(omega.y()),
					V.y() * cos(omega.x()) + V.z() * sin(omega.x()),
					V.z() * cos(omega.x()) - V.y() * sin(omega.x()) + V.z() * cos(omega.y()) - V.x() * sin(omega.y()));

				determinantPrefixCentralForceY = omega.y() * R.z() - omega.z() * R.y() - omega.x() * R.z() + omega.z() * R.x() + omega.x() * R.y() - omega.y() * R.x();
				prefixCentralForce =
					(determinantPrefixCentralForceY > 0) ?
					getCross(omega, R) :
					getCross(R, omega);

				determinantCentralForceY = omega.y() * prefixCentralForce.z() - omega.z() * prefixCentralForce.y() - omega.x() * prefixCentralForce.z() + omega.z() * prefixCentralForce.x() + omega.x() * prefixCentralForce.y() - omega.y() * prefixCentralForce.x();
				centralForce =
					(determinantCentralForceY > 0) ?
					getCross(omega, prefixCentralForce) :
					getCross(prefixCentralForce, omega);

				determinantTangentialForceY = dOmega.y() * R.z() - dOmega.z() * R.y() - dOmega.x() * R.z() + dOmega.z() * R.x() + dOmega.x() * R.y() - dOmega.y() * R.x();
				tangentialForce =
					(determinantTangentialForceY > 0) ?
					getCross(dOmega, R) :
					getCross(R, dOmega);

				determinantCoriolisForceY = omega.y() * V.z() - omega.z() * V.y() - omega.x() * V.z() + omega.z() * V.x() + omega.x() * V.y() - omega.y() * V.x();
				CoriolisForce =
					(determinantCoriolisForceY > 0) ?
					2 * getCross(omega, V) :
					2 * getCross(V, omega);

				fixedA = centralForce + tangentialForce - CoriolisForce;

				A = Vector3(
					fixedA.x() / cos(omega.x()),
					fixedA.y() / cos(omega.y()),
					0);

				newVY = Vector2(A.x(), A.y());
			}

			auto result = (velocity_ + (newVX + newVY) * sim_->timeStep_);

			// heave
			auto platformVeclocity = sim_->getPlatformVelocity();
			
			float
				accelerationZ = platformVeclocity.z() * pow(sim_->timeStep_, 2),
				oldAccelerationZ = oldPlatformVelocity_.z() * pow(sim_->timeStep_, 2);

			auto difference = fabs(accelerationZ) - fabs(oldAccelerationZ);

			if (difference > 0)	
				result = result * (1 + fabs(difference));
			else
				result = result * (1 - fabs(difference));

			oldPlatformVelocity_ = platformVeclocity;

			correction += result * platformFactor_;
		}
	}

	/* Search for the best new velocity. */
	void Agent::computeNewVelocity()
	{
		if (prefVelocity_ * prefVelocity_ > sqr(radius_))
			newVelocity_ = normalize(prefVelocity_) * radius_;
		else
			newVelocity_ = prefVelocity_;

		correction = Vector2();

		getRepulsiveAgentForce();
		getRepulsiveObstacleForce();
		getAttractiveForce();

		if(sim_->IsMovingPlatform)
			getMovingPlatformForce();
    
		newVelocity_ += correction;
	}

  void Agent::insertAgentNeighbor(const Agent* agent, float& rangeSq)
  {
    if (this != agent) {
      const auto distSq = absSq(position_ - agent->position_);

      if (distSq < rangeSq) {
        if (agentNeighbors_.size() < maxNeighbors_) {
          agentNeighbors_.push_back(std::make_pair(distSq,agent));
        }
		auto i = agentNeighbors_.size() - 1;
        while (i != 0 && distSq < agentNeighbors_[i-1].first) {
          agentNeighbors_[i] = agentNeighbors_[i-1];
          --i;
        }
        agentNeighbors_[i] = std::make_pair(distSq, agent);

        if (agentNeighbors_.size() == maxNeighbors_) {
          rangeSq = agentNeighbors_.back().first;
        }
      }
    }
  }

	void Agent::insertObstacleNeighbor(const Obstacle* obstacle, float rangeSq)
	{
		const Obstacle* const nextObstacle = obstacle->nextObstacle;

		const auto distSq = distSqPointLineSegment(obstacle->point_, nextObstacle->point_, position_);

		if (distSq < rangeSq) 
		{
			obstacleNeighbors_.push_back(std::make_pair(distSq,obstacle));
      
			auto i = obstacleNeighbors_.size() - 1;
			while (i != 0 && distSq < obstacleNeighbors_[i-1].first) 
			{
				obstacleNeighbors_[i] = obstacleNeighbors_[i-1];
				--i;
			}
			
			obstacleNeighbors_[i] = std::make_pair(distSq, obstacle);
		}
	}

	void Agent::insertAgentNeighborsIndex(const Agent* agent, float& rangeSq)
	{
		if (this != agent) 
		{
			const auto distSq = absSq(position_ - agent->position_);

			if (distSq < rangeSq) 
			{
				agentNeighborsIndexList_.push_back(std::make_pair(agent->id_, distSq));
				auto i = agentNeighborsIndexList_.size() - 1;
        
				while (i != 0 && distSq < agentNeighborsIndexList_[i-1].second) 
				{
					agentNeighborsIndexList_[i] = agentNeighborsIndexList_[i - 1];
					--i;
				}

				agentNeighborsIndexList_[i] = std::make_pair(agent->id_, distSq);
			}
		}
	}

	Vector2 Agent::getNearestPoint(Vector2 *start, Vector2 *end, Vector2 *point) const
	{
		auto relativeEndPoint = *end - *start;
		auto relativePoint = *point - *start;
		double lambda = relativePoint * relativeEndPoint / relativeEndPoint.GetLengthSquared();

		if(lambda <= 0)
			return *start;
		else if(lambda >= 1)
			return *end;
		else
			return *start + static_cast<float>(lambda) * relativeEndPoint;
	}

	inline float getMinFloat(float a, float b)
	{
		return a < b ? a : b;
	}

	void Agent::update()
	{
		getAccelerationTerm();
	}

	Vector3 Agent::getCross(const Vector3 &left, const Vector3 &right) const
	{
		float 
			X,
			Y,
			Z;

		X = (left.y() * right.z()) - (left.z() * right.y());
        Y = (left.z() * right.x()) - (left.x() * right.z());
		Z = (left.x() * right.y()) - (left.y() * right.x());

		return Vector3(X, Y, Z);
	}

	double Agent::degreesToRadians(float degree) const
	{
        return degree * (M_PI / 180.0f);
    }

	double Agent::radiansToDegrees(float radian) const
	{
        return radian * (180.0f / M_PI);
    }

	Vector3 Agent::getRoll(ParameterType pt, TimeType tt) const
	{
		Vector3 rotation;
		float value = 0;

		if(tt == PAST)
			rotation = sim_->rotationPast_;
		else if(tt == PAST2NOW)
			rotation = sim_->rotationPast2Now_;
		else if(tt == NOW)
			rotation = sim_->rotationNow_;
		else if(tt == NOW2FUTURE)
			rotation == sim_->rotationNow2Future_;
		if(tt == FUTURE)
			rotation = sim_->rotationFuture_;

		if(pt == X)
			value = rotation.x();
		
		if(pt == Y)
			value = rotation.y();

		if (pt == Z)
			value = rotation.z();

		return Vector3(value, value, value);
	}

	Vector3 Agent::getOmega(ParameterType pt, TimeType tt)
	{
		float value = 0;
		
		if(tt == NOW)
			value = (getRoll(pt, NOW2FUTURE).x() - getRoll(pt, PAST2NOW).x()) / sim_->timeStep_;
		else if(tt == NOW2FUTURE)
			value = (getRoll(pt, FUTURE).x() - getRoll(pt, NOW).x()) / sim_->timeStep_;
		else if(tt == PAST2NOW)
			value = (getRoll(pt, NOW).x() - getRoll(pt, PAST).x()) / sim_->timeStep_;
			
		if(pt == X)
			return Vector3(value, 0, 0);
		
		if(pt == Y)
			return Vector3(0, value, 0);

		if (pt == Z)
			return Vector3(0, 0, value);

		return Vector3();
	}

	Vector3 Agent::getDOmega(ParameterType pt, TimeType tt)
	{
		float value = 0;

		if(tt == NOW)
			value = (getOmega(pt, NOW2FUTURE).x() - getOmega(pt, PAST2NOW).x()) / sim_->timeStep_;
			
		if(pt == X)
			return Vector3(value, 0, 0);
		
		if(pt == Y)
			return Vector3(0, value, 0);

		if (pt == Z)
			return Vector3(0, 0, value);

		return Vector3();
	}

	bool Agent::isIntersect(Vector2 a, Vector2 b, Vector2 c, Vector2 d) const
	{
		auto v1 = (d.x() - c.x())*(a.y() - c.y()) - (d.y() - c.y())*(a.x() - c.x());
		auto v2 = (d.x() - c.x())*(b.y() - c.y()) - (d.y() - c.y())*(b.x() - c.x());
		auto v3 = (b.x() - a.x())*(c.y() - a.y()) - (b.y() - a.y())*(c.x() - a.x());
		auto v4 = (b.x() - a.x())*(d.y() - a.y()) - (b.y() - a.y())*(d.x() - a.x());
		
		return (v1 * v2 < TOLERANCE) && (v3 * v4 < 0);
	}

	Vector2 Agent::getIntersection(Vector2 a, Vector2 b, Vector2 c, Vector2 d) const
	{
		float 
			x1 = a.x(), 
			x2 = b.x(), 
			x3 = c.x(), 
			x4 = d.x(), 
			y1 = a.y(), 
			y2 = b.y(), 
			y3 = c.y(), 
			y4 = d.y();

		auto x = ((x3*y4 - x4*y3)*(x2 - x1) - (x1*y2 - x2*y1)*(x4 - x3)) / ((y1 - y2)*(x4 - x3) - (y3 - y4)*(x2 - x1));
		auto y = ((y3 - y4)*x - (x3*y4 - x4*y3)) / (x4 - x3);

		return Vector2(x, y);
	}

	SimpleMatrix Agent::getRotationX(float angle) const
	{
		auto result = SimpleMatrix();	// identity

		auto c = cos(angle);
		auto s = sin(angle);

		result.m22 = c;
		result.m23 = s;
		result.m32 = -s;
		result.m33 = c;

		return result;
	}

	SimpleMatrix Agent::getRotationY(float angle) const
	{
		auto result = SimpleMatrix();	// identity

		auto c = cos(angle);
		auto s = sin(angle);

		result.m11 = c;
		result.m13 = -s;
		result.m31 = s;
		result.m33 = c;

		return result;
	}

	SimpleMatrix Agent::getRotationZ(float angle) const
	{
		auto result = SimpleMatrix();	// identity

		auto c = cos(angle);
		auto s = sin(angle);

		result.m11 = c;
		result.m12 = s;
		result.m21 = -s;
		result.m22 = c;

		return result;
	}
}
