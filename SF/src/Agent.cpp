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
	ForceAcceleration::ForceAcceleration(float a, int d)
	{
		acceleration = a, direction = d;
	}

	int ForceAcceleration::getDirection()
	{
		return direction;
	}

	int ForceAcceleration::setDirectionForZ(float z)
	{
		if(z > 0)
			return 1;
		else if(z < 0)
			return -1;
	}
		
	void ForceAcceleration::update(float a, int d)
	{
		acceleration = a, direction = d;
	}

	float ForceAcceleration::getMultCoefficient()
	{
		if(direction < 0)
		{
			if(acceleration < 0)
				return 0.75;
			else if(acceleration > 0)
				return 1.25;
			else
				return 1;
		}
		else if(direction > 0)
		{
			if(acceleration < 0)
				return 1.25;
			else if(acceleration > 0)
				return 0.75;
			else
				return 1;
		}
		else
		{
			return 1;
		}
	}

	Agent::Agent(SFSimulator* sim) : 
			agentNeighbors_(), 
			maxNeighbors_(0), 
			maxSpeed_(0.0f), 
			neighborDist_(0.0f), 
			newVelocity_(),
			obstacleNeighbors_(), 
			position_(),
			prefVelocity_(), 
			radius_(0.0f), 
			sim_(sim), 
			direction_(1),
			timeHorizonObst_(0.0f), 
			velocity_(), 
			accelerationBuffer_(0.0f),
			previosPosition_(INT_MIN, INT_MIN), 
			forceAcceleration_(0, 0),
			oldPlatformVelocity_(),
			id_(0)
  { setNullSpeed(id_); }

  Agent::~Agent() { }

  void Agent::computeNeighbors()
  {
    obstacleNeighbors_.clear();
    float rangeSq = sqr(timeHorizonObst_ * maxSpeed_ + radius_);
    sim_->kdTree_->computeObstacleNeighbors(this, rangeSq);

    agentNeighbors_.clear();
    if (maxNeighbors_ > 0) {
      rangeSq = sqr(neighborDist_);
      sim_->kdTree_->computeAgentNeighbors(this, rangeSq);
    }
  }

  void Agent::setSpeedList(int index, float value)
  {
	  if (speedList_.count(index) < 1)
		  speedList_.insert(std::make_pair(index, value));
	  else
		  speedList_[index] = value;
  }

  void Agent::setNullSpeed(int id)
  {
	  if (speedList_.count(id) < 1)
		  setSpeedList(id, 0.0f);
  }

  float Agent::getPerception(Vector2 *arg1, Vector2 *arg2)
  {
	  if (getLength(*arg1) * getLength(*arg2) * getCos(*arg1, *arg2) > 0)
         return 1;

	  return perception_;
  }


  float Agent::getNormalizedSpeed(float currentSpeed, float maxSpeed)
    {
        if (currentSpeed <= maxSpeed)
            return 1;

        return maxSpeed / currentSpeed;
    }

  /* Search for the best new velocity. */
  void Agent::computeNewVelocity()
  {
	if (prefVelocity_ * prefVelocity_ > sqr(radius_))
	    newVelocity_ = normalize(prefVelocity_) * radius_;
    else
		newVelocity_ = prefVelocity_;

    Vector2 correction = Vector2();

    // <F2>
	for (int i = 0; i < agentNeighbors_.size(); i++)
    {
        setNullSpeed(agentNeighbors_[i].second->id_);
        Vector2 pos = agentNeighbors_[i].second->position_;
        Vector2 velocity = agentNeighbors_[i].second->velocity_;

		Vector2 y = agentNeighbors_[i].second->velocity_ * speedList_[agentNeighbors_[i].second->id_] * sim_->timeStep_;
		Vector2 d = position_ - pos;
		float radius = speedList_[agentNeighbors_[i].second->id_] * sim_->timeStep_;
		float b = sqrt(sqr(getLength(d) + getLength(d - y)) - sqr(radius)) / 2;
		float potential = repulsiveAgent_ * exp(-b / repulsiveAgent_);
		float ratio = (getLength(d) + getLength(d - y)) / 2 * b;
		Vector2 sum = (d / getLength(d) + (d - y) / getLength(d - y));

		correction += potential * ratio * sum * getPerception(&position_, &pos) * repulsiveAgentFactor_;

	}
	// </F2>
	
    // <F3>
	float minDistanceSquared = INT_MAX;
	Vector2 minDiff = Vector2();
	repulsiveObstacle_ = 1 / repulsiveObstacle_;

	for (int i = 0; i < obstacleNeighbors_.size(); i++)
    {
		setNullSpeed(id_);

		Vector2 start = obstacleNeighbors_[i].second->point_;
		Vector2 end = obstacleNeighbors_[i].second->nextObstacle->point_;
        Vector2 closestPoint = getNearestPoint(&start, &end, &position_);

        Vector2 diff = position_ - closestPoint;

        float distanceSquared = diff.GetLengthSquared();
        if (distanceSquared < minDistanceSquared)
        {
            minDistanceSquared = distanceSquared;
            minDiff = diff;
        }
    }

	float distance = sqrt(minDistanceSquared) - radius_;
	float forceAmount = repulsiveObstacleFactor * exp(-distance / repulsiveObstacle_);
        
    correction += forceAmount * minDiff.normalized();
    // </F3>
	
	// <F5>
	//Vector3 pv = sim_->getPlatformVelocity();

	/*!
	 *	@brief	2D Roration section
	 */
	// rotation center
	/*Vector3 center = sim_->getRotationDegreeSet().getCenter();	
	
	// relative values	
	float 
		diffX = position_.x() - center.x(),
		diffY = position_.y() - center.y(),
		diffZ = -center.z();

	// rotation radius
	float 
		radiusXOY = sqrt(pow(diffX, 2) + pow(diffY, 2)),
		radiusYOZ = sqrt(pow(diffY, 2) + pow(diffZ, 2)),
		radiusXOZ = sqrt(pow(diffX, 2) + pow(diffZ, 2));			

	float mult = 20;

	// rotaion degree set
	float 
		angleX = sim_->getRotationDegreeSet().getRotationOX() * sim_->timeStep_,
		angleY = sim_->getRotationDegreeSet().getRotationOY() * sim_->timeStep_,
		angleZ = sim_->getRotationDegreeSet().getRotationOZ() * sim_->timeStep_;

	// angle projection
	float 
		angleXY, 
		angleXZ, 
		angleYZ;

	float 
		currentAngleBySin,
		currentAngleByCos;
	
	Vector3
		newPosition,
		rotationVector;

	// angleXY
	if(angleZ != 0)
	{
		currentAngleBySin = asin(diffY / radiusXOY),
		currentAngleByCos = acos(diffX / radiusXOY);

		if (diffX < 0 && !(diffX < 0 && diffY < 0))
			angleXY = currentAngleByCos;
		else if(diffX < 0 && diffY < 0)
			angleXY = M_PI - currentAngleBySin;
		else
			angleXY = currentAngleBySin;

		// relative positions X
		newPosition += Vector3(radiusXOY * cos(angleZ + angleXY) + center.x(), radiusXOY * sin(angleZ + angleXY) + center.y(), center.z());
		rotationVector = Vector3(position_.x(), position_.y(), 0) - newPosition;
	}

	// angleXZ
	if(angleY != 0)
	{
		angleXZ = sim_->getPlatformRotationXZ();

		Vector3 XZforce = 
			Vector3(radiusXOZ * cos(angleXZ - angleY) + center.x(), center.y(), radiusXOZ * sin(angleXZ - angleY) + center.z()) - 
			Vector3(radiusXOZ * cos(angleXZ) + center.x(), center.y(), radiusXOZ * sin(angleXZ) + center.z());
		
		pv += mult * XZforce;
	}
	
	// angleYZ
	if(angleX != 0)
	{
		angleYZ = sim_->getPlatformRotationYZ();

		Vector3 YZforce = 
			Vector3(center.x(), radiusYOZ * cos(angleYZ - angleX) + center.y(), radiusYOZ * sin(angleYZ - angleX) + center.z()) - 
			Vector3(center.x(), radiusYOZ * cos(angleYZ) + center.y(), radiusYOZ * sin(angleYZ) + center.z());
		
		pv += mult * YZforce;
	}

	correction += Vector2(rotationVector.x(), rotationVector.y());*/

	/*!
	 *	@brief	Forvard section
	 */
	/*if(isNotPlaneCase(pv))
	{
		Vector2 totalFieldProjection = getVectorProjectionXY(pv);
		float inclineAngle = fabs(getInclineAngle(pv));
		float rotationAngle = fabs(getRotationAngle(pv));
		float criticalCorrection = getCriticalAnglesCorrection(inclineAngle, rotationAngle);
		float verticalShiftDirection = (pv.z() >= 0) ? -1 : 1;

		Vector2 total = totalFieldProjection * criticalCorrection * verticalShiftDirection / friction_; 

		correction += total;
	}
	
	oldPlatformVelocity_ = pv;*/
	// </F5>

	// <F5>
	if(sim_->rotationInFuture_ != Vector3())
	{
		Vector3 
			omega,
			dOmega,
			R, 
			V, 
			A,
			fixedOmega,
			fixedR, 
			fixedV,
			fixedA;

		Vector2
			newVX = Vector2(),
			newVY = Vector2();

		float 
			radianX = degreesToRadians(sim_->rotationNow_.x()),
			radianY = degreesToRadians(sim_->rotationNow_.y());

		if(fabs(radianX) > 0.001f)
		{
			ParameterType parameterType = X;
			omega = getOmega(parameterType, sim_->globalTime_, sim_->timeStep_, radianX);
			dOmega = getDOmega(parameterType, sim_->globalTime_, sim_->timeStep_, radianX);

			R = Vector3(position_.x(), position_.y(), 0);
			V = Vector3(velocity_.x(), velocity_.y(), 0);
	
			fixedR = Vector3(
				R.x() * cos(omega.y()) + R.z() * sin(omega.y()),
				R.y() * cos(omega.x()) + R.z() * sin(omega.y()),
				R.z() * cos(omega.x()) - R.y() * sin(omega.x()) + R.z() * cos(omega.y()) + R.x() * sin(omega.y()));

			fixedV = Vector3(
				V.x() * cos(omega.y()) + V.z() * sin(omega.y()),
				V.y() * cos(omega.x()) + V.z() * sin(omega.x()),
				V.z() * cos(omega.x()) - V.y() * sin(omega.x()) + V.z() * cos(omega.y()) + V.x() * sin(omega.y()));

			fixedA = getCross(omega, getCross(omega, fixedR)) + getCross(dOmega, fixedR) - 2 * getCross(omega, fixedV);
	
			A = Vector3(fixedA.x() / cos(omega.x()), fixedA.y() / cos(omega.y()), 0);

			newVX = Vector2(A.x(), A.y());
		}

		if(fabs(radianY) > 0.001f)
		{
			ParameterType parameterType = Y;
			omega = getOmega(parameterType, sim_->globalTime_, sim_->timeStep_, radianY);
			dOmega = getDOmega(parameterType, sim_->globalTime_, sim_->timeStep_, radianY);
	
			R = Vector3(position_.x(), position_.y(), 0);
			V = Vector3(velocity_.x(), velocity_.y(), 0);
	
			fixedR = Vector3(
				R.x() * cos(omega.y()) + R.z() * sin(omega.y()),
				R.y() * cos(omega.x()) + R.z() * sin(omega.y()),
				R.z() * cos(omega.x()) - R.y() * sin(omega.x()) + R.z() * cos(omega.y()) + R.x() * sin(omega.y()));

			fixedV = Vector3(
				V.x() * cos(omega.y()) + V.z() * sin(omega.y()),
				V.y() * cos(omega.x()) + V.z() * sin(omega.x()),
				V.z() * cos(omega.x()) - V.y() * sin(omega.x()) + V.z() * cos(omega.y()) + V.x() * sin(omega.y()));

			fixedA = getCross(omega, getCross(omega, fixedR)) + getCross(dOmega, fixedR) - 2 * getCross(omega, fixedV);
	
			A = Vector3(
				fixedA.x() / cos(omega.x()),
				fixedA.y() / cos(omega.y()),
				0);

			newVY = Vector2(A.x(), A.y());
		}

		Vector2 result = (velocity_ + (newVX + newVY) * sim_->timeStep_);

	
		Vector3 platformVeclocity = sim_->getPlatformVelocity();
		float 
			accelerationZ = platformVeclocity.z() * pow(sim_->timeStep_, 2),
			oldAccelerationZ = oldPlatformVelocity_.z() * pow(sim_->timeStep_, 2);

		float difference = fabs(accelerationZ) - fabs(oldAccelerationZ);

		if(difference > 0)	// positive difference
			result = result * (1 + fabs(difference));
		else
			result = result * (1 - fabs(difference));

		oldPlatformVelocity_ = platformVeclocity;


		correction += result * 0.2f;
	}
	// </F5>
	
    
    newVelocity_ += correction;
  }

  void Agent::insertAgentNeighbor(const Agent* agent, float& rangeSq)
  {
    if (this != agent) {
      const float distSq = absSq(position_ - agent->position_);

      if (distSq < rangeSq) {
        if (agentNeighbors_.size() < maxNeighbors_) {
          agentNeighbors_.push_back(std::make_pair(distSq,agent));
        }
        size_t i = agentNeighbors_.size() - 1;
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

    const float distSq = distSqPointLineSegment(obstacle->point_, nextObstacle->point_, position_);

    if (distSq < rangeSq) {
      obstacleNeighbors_.push_back(std::make_pair(distSq,obstacle));
      
      size_t i = obstacleNeighbors_.size() - 1;
      while (i != 0 && distSq < obstacleNeighbors_[i-1].first) {
        obstacleNeighbors_[i] = obstacleNeighbors_[i-1];
        --i;
      }
      obstacleNeighbors_[i] = std::make_pair(distSq, obstacle);
    }
  }

  void Agent::insertAgentNeighborsIndex(const Agent* agent, float& rangeSq)
  {if (this != agent) {
      const float distSq = absSq(position_ - agent->position_);

      if (distSq < rangeSq) {
		agentNeighborsIndexList_.push_back(std::make_pair(agent->id_, distSq));
		size_t i = agentNeighborsIndexList_.size() - 1;
        
		while (i != 0 && distSq < agentNeighborsIndexList_[i-1].second) {
          agentNeighborsIndexList_[i] = agentNeighborsIndexList_[i - 1];
          --i;
        }

		agentNeighborsIndexList_[i] = std::make_pair(agent->id_, distSq);
       }
      }
  }

	Vector2 Agent::getNearestPoint(Vector2 *start, Vector2 *end, Vector2 *point)
	{
		Vector2 relativeEndPoint = *end - *start;
		Vector2 relativePoint = *point - *start;
		double lambda = relativePoint * relativeEndPoint / relativeEndPoint.GetLengthSquared();

		if(lambda <= 0)
			return *start;
		else if(lambda >= 1)
			return *end;
		else
			return *start + (float)lambda * relativeEndPoint;
	}

	inline float getMinFloat(float a, float b)
	{
		return a < b ? a : b;
	}

	float Agent::getCriticalAnglesCorrection(float a, float b)
	{	
		return 1 / getMinFloat(a, b);
	}

	bool Agent::isNotPlaneCase(Vector3 s)
	{
		return !(s.x() == 0 && s.y() == 0 && s.z() == 0);
	}

	float Agent::getInclineAngle(Vector3 s)
	{
		if(s.x() != 0)
			return getCos(Vector2(1, 0), getVectorProjectionXZ(s));	
		else if(s.y() != 0)
			return getCos(Vector2(0, 1), getVectorProjectionXZ(s));	
		else
			return 1;
	}

	float Agent::getRotationAngle(Vector3 s)
	{
		if(s.x() != 0)
			return getCos(Vector2(1, 0), getVectorProjectionXY(s));
		else if(s.y() != 0)
			return getCos(Vector2(0, 1), getVectorProjectionXY(s));
		else return 1;
	}

	Vector2 Agent::getVectorProjectionXY(Vector3 s)
	{
		return Vector2(s.x(), s.y());
	}

	Vector2 Agent::getVectorProjectionXZ(Vector3 s)
	{
		return Vector2(s.x(), s.z());
	}

	Vector2 Agent::getVectorProjectionYZ(Vector3 s)
	{
		return Vector2(s.y(), s.z());
	}

  void Agent::update()
  {
	setNullSpeed(id_);

	if (previosPosition_.x() == INT_MIN && previosPosition_.y() == INT_MIN)
      previosPosition_ = position_;

	velocity_ = newVelocity_;

	if (fabs(prefVelocity_.x()) < 0.0001f && fabs(prefVelocity_.y()) < 0.0001f)
    {
		acceleration_ = 0.0f;
        setSpeedList(id_, 0.0f);
    }

    float mult = getNormalizedSpeed(speedList_[id_], maxSpeed_);
    float tempAcceleration = 1 / relaxationTime_ * (maxSpeed_ - speedList_[id_]) * mult;

    if (!isForced_)
    {
		acceleration_ += tempAcceleration;
        accelerationBuffer_ += tempAcceleration;
    }
    else
    {
        isForced_ = false;
		acceleration_ += accelerationBuffer_ * accelerationCoefficient_;
        accelerationBuffer_ = 0;
    }
	
	float diff = 0;

	position_ += velocity_ * sim_->timeStep_ * acceleration_;

    setSpeedList(id_, (float) sqrt(pow((position_ - previosPosition_).x(), 2) + pow((position_ - previosPosition_).y(), 2)) / sim_->timeStep_);

    previosPosition_ = position_;
  }

	Vector3 Agent::getCross(Vector3 left, Vector3 right)
	{
		float X,
			Y,
			Z;

		X = (left.y() * right.z()) - (left.z() * right.y());
        Y = (left.z() * right.x()) - (left.x() * right.z());
		Z = (left.x() * right.y()) - (left.y() * right.x());

		return Vector3(X, Y, Z);
	}

	float Agent::degreesToRadians(float degree)
    {
        return degree * (M_PI / 180.0f);
    }

	float Agent::radiansToDegrees(float radian)
    {
        return radian * (180.0f / M_PI);
    }

	Vector3 Agent::getRoll(ParameterType pt, float t, float radian)
	{
		float value;

		if(pt == X)
			value = sim_->getRotationDegreeSet().getRotationOX();
		
		if(pt == Y)
			value = sim_->getRotationDegreeSet().getRotationOY();

		return Vector3(value, value, 0);
	}

	Vector3 Agent::getOmega(ParameterType pt, float t, float dt, float radian)
	{
		if(pt == X)
		{
			float value = 
				(getRoll(pt, t + dt / 2, radian).x() - 
				getRoll(pt, t - dt / 2, radian).x()) / dt;
			
			return Vector3(value, 0, 0);
		}
		
		if(pt == Y)
		{
			float value = 
				(getRoll(pt, t + dt / 2, radian).y() - 
				getRoll(pt, t - dt / 2, radian).y()) / dt;
			
			return Vector3(0, value, 0);
		}
	}

	Vector3 Agent::getDOmega(ParameterType pt, float t, float dt, float radian)
	{
		if(pt == X)
		{
			float value	= 
				(getOmega(pt, t + dt / 2, dt, radian).x() - 
				getOmega(pt, t - dt / 2, dt, radian).x()) / dt;
			
			return Vector3(value, 0, 0);
		}

		if(pt == Y)
		{
			float value	= 
				(getOmega(pt, t + dt / 2, dt, radian).y() - 
				getOmega(pt, t - dt / 2, dt, radian).y()) / dt;
			
			return Vector3(0, value, 0);
		}
	}
}
