// This is an independent project of an individual developer. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

#include <algorithm>

#include "../include/Agent.h"
#include "../include/Obstacle.h"
#include "../include/KdTree.h"
#include <iostream>
#ifdef __linux__
#include "../include/stacktrace.h"
#define PRINT_STACK_TRACE Util::print_stacktrace();
#define ISNAN std::isnan
#else
#define PRINT_STACK_TRACE
#define ISNAN _isnan
#endif

namespace SF
{
	/// <summary> Defines an agent in the simulation </summary>
	/// <param name="sim"> The simulator instance </param>
	Agent::Agent(SFSimulator* sim) :
		isDeleted_(false),					// mark for deleting 
		isForced_(false),					// mark preventing high speed after meeting with the obstacle 
		id_(0),								// unique identifier 
		maxNeighbors_(0),					// max count of neighbors
		acceleration_(0),					// acceleration buffer preventing high speed after meeting with the obstacle 
		relaxationTime_(0),					// time of approching the max speed  
		maxSpeed_(0.0f),					// max speed 
		force_(1.0f),					// force
		neighborDist_(0.0f),				// min distance for neighbors 
		radius_(0.0f),						// range around agent defined by radius 
		timeHorizonObst_(0.0f),				// iteration time interval
		accelerationCoefficient_(0),		// accelereation factor coefficient for acceleration term 
		repulsiveAgent_(0),					// repulsive exponential agent coefficient for agent repulsive force 
		repulsiveAgentFactor_(0),			// repulsive factor agent coefficient for agent repulsive force 
		repulsiveObstacle_(0),				// repulsive exponential obstacle coefficient for obstacle repulsive force 
		repulsiveObstacleFactor_(0),		// repulsive factor obstacle coefficient for obstacle repulsive force 
		obstacleRadius_(0.1f),				// min agent to obstacle distance 
		platformFactor_(0),					// factor platform coefficient for moving platform force 
		perception_(0),						// angle of perception 
		friction_(0),						// friction platform coefficient for moving platform force
		obstaclePressure_(),				// total pressure for obstacle repulsive force 
		agentPressure_(),					// total pressure for agent repulsive force 
		correction(),						// current correction vector
		newVelocity_(),						// new result vector
		position_(),						// current position
		prefVelocity_(),					// pre-computed velocity
		previosPosition_(INT_MIN, INT_MIN),	// saved previous position
		velocity_(),						// current result vector
		obstacleForce_(),					// graphic representation of obst force
		agentForce_(),						// graphic representation of agent force
		oldPlatformVelocity_(),				// saved previous platform velocity
		obstacleNeighbors_(),				// list of neighbor obstacles
		agentNeighbors_(),					// list of neighbor agents
		agentNeighborsIndexList_(),			// list of neighbor agent identifiers
		attractiveIds_(),					// list of attractive agent identifiers
		speedList_(),						// map of agent speeds
		sim_(sim),							// simulator instance
		TOLERANCE(0.00001f)
	{ 
		
	  setNullSpeed(id_);
	}

	/// <summary> Destructor </summary>
	Agent::~Agent()
	{ }

	/// <summary> Computes the neighbors of this agent </summary>
	void Agent::computeNeighbors()
	{
		try
		{
			// obstacle section
			obstacleNeighbors_.clear();
			float rangeSq = sqr(timeHorizonObst_ * maxSpeed_ + radius_);
			sim_->kdTree_->computeObstacleNeighbors(this, rangeSq);

			// agent section
			agentNeighbors_.clear();
			if (maxNeighbors_ > 0)
			{
				rangeSq = sqr(neighborDist_);
				sim_->kdTree_->computeAgentNeighbors(this, rangeSq);
			}
		}
		catch(const std::runtime_error& re)
		{
			// speciffic handling for runtime_error
			std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cerr << "Runtime error: " << re.what() << std::endl;
			PRINT_STACK_TRACE
			exit(EXIT_FAILURE);
		}
		catch(const std::exception& ex)
		{
			// speciffic handling for all exceptions extending std::exception, except
			// std::runtime_error which is handled explicitly
			std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cerr << "Error occurred: " << ex.what() << std::endl;
			PRINT_STACK_TRACE
			exit(EXIT_FAILURE);
		}
		catch (...)
		{
			std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			PRINT_STACK_TRACE
			exit(EXIT_FAILURE);
		}
	}

	/// <summary> Updates speed list containing speed values corresponding each agent  </summary>
	/// <param name="index"> Agent ID </param>
	/// <param name="value"> New speed value </param>
	void Agent::setSpeedList(size_t index, float value)
	{
		//if (speedList_.count(index) < 1)
		//	speedList_.insert(std::make_pair(index, value));
		//else
			speedList_[index] = value;
	}

	/// <summary> Null-initialization for speed list </summary>
	/// <param name="id"> Agent ID </param>
	void Agent::setNullSpeed(size_t id)
	{
		if (speedList_.count(id) < 1)
			setSpeedList(id, 0.0f);
	}

	/// <summary> Finds perception of some point </summary>
	/// <param name="from"> Agent position </param>
	/// <param name="to"> Position of percepted point </param>
	/// <returns> The angle of perception </returns>
	float Agent::getPerception(Vector2 *from, Vector2 *to) const
	{
		if (getLength(*from) * getLength(*to) * getCos(*from, *to) > 0)
			return 1;

		return perception_;
	}

	/// <summary> Normalizing the velocity </summary>
	/// <param name="currentSpeed"> Current speed </param>
	/// <param name="maxSpeed"> Max speed </param>
	/// <returns> Normalized speed </returns>
	float Agent::getNormalizedSpeed(float currentSpeed, float maxSpeed) const
	{
        if (currentSpeed <= maxSpeed)
            return 1;

        return maxSpeed / currentSpeed;
    }

	/// <summary> Acceleration term method </summary>
	void Agent::getAccelerationTerm()
	{
		try
		{
			setNullSpeed(id_);

			if ((fabs(previosPosition_.x() - INT_MIN) < SF_EPSILON) && (fabs(previosPosition_.y() - INT_MIN) < SF_EPSILON))
				previosPosition_ = position_;

			if (newVelocity_.x() != newVelocity_.x() //Is NAN check
				|| newVelocity_.y() != newVelocity_.y())
			{
				printf("error! getAccelerationTerm \n");
			}

			velocity_ = newVelocity_;

			//if (newVelocity_.x() != newVelocity_.x() //Is NAN check
			//	|| newVelocity_.y() != newVelocity_.y())
			if(ISNAN(newVelocity_.x()) || ISNAN(newVelocity_.y()))
			{
				printf("error!getAccelerationTerm \n ");
			}

			if (fabs(prefVelocity_.x()) < TOLERANCE && fabs(prefVelocity_.y()) < TOLERANCE)
			{
				acceleration_ = 0.0f;
				setSpeedList(id_, 0.0f);
			}

			float speed = speedList_[id_];
			float mult = getNormalizedSpeed(speedList_[id_], maxSpeed_);
			float tempAcceleration = 1 / relaxationTime_ * (maxSpeed_ - speedList_[id_]) * mult;

			if (!isForced_)
				acceleration_ += tempAcceleration;
			else acceleration_ = 0;

			position_ += velocity_ * sim_->timeStep_ * acceleration_;

			//if (newVelocity_.x() != newVelocity_.x() //Is NAN check
			//	|| newVelocity_.y() != newVelocity_.y())
			if(ISNAN(newVelocity_.x()) || ISNAN(newVelocity_.y()))
			{
				printf("error!\n");
			}

			double minLength = DBL_MAX;
			Vector2 p;
			bool hasIntersection = false;

			//for(auto on: obstacleNeighbors_)
			for (size_t i = 0; i < obstacleNeighbors_.size(); i++)
			{
				const Obstacle* obstacle = obstacleNeighbors_[i].second;
				if (isIntersect(position_, previosPosition_, obstacle->point_, obstacle->nextObstacle->point_))
				{
					if (!hasIntersection)
						hasIntersection = true;
					Vector2 intersectionPoint = getIntersection(position_, previosPosition_, obstacle->point_, obstacle->nextObstacle->point_);
					float l = getLength(intersectionPoint - previosPosition_);
					p = intersectionPoint;

					if (l < minLength)
					{
						minLength = l;
						p = intersectionPoint;
					}
				}
			}

			if (hasIntersection)
			{
				//			auto difference = p - previosPosition_;
				//			auto m = (getLength(difference) - obstacleRadius_) / getLength(difference);
				//
				//			if (getLength(difference) > obstacleRadius_ && getLength(difference) <= TOLERANCE)
				//			{
				//				if (m >= 0 && m <= TOLERANCE / 2)
				//					position_ = previosPosition_ + difference * m;
				//				else
				//					position_ = previosPosition_;
				//			}
				//
				//			if (getLength(difference) > obstacleRadius_ && getLength(difference) > TOLERANCE)
				//				position_ = previosPosition_;
				//
				//			if (getLength(difference) <= obstacleRadius_)
				//				position_ = previosPosition_;
				//
				//			isForced_ = true;
				position_ = previosPosition_;
			}
			else
				isForced_ = false;

			setSpeedList(id_, static_cast<float>(sqrt(pow((position_ - previosPosition_).x(), 2) + pow((position_ - previosPosition_).y(), 2))) / sim_->timeStep_);

			previosPosition_ = position_;
		}
		catch(const std::runtime_error& re)
		{
			// speciffic handling for runtime_error
			std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cerr << "Runtime error: " << re.what() << std::endl;
			PRINT_STACK_TRACE
			exit(EXIT_FAILURE);
		}
		catch(const std::exception& ex)
		{
			// speciffic handling for all exceptions extending std::exception, except
			// std::runtime_error which is handled explicitly
			std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cerr << "Error occurred: " << ex.what() << std::endl;
			PRINT_STACK_TRACE
			exit(EXIT_FAILURE);
		}
		catch (...)
		{
			std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			PRINT_STACK_TRACE
			exit(EXIT_FAILURE);
		}
	}

	/// <summary> Repulsive agent force </summary>
	void Agent::getRepulsiveAgentForce()
	{
		try
		{
			//std::cout << "agent id: " << this->id_ << "  agentNeighbors_.size(): " <<  agentNeighbors_.size() << std::endl;
			double pressure = 0;
			Vector2 forceSum;// = Vector2();
			float maxForceLength = FLT_MIN;
			speedList_.clear();
			//for(auto an: agentNeighbors_)
			for(size_t i = 0; i < agentNeighbors_.size(); i++)
			{
				const Agent* ag= agentNeighbors_[i].second;
				setNullSpeed(ag->id_);
				Vector2 pos = ag->position_;

				if (position_ == pos)
					continue;

				Vector2 velocity = ag->velocity_;

				Vector2 y = ag->velocity_ * speedList_[ag->id_] * sim_->timeStep_;
				Vector2 d = position_ - pos;
				float radius = speedList_[ag->id_] * sim_->timeStep_;

				float b = sqrt(sqr(getLength(d) + getLength(d - y)) - sqr(radius)) / 2;
				float potential = repulsiveAgent_ * exp(-b / repulsiveAgent_);
				float ratio = (getLength(d) + getLength(d - y)) / 2 * b;
				Vector2 sum = (d / getLength(d) + (d - y) / getLength(d - y));
				Vector2 force = potential * ratio * sum * getPerception(&position_, &pos) * repulsiveAgentFactor_;

				//if(ISNAN(force.x()) || ISNAN(force.y()))
				//{
				//	std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				//}

				float length = getLength(force);
				pressure += length;

				if (maxForceLength < length)
					maxForceLength = length;

				//if(ISNAN(forceSum.x()) || ISNAN(forceSum.y()))
				//{
				//	std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				//}

				forceSum += force * ag->force_;

				//if(ISNAN(forceSum.x()) || ISNAN(forceSum.y()))
				//{
				//	std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				//}
			}

			//std::cout << "speedList_.size(): " << speedList_.size() << std::endl;

			float forceSumLength = getLength(forceSum);

			if (forceSumLength > maxForceLength)
			{
				float coeff = maxForceLength / forceSumLength;
				forceSum *= coeff;

				//if(ISNAN(forceSum.x()) || ISNAN(forceSum.y()))
				//{
				//	std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
				//}
			}

			float maxPressure = repulsiveAgent_ * repulsiveAgentFactor_ * pow(10 * repulsiveAgent_, 2) * 0.8 / 10;
			agentPressure_ = (pressure < maxPressure) ? pressure / maxPressure : 1.0;

			correction += forceSum;

			//if(ISNAN(correction.x()) || ISNAN(correction.y()))
			//{
			//std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			//}

			agentForce_ = forceSum;
		}
		catch(const std::runtime_error& re)
		{
			// speciffic handling for runtime_error
			std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cerr << "Runtime error: " << re.what() << std::endl;
			PRINT_STACK_TRACE
			exit(EXIT_FAILURE);
		}
		catch(const std::exception& ex)
		{
			// speciffic handling for all exceptions extending std::exception, except
			// std::runtime_error which is handled explicitly
			std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cerr << "Error occurred: " << ex.what() << std::endl;
			PRINT_STACK_TRACE
			exit(EXIT_FAILURE);
		}
		catch(...)
		{
			std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			PRINT_STACK_TRACE
			exit(EXIT_FAILURE);
		}
	}

	/// <summary> Repulsive obstacle force </summary>
	void Agent::getRepulsiveObstacleForce()
	{
		try
		{
			Vector2 forceSum = Vector2();
			float maxForceLength = FLT_MIN;
			float minDistanceToObstacle = FLT_MAX;

			std::vector<Vector2> nearestObstaclePointList;
			nearestObstaclePointList.reserve(obstacleNeighbors_.size());
			//nearestObstaclePointList.clear();

			Vector2 sum;

			std::vector<Vector2> forces;
			forces.clear();

			//for (auto on : obstacleNeighbors_)
			for (size_t i = 0; i < obstacleNeighbors_.size(); i++)
			{
				setNullSpeed(id_);

				const Obstacle* obstacle = obstacleNeighbors_[i].second;
				Vector2 start = obstacle->point_;
				Vector2 end = obstacle->nextObstacle->point_;
				Vector2 closestPoint = getNearestPoint(&start, &end, &position_);

				bool hasSuchClosestPoint = false;

				//for (auto nop : nearestObstaclePointList)
				for (size_t j = 0; j < nearestObstaclePointList.size(); j++)
				{
					Vector2 l = nearestObstaclePointList[j] - closestPoint;
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

			//for (auto on : obstacleNeighbors_)
			for (size_t i = 0; i < obstacleNeighbors_.size(); i++)
			{
				const Obstacle* obstacle = obstacleNeighbors_[i].second;
				Vector2 start = obstacle->point_;
				Vector2 end = obstacle->nextObstacle->point_;
				Vector2 closestPoint = getNearestPoint(&start, &end, &position_);

				size_t j = 0;
				for (size_t k = 0; k < nearestObstaclePointList.size(); k++)
				{
					Vector2 nop = nearestObstaclePointList[k];
					Vector2 l = nop - closestPoint;
					if (l.GetLengthSquared() > TOLERANCE)
					{
						if ((nop - start).GetLengthSquared() < TOLERANCE || (nop - end).GetLengthSquared() < TOLERANCE)
							nearestObstaclePointList.erase(nearestObstaclePointList.begin() + j);
					}

					j++;
				}
			}

			//for (auto nop : nearestObstaclePointList)
			for (size_t i = 0; i < nearestObstaclePointList.size(); i++)
			{
				Vector2 closestPoint = nearestObstaclePointList[i];

				Vector2 diff = position_ - closestPoint;
				float distanceSquared = diff.GetLengthSquared();
				float absoluteDistanceToObstacle = sqrt(distanceSquared);
				float distance = absoluteDistanceToObstacle - radius_;

				if (absoluteDistanceToObstacle < minDistanceToObstacle)
					minDistanceToObstacle = absoluteDistanceToObstacle;

				float forceAmount = repulsiveObstacleFactor_ * exp(-distance / repulsiveObstacle_);
				Vector2 force = forceAmount * diff.normalized();

				if(ISNAN(force.x()) || ISNAN(force.y()))
				{
					printf("error! force is NAN\n");
					getchar();
				}

				forces.push_back(force);
				forceSum += force;

				float length = getLength(force);

				if (maxForceLength < length)
					maxForceLength = length;
			}

			float lengthSum = 0;
			//for (auto force : forces)
			for(size_t i = 0; i < forces.size(); i++)
			{
				lengthSum += getLength(forces[i]);
			}

			std::vector<float> forceWeightList;
			forceWeightList.reserve(forces.size());
			//forceWeightList.clear();
			//for (auto force : forces)
			for(size_t i = 0; i < forces.size(); i++)
			{
				if(ISNAN(forces[i].x()) || ISNAN(forces[i].y()))
				{
					printf("error! forces[%d] is NAN \n", i);
					getchar();
				}

				if (lengthSum == 0)
				{
					forceWeightList.push_back(getLength(forces[i]) / std::numeric_limits<float>::min());
					if(ISNAN(forceWeightList[forceWeightList.size() - 1]))
					{
						printf("error! forceWeightList[%d] is NAN\n", forceWeightList.size() - 1 );
						getchar();
					}
				}
				else
				{
					forceWeightList.push_back(getLength(forces[i]) / lengthSum);
				}

			}


			Vector2 total = Vector2();
			for (size_t i = 0; i < forces.size(); i++)
				total += forces[i] * forceWeightList[i];

			if(ISNAN(total.x()) || ISNAN(total.y()))
			{
				printf("error!\n");
				getchar();
			}

			obstaclePressure_ = getLength(total);
			correction += total;

			if(ISNAN(correction.x()) || ISNAN(correction.y()))
			{
				printf("error!\n");
				getchar();
			}

			obstacleForce_ = position_ + total;
		}
		catch(const std::runtime_error& re)
		{
			// speciffic handling for runtime_error
			std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cerr << "Runtime error: " << re.what() << std::endl;
			PRINT_STACK_TRACE
			exit(EXIT_FAILURE);
		}
		catch(const std::exception& ex)
		{
			// speciffic handling for all exceptions extending std::exception, except
			// std::runtime_error which is handled explicitly
			std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cerr << "Error occurred: " << ex.what() << std::endl;
			PRINT_STACK_TRACE
			exit(EXIT_FAILURE);
		}
		catch(...)
		{
			std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			PRINT_STACK_TRACE
			exit(EXIT_FAILURE);
		}
	}


//	/// <summary> Repulsive obstacle force </summary>
//	void Agent::getRepulsiveObstacleForce()
//	{
//		auto forceSum = Vector2();
//		auto maxForceLength = FLT_MIN;
//		auto minDistanceToObstacle = FLT_MAX;
//
//		std::vector<Vector2> nearestObstaclePointList;
//		nearestObstaclePointList.clear();
//
//		Vector2 sum;
//		
//		std::vector<Vector2> forces;
//		forces.clear();
//
//		for(auto on: obstacleNeighbors_)
//		{
//			setNullSpeed(id_);
//
//			auto obstacle = on.second;
//			auto start = obstacle->point_;
//			auto end = obstacle->nextObstacle->point_;
//			auto closestPoint = getNearestPoint(&start, &end, &position_);
//
//			auto hasSuchClosestPoint = false;
//
//			for(auto nop: nearestObstaclePointList)
//			{
//				auto l = nop - closestPoint;
//				if (fabsf(l.GetLengthSquared()) < TOLERANCE)
//				{
//					hasSuchClosestPoint = true;
//					break;
//				}
//			}
//
//			if (hasSuchClosestPoint)
//				continue;
//
//			nearestObstaclePointList.push_back(closestPoint);
//		}
//
//		for(auto on: obstacleNeighbors_)
//		{
//			auto obstacle = on.second;
//			auto start = obstacle->point_;
//			auto end = obstacle->nextObstacle->point_;
//			auto closestPoint = getNearestPoint(&start, &end, &position_);
//
//			size_t j = 0;
//			for (size_t i = 0; i < nearestObstaclePointList.size(); i++)
//			{
//				auto nop = nearestObstaclePointList[i];
//				auto l = nop - closestPoint;
//				if (l.GetLengthSquared() > TOLERANCE)
//				{
//					if ((nop - start).GetLengthSquared() < TOLERANCE || (nop - end).GetLengthSquared() < TOLERANCE)
//					nearestObstaclePointList.erase(nearestObstaclePointList.begin() + j);
//				}
//
//				j++;
//			}
//		}
//
//		for(auto nop: nearestObstaclePointList)
//		{
//			auto closestPoint = nop;
//
//			auto diff = position_ - closestPoint;
//			auto distanceSquared = diff.GetLengthSquared();
//			auto absoluteDistanceToObstacle = sqrt(distanceSquared);
//			auto distance = absoluteDistanceToObstacle - radius_;
//		
//			if (absoluteDistanceToObstacle < minDistanceToObstacle)
//				minDistanceToObstacle = absoluteDistanceToObstacle;
//
//			auto forceAmount = repulsiveObstacleFactor_ * exp(-distance / repulsiveObstacle_);
//			auto force = forceAmount * diff.normalized();
//
//			forces.push_back(force);
//			forceSum += force;
//			
//			auto length = getLength(force);
//
//			if (maxForceLength < length)
//				maxForceLength = length;
//		}
//
//		float lengthSum = 0;
//		for(auto force: forces)
//			lengthSum += getLength(force);
//		
//		std::vector<float> forceWeightList;
//		forceWeightList.clear();
//		for (auto force : forces) 
//		{
//			if (lengthSum == 0)
//				forceWeightList.push_back(getLength(force) / std::numeric_limits<double>::min());
//			else
//				forceWeightList.push_back(getLength(force) / lengthSum);
//		}
//		
//		
//		auto total = Vector2();
//		for (size_t i = 0; i < forces.size(); i++)
//			total += forces[i] * forceWeightList[i];
//		
//		obstaclePressure_ = getLength(total);
//		correction += total;
//
////		if (forces.size() > 0)
//			// TODO: coeff of smth else
//			obstacleForce_ = position_ + total;
////		else
////			obstacleForce_ = position_;
//	}

/// <summary> Attractive force </summary>
void Agent::getAttractiveForce()
{
	try
	{
		if(attractiveIds_.size() > 0)
		{
			//for(auto ai: attractiveIds_)
			for(int i = 0; i < attractiveIds_.size(); i++)
			{
				if (attractiveIds_[i] == id_)
					continue;

				Vector2 anp = sim_->agents_.GetAgentWithID(attractiveIds_[i])->position_;

				Vector2 pairPosition = anp;
				Vector2 normalizedDistance = normalize(position_ - anp);

				float first = sim_->repulsiveStrength_ * exp((2 * radius_ - getLength(normalizedDistance)) / sim_->repulsiveRange_);
				float second = sim_->attractiveStrength_ * exp((2 * radius_ - getLength(normalizedDistance)) / sim_->attractiveRange_);

				Vector2 add = (first - second) * getPerception(&position_, &(anp)) * normalizedDistance;

				correction += add;

				if(ISNAN(correction.x()) || ISNAN(correction.y()))
				{
					printf("error!\n");
					getchar();
				}
			}
		}
	}
	catch(const std::runtime_error& re)
	{
		// speciffic handling for runtime_error
		std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cerr << "Runtime error: " << re.what() << std::endl;
		PRINT_STACK_TRACE
		exit(EXIT_FAILURE);
	}
	catch(const std::exception& ex)
	{
		// speciffic handling for all exceptions extending std::exception, except
		// std::runtime_error which is handled explicitly
		std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		std::cerr << "Error occurred: " << ex.what() << std::endl;
		PRINT_STACK_TRACE
		exit(EXIT_FAILURE);
	}
	catch(...)
	{
		std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
		PRINT_STACK_TRACE
		exit(EXIT_FAILURE);
	}
}

	/// <summary> Moving platform force </summary>
	void Agent::getMovingPlatformForce()
	{
		try
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
					ParameterType parameterType = X;
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
					ParameterType parameterType = Y;
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

				Vector2 result = (velocity_ + (newVX + newVY) * sim_->timeStep_);

				// heave
				// TODO good heave
				Vector3 platformVeclocity = sim_->getPlatformVelocity();

				float
					accelerationZ = platformVeclocity.z() * pow(sim_->timeStep_, 2),
					oldAccelerationZ = oldPlatformVelocity_.z() * pow(sim_->timeStep_, 2);

				float difference = fabs(accelerationZ) - fabs(oldAccelerationZ);

				if (difference > 0)	
					result = result * (1 + fabs(difference));
				else
					result = result * (1 - fabs(difference));

				oldPlatformVelocity_ = platformVeclocity;

				correction += result * platformFactor_;

				if(ISNAN(correction.x()) || ISNAN(correction.y()))
				{
					printf("error!\n");
					getchar();
				}
			}
		}
		catch(const std::runtime_error& re)
		{
			// speciffic handling for runtime_error
			std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cerr << "Runtime error: " << re.what() << std::endl;
			PRINT_STACK_TRACE
			exit(EXIT_FAILURE);
		}
		catch(const std::exception& ex)
		{
			// speciffic handling for all exceptions extending std::exception, except
			// std::runtime_error which is handled explicitly
			std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cerr << "Error occurred: " << ex.what() << std::endl;
			PRINT_STACK_TRACE
			exit(EXIT_FAILURE);
		}
		catch (...)
		{
			std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			PRINT_STACK_TRACE
			exit(EXIT_FAILURE);
		}
	}

	/// <summary> Search for the best new velocity </summary>
	void Agent::computeNewVelocity()
	{
		try
		{
			if(ISNAN(newVelocity_.x()) || ISNAN(newVelocity_.y()))
			{
				std::cerr << "error! newVelocity_ is NAN" << std::endl;
				std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			}

			if (prefVelocity_ * prefVelocity_ > sqr(radius_))
				newVelocity_ = normalize(prefVelocity_) * radius_;
			else
				newVelocity_ = prefVelocity_;

			if(ISNAN(newVelocity_.x()) || ISNAN(newVelocity_.y()))//Is NAN check
			{
				std::cerr << "error! newVelocity_ is NAN" << std::endl;
				std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			}

			correction = Vector2();

			if(ISNAN(correction.x()) || ISNAN(correction.y()))
			{
				std::cerr << "error! correction is NAN" << std::endl;
				std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			}

			getRepulsiveAgentForce();
			if(ISNAN(correction.x()) || ISNAN(correction.y()))
			{
				std::cerr << "error! correction is NAN" << std::endl;
				std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			}
			getRepulsiveObstacleForce();
			if(ISNAN(correction.x()) || ISNAN(correction.y()))
			{
				std::cerr << "error! correction is NAN" << std::endl;
				std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			}
			getAttractiveForce();
			if(ISNAN(correction.x()) || ISNAN(correction.y()))
			{
				std::cerr << "error! correction is NAN" << std::endl;
				std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			}

			if(sim_->IsMovingPlatform)
				getMovingPlatformForce();

			if(ISNAN(correction.x()) || ISNAN(correction.y()))
			{
				std::cerr << "error! correction is NAN" << std::endl;
				std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			}

			newVelocity_ += correction;

			if(ISNAN(newVelocity_.x()) || ISNAN(newVelocity_.y()))
			{
				std::cerr << "error! correction is NAN" << std::endl;
				std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			}
		}
		catch(const std::runtime_error& re)
		{
			// speciffic handling for runtime_error
			std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cerr << "Runtime error: " << re.what() << std::endl;
			PRINT_STACK_TRACE
			exit(EXIT_FAILURE);
		}
		catch(const std::exception& ex)
		{
			// speciffic handling for all exceptions extending std::exception, except
			// std::runtime_error which is handled explicitly
			std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			std::cerr << "Error occurred: " << ex.what() << std::endl;
			PRINT_STACK_TRACE
			exit(EXIT_FAILURE);
		}
		catch (...)
		{
			std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			PRINT_STACK_TRACE
			exit(EXIT_FAILURE);
		}
	}

	/// <summary> Inserts an agent neighbor into the set of neighbors of this agent </summary>
	/// <param name="agent"> A pointer to the agent to be inserted </param>
	/// <param name="rangeSq"> The squared range around this agent </param>
	void Agent::insertAgentNeighbor(const Agent* agent, float& rangeSq)
	{
		if (this != agent) 
		{
			const float distSq = absSq(position_ - agent->position_);

			if (distSq < rangeSq) 
			{
				if (agentNeighbors_.size() < maxNeighbors_) 
					agentNeighbors_.push_back(std::make_pair(distSq,agent));
				
				size_t i = agentNeighbors_.size() - 1;
			
				while (i != 0 && distSq < agentNeighbors_[i-1].first) 
				{
					agentNeighbors_[i] = agentNeighbors_[i-1];
					--i;
				}
				
				agentNeighbors_[i] = std::make_pair(distSq, agent);

				if (agentNeighbors_.size() == maxNeighbors_) 
					rangeSq = agentNeighbors_.back().first;
			}
		}
	}

	/// <summary> Inserts a static obstacle neighbor into the set of neighbors of this agent </summary>
	/// <param name="agent"> A pointer to the obstacle to be inserted </param>
	/// <param name="rangeSq"> The squared range around this agent </param>
	void Agent::insertObstacleNeighbor(const Obstacle* obstacle, float rangeSq)
	{
		const Obstacle* const nextObstacle = obstacle->nextObstacle;

		const float distSq = distSqPointLineSegment(obstacle->point_, nextObstacle->point_, position_);

		if (distSq < rangeSq) 
		{
			obstacleNeighbors_.push_back(std::make_pair(distSq,obstacle));
      
			size_t i = obstacleNeighbors_.size() - 1;
			while (i != 0 && distSq < obstacleNeighbors_[i-1].first) 
			{
				obstacleNeighbors_[i] = obstacleNeighbors_[i-1];
				--i;
			}
			
			obstacleNeighbors_[i] = std::make_pair(distSq, obstacle);
		}
	}

	/// <summary> Inserts an neighbor agent identifier into the set of neighbors of this agent </summary>
	/// <param name="agent"> A pointer to the agent ID to be inserted </param>
	/// <param name="rangeSq"> The squared range around this agent </param>
	void Agent::insertAgentNeighborsIndex(const Agent* agent, const float& rangeSq)
	{
		if (this != agent) 
		{
			const float distSq = absSq(position_ - agent->position_);

			if (distSq < rangeSq) 
			{
				agentNeighborsIndexList_.push_back(std::make_pair(agent->id_, distSq));
				size_t i = agentNeighborsIndexList_.size() - 1;
        
				while (i != 0 && distSq < agentNeighborsIndexList_[i-1].second) 
				{
					agentNeighborsIndexList_[i] = agentNeighborsIndexList_[i - 1];
					--i;
				}

				agentNeighborsIndexList_[i] = std::make_pair(agent->id_, distSq);
			}
		}
	}

	/// <summary> Gets point on line nearest to selected position  </summary>
	/// <param name="start"> Position of start of line </param>
	/// <param name="end"> Position of end of line </param>
	/// <param name="point"> Selected point </param>
	/// <returns> The nearest point </returns>
	Vector2 Agent::getNearestPoint(Vector2 *start, Vector2 *end, Vector2 *point) const
	{
		Vector2 relativeEndPoint = *end - *start;
		Vector2 relativePoint = *point - *start;
		double lambda = relativePoint * relativeEndPoint / relativeEndPoint.GetLengthSquared();

		if(lambda <= 0)
			return *start;
		else if(lambda >= 1)
			return *end;
		else
			return *start + static_cast<float>(lambda) * relativeEndPoint;
	}

	/// <summary> Returns the smaller of two float numbers </summary>
	/// <param name="end"> First number </param>
	/// <param name="point"> Second number </param>
	/// <returns> Smaller number </returns>
	inline float getMinFloat(float a, float b)
	{
		return a < b ? a : b;
	}

	/// <summary> Used for acceleration term method calling </summary>
	void Agent::update()
	{
		// TODO: you can improve some logic
		getAccelerationTerm();
	}

	/// <summary> Matrix cross for moving platform </summary>
	/// <param name="left"> Left matrix </param>
	/// <param name="right"> Right matrix </param>
	/// <returns> Result matrix </returns>
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

	/// <summary> Degree-to-radian conversion </summary>
	/// <param name="degree"> Degree value </param>
	/// <returns> Radian value </returns>
	double Agent::degreesToRadians(float degree) const
	{
        return degree * (M_PI / 180.0f);
    }

	/// <summary> Radian-to-degree conversion </summary>
	/// <param name="radian"> Radian value </param>
	/// <returns> Degree value </returns>
	double Agent::radiansToDegrees(float radian) const
	{
        return radian * (180.0f / M_PI);
    }

	/// <summary> Gets current roll </summary>
	/// <param name="pt"> Rotation projection type </param>
	/// <param name="tt"> Rotation time type </param>
	/// <returns> Roll </returns>
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

	/// <summary> Gets Omega matrix </summary>
	/// <param name="pt"> Rotation projection type </param>
	/// <param name="tt"> Rotation time type </param>
	/// <returns> Omega matrix </returns>
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

	/// <summary> Gets DOmega matrix </summary>
	/// <param name="pt"> Rotation projection type </param>
	/// <param name="tt"> Rotation time type </param>
	/// <returns> DOmega matrix </returns>
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

	/// <summary> Has intersection computing method </summary>
	/// <param name="a"> Start of first line </param>
	/// <param name="b"> End of first line </param>
	/// <param name="c"> Start of second line </param>
	/// <param name="d"> End of second line </param>
	/// <returns> True if two lines has intersection, false elsewhere </returns>
	bool Agent::isIntersect(Vector2 a, Vector2 b, Vector2 c, Vector2 d) const
	{
		float v1 = (d.x() - c.x())*(a.y() - c.y()) - (d.y() - c.y())*(a.x() - c.x());
		float v2 = (d.x() - c.x())*(b.y() - c.y()) - (d.y() - c.y())*(b.x() - c.x());
		float v3 = (b.x() - a.x())*(c.y() - a.y()) - (b.y() - a.y())*(c.x() - a.x());
		float v4 = (b.x() - a.x())*(d.y() - a.y()) - (b.y() - a.y())*(d.x() - a.x());
		
		return (v1 * v2 < TOLERANCE) && (v3 * v4 < 0);
	}

	/// <summary> Gets intersection point </summary>
	/// <param name="a"> Start of first line </param>
	/// <param name="b"> End of first line </param>
	/// <param name="c"> Start of second line </param>
	/// <param name="d"> End of second line </param>
	/// <returns> Intersection point </returns>
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

		float x = ((x3*y4 - x4*y3)*(x2 - x1) - (x1*y2 - x2*y1)*(x4 - x3)) / ((y1 - y2)*(x4 - x3) - (y3 - y4)*(x2 - x1));
		float y = ((y3 - y4)*x - (x3*y4 - x4*y3)) / (x4 - x3);

		return Vector2(x, y);
	}

	/// <summary> Gets rotation X matrix </summary>
	/// <param name="angle"> Rotation angle </param>
	/// <returns> Rotation matrix </returns>
	SimpleMatrix Agent::getRotationX(float angle) const
	{
		SimpleMatrix result = SimpleMatrix();	// identity

		float c = cos(angle);
		float s = sin(angle);

		result.m22 = c;
		result.m23 = s;
		result.m32 = -s;
		result.m33 = c;

		return result;
	}

	/// <summary> Gets rotation Y matrix </summary>
	/// <param name="angle"> Rotation angle </param>
	/// <returns> Rotation matrix </returns>
	SimpleMatrix Agent::getRotationY(float angle) const
	{
		SimpleMatrix result = SimpleMatrix();	// identity

		float c = cos(angle);
		float s = sin(angle);

		result.m11 = c;
		result.m13 = -s;
		result.m31 = s;
		result.m33 = c;

		return result;
	}

	/// <summary> Gets rotation Z matrix </summary>
	/// <param name="angle"> Rotation angle </param>
	/// <returns> Rotation matrix </returns>
	SimpleMatrix Agent::getRotationZ(float angle) const
	{
		SimpleMatrix result = SimpleMatrix();	// identity

		float c = cos(angle);
		float s = sin(angle);

		result.m11 = c;
		result.m12 = s;
		result.m21 = -s;
		result.m22 = c;

		return result;
	}

	/// <summary> Serialize agent </summary>
	/// <returns> Serialized agent array </returns>
	unsigned char* Agent::Serialize() const
	{
		size_t bufferSize = sizeof(size_t) + sizeof(*this);
		// The buffer we will be writing bytes into
		unsigned char* outBuf = new unsigned char[bufferSize];

		// A pointer we will advance whenever we write data
		unsigned char* p = outBuf;

		memcpy(p, &bufferSize, sizeof(size_t));
		p += sizeof(size_t);

		memcpy(p, &this->isDeleted_, sizeof(bool));
		p += sizeof(bool);

		//bool isForced_;
		memcpy(p, &this->isForced_, sizeof(bool));
		p += sizeof(bool);

		//size_t id_;
		memcpy(p, &this->id_, sizeof(size_t));
		p += sizeof(size_t);

		//size_t maxNeighbors_;
		memcpy(p, &this->maxNeighbors_, sizeof(size_t));
		p += sizeof(size_t);

		//float acceleration_;
		memcpy(p, &this->acceleration_, sizeof(float));
		p += sizeof(float);

		//float relaxationTime_;
		memcpy(p, &this->relaxationTime_, sizeof(float));
		p += sizeof(float);

		//float maxSpeed_;
		memcpy(p, &this->maxSpeed_, sizeof(float));
		p += sizeof(float);

		//float force_;
		memcpy(p, &this->force_, sizeof(float));
		p += sizeof(float);

		//float neighborDist_;
		memcpy(p, &this->neighborDist_, sizeof(float));
		p += sizeof(float);

		//float radius_;
		memcpy(p, &this->radius_, sizeof(float));
		p += sizeof(float);

		//float timeHorizonObst_;
		memcpy(p, &this->timeHorizonObst_, sizeof(float));
		p += sizeof(float);

		//float accelerationCoefficient_;
		memcpy(p, &this->accelerationCoefficient_, sizeof(float));
		p += sizeof(float);

		//float repulsiveAgent_;
		memcpy(p, &this->repulsiveAgent_, sizeof(float));
		p += sizeof(float);

		//float repulsiveAgentFactor_;
		memcpy(p, &this->repulsiveAgentFactor_, sizeof(float));
		p += sizeof(float);

		//float repulsiveObstacle_;
		memcpy(p, &this->repulsiveObstacle_, sizeof(float));
		p += sizeof(float);

		//float repulsiveObstacleFactor_;
		memcpy(p, &this->repulsiveObstacleFactor_, sizeof(float));
		p += sizeof(float);

		//float obstacleRadius_;
		memcpy(p, &this->obstacleRadius_, sizeof(float));
		p += sizeof(float);

		//float platformFactor_;
		memcpy(p, &this->platformFactor_, sizeof(float));
		p += sizeof(float);

		//float perception_;
		memcpy(p, &this->perception_, sizeof(float));
		p += sizeof(float);

		//float friction_;
		memcpy(p, &this->friction_, sizeof(float));
		p += sizeof(float);

		//double obstaclePressure_;
		memcpy(p, &this->obstaclePressure_, sizeof(double));
		p += sizeof(double);

		//double agentPressure_;
		memcpy(p, &this->agentPressure_, sizeof(double));
		p += sizeof(double);


		//Vector2 correction;
		memcpy(p, &this->correction, sizeof(Vector2));
		p += sizeof(Vector2);

		//Vector2 newVelocity_;
		memcpy(p, &this->newVelocity_, sizeof(Vector2));
		p += sizeof(Vector2);

		//Vector2 position_;
		memcpy(p, &this->position_, sizeof(Vector2));
		p += sizeof(Vector2);

		//Vector2 prefVelocity_;
		memcpy(p, &this->prefVelocity_, sizeof(Vector2));
		p += sizeof(Vector2);

		//Vector2 previosPosition_;
		memcpy(p, &this->previosPosition_, sizeof(Vector2));
		p += sizeof(Vector2);

		//Vector2 velocity_;
		memcpy(p, &this->velocity_, sizeof(Vector2));
		p += sizeof(Vector2);

		//Vector2 obstacleForce_;
		memcpy(p, &this->obstacleForce_, sizeof(Vector2));
		p += sizeof(Vector2);

		//Vector2 agentForce_;
		memcpy(p, &this->agentForce_, sizeof(Vector2));
		p += sizeof(Vector2);

		//Vector3 oldPlatformVelocity_;
		memcpy(p, &this->oldPlatformVelocity_, sizeof(Vector3));
		p += sizeof(Vector3);
		
		return outBuf;
		//SFSimulator* sim_;
	}

	/// <summary> Deserialize agent from array </summary>
	/// <param name="angle"> array that contains serialized agent </param>
	/// <returns> deserialized agent </returns>
	Agent* Agent::Deseriaize(unsigned char * arr)
	{
		Agent* ag= new Agent(NULL);
		// A pointer we will advance whenever we write data
		unsigned char* p = arr;

		size_t buffSize = 0;
		memcpy(&buffSize, p, sizeof(size_t));
		p += sizeof(size_t);

		memcpy(&ag->isDeleted_, p, sizeof(bool));
		p += sizeof(bool);

		//bool isForced_;
		memcpy(&ag->isForced_, p, sizeof(bool));
		p += sizeof(bool);

		//size_t id_;
		memcpy(&ag->id_, p, sizeof(size_t));
		p += sizeof(size_t);

		size_t maxNeighbors_;
		memcpy(&ag->maxNeighbors_, p, sizeof(size_t));
		p += sizeof(size_t);

		//float acceleration_;
		memcpy(&ag->acceleration_, p, sizeof(float));
		p += sizeof(float);

		//float relaxationTime_;
		memcpy(&ag->relaxationTime_, p, sizeof(float));
		p += sizeof(float);

		//float maxSpeed_;
		memcpy(&ag->maxSpeed_, p, sizeof(float));
		p += sizeof(float);

		//float force_;
		memcpy(&ag->force_, p, sizeof(float));
		p += sizeof(float);

		//float neighborDist_;
		memcpy(&ag->neighborDist_, p, sizeof(float));
		p += sizeof(float);

		//float radius_;
		memcpy(&ag->radius_, p, sizeof(float));
		p += sizeof(float);

		//float timeHorizonObst_;
		memcpy(&ag->timeHorizonObst_, p, sizeof(float));
		p += sizeof(float);

		//float accelerationCoefficient_;
		memcpy(&ag->accelerationCoefficient_, p, sizeof(float));
		p += sizeof(float);

		//float repulsiveAgent_;
		memcpy(&ag->repulsiveAgent_, p, sizeof(float));
		p += sizeof(float);

		//float repulsiveAgentFactor_;
		memcpy(&ag->repulsiveAgentFactor_, p, sizeof(float));
		p += sizeof(float);

		//float repulsiveObstacle_;
		memcpy(&ag->repulsiveObstacle_, p, sizeof(float));
		p += sizeof(float);

		//float repulsiveObstacleFactor_;
		memcpy(&ag->repulsiveObstacleFactor_, p, sizeof(float));
		p += sizeof(float);

		//float obstacleRadius_;
		memcpy(&ag->obstacleRadius_, p, sizeof(float));
		p += sizeof(float);

		//float platformFactor_;
		memcpy(&ag->platformFactor_, p, sizeof(float));
		p += sizeof(float);

		//float perception_;
		memcpy(&ag->perception_, p, sizeof(float));
		p += sizeof(float);

		//float friction_;
		memcpy(&ag->friction_, p, sizeof(float));
		p += sizeof(float);

		//double obstaclePressure_;
		memcpy(&ag->obstaclePressure_, p, sizeof(double));
		p += sizeof(double);

		//double agentPressure_;
		memcpy(&ag->agentPressure_, p, sizeof(double));
		p += sizeof(double);


		//Vector2 correction;
		memcpy(&ag->correction, p, sizeof(Vector2));
		p += sizeof(Vector2);

		//Vector2 newVelocity_;
		memcpy(&ag->newVelocity_, p, sizeof(Vector2));
		p += sizeof(Vector2);

		//Vector2 position_;
		memcpy(&ag->position_, p, sizeof(Vector2));
		p += sizeof(Vector2);

		//Vector2 prefVelocity_;
		memcpy(&ag->prefVelocity_, p, sizeof(Vector2));
		p += sizeof(Vector2);

		//Vector2 previosPosition_;
		memcpy(&ag->previosPosition_, p, sizeof(Vector2));
		p += sizeof(Vector2);

		//Vector2 velocity_;
		memcpy(&ag->velocity_, p, sizeof(Vector2));
		p += sizeof(Vector2);

		//Vector2 obstacleForce_;
		memcpy(&ag->obstacleForce_, p, sizeof(Vector2));
		p += sizeof(Vector2);

		//Vector2 agentForce_;
		memcpy(&ag->agentForce_, p, sizeof(Vector2));
		p += sizeof(Vector2);

		//Vector3 oldPlatformVelocity_;
		memcpy(&ag->oldPlatformVelocity_, p, sizeof(Vector3));
		p += sizeof(Vector3);

		return ag;
	}
}
