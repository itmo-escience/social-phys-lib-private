/*
 *  Agent.h
 *  SF Library.
 */

/*!
 *  @file       Agent.h
 *  @brief      Contains the Agent class.
 */

#ifndef AGENT_H
#define AGENT_H

#include <map>
#include "Definitions.h"
#include "SFSimulator.h"
#include "Vector3.h"
#include "SimpleMatrix.h"

namespace SF
{
	/*
	*	@brief		Defines simple acceleration pair
	*/
	class ForceAcceleration
	{
	public:
		ForceAcceleration(float a, int d);
		int getDirection();
		int setDirectionForZ(float z);
		void update(float a, int d);
		float getMultCoefficient();
		
		float acceleration;	
		int direction;
	};


  /*!
   *  @brief      Defines an agent in the simulation.
   */
  class Agent
  {
  private:
    /*!
     *  @brief      Constructs an agent instance.
     *  @param      sim             The simulator instance.
     */
    explicit Agent(SFSimulator* sim);

    /*!
     *  @brief      Destroys this agent instance.
     */
    ~Agent();

    /*!
     *  @brief      Computes the neighbors of this agent.
     */
    void computeNeighbors();

    /*!
     *  @brief      Computes the new velocity of this agent.
     */
    void computeNewVelocity();

    /*!
     *  @brief      Inserts an agent neighbor into the set of neighbors of
     *              this agent.
     *  @param      agent           A pointer to the agent to be inserted.
     *  @param      rangeSq         The squared range around this agent.
     */
    void insertAgentNeighbor(const Agent* agent, float& rangeSq);

	void insertAgentNeighborsIndex(const Agent* agent, float& rangeSq);


    /*!
     *  @brief      Inserts a static obstacle neighbor into the set of neighbors
     *              of this agent.
     *  @param      obstacle        The number of the static obstacle to be
     *                              inserted.
     *  @param      rangeSq         The squared range around this agent.
     */
    void insertObstacleNeighbor(const Obstacle* obstacle, float rangeSq);

    /*!
     *  @brief      Updates the two-dimensional position and two-dimensional
     *              velocity of this agent.
     */
    void update();

	/*
	*   @brief		Updates speed list containing speed values corresponding 
	*				each agent 
	*	@param		index		Id of agent
	*	@param		value		Value of agent speed
	*/
	void setSpeedList(int index, float value);

	/*
	*	@brief		Sets the null values of speed
	*	@param		id			Isd of agent
	*/
	void setNullSpeed(int id);

	/*
	*	@brief		Finds perception of some point by agent
	*	@param		arg1		Position of agent
	*	@param		arg2		Position of percepted point
	*/
	float getPerception(Vector2 *arg1, Vector2 *arg2);

	 /*!
     *  @brief      Normalizing the velocity
     *  @param      currentSpeed	Current speed
     *  @param      maxSpeed		Max speed
     */
	float getNormalizedSpeed(float currentSpeed, float maxSpeed);
		
	/*
	*	@brief		Gets point on line nearest to selected position 
	*	@param		start		Position of start of line
	*	@param		end			Position of end of line
	*	@param		point		Selected point
	*/
	Vector2 getNearestPoint(Vector2 *start, Vector2 *end, Vector2 *point);

	/*
	*	@brief		Returns correction coefficient for big angles
	*	@param		a		cos of first angle
	*	@param		b		cos of second angle
	*/
	float getCriticalAnglesCorrection(float a, float b);

	/*
	*	@brief		Checks if case is simple
	*	@param		s		Vector
	*/
	bool isNotPlaneCase(Vector3 s);

	/*
	*	@brief		Returns cos of incline angle
	*	@param		s		Vector
	*/
	float getInclineAngle(Vector3 s);

	/*
	*	@brief		Returns cos of rotation angle
	*	@param		s		Vector
	*/
	float getRotationAngle(Vector3 s);

	/*
	*	@brief		Returns projection for XOY plane
	*	@param		s		Vector
	*/
	Vector2 getVectorProjectionXY(Vector3 s);

	/*
	*	@brief		Returns projection for XOZ plane
	*	@param		s		Vector
	*/
	Vector2 getVectorProjectionXZ(Vector3 s);

	/*
	*	@brief		Returns projection for YOZ plane
	*	@param		s		Vector
	*/

	    
	typedef enum
	{
		X = 1,
		Y
	}
	ParameterType;

	typedef enum
	{
		PAST = 1,
		PAST2NOW,
		NOW, 
		NOW2FUTURE,
		FUTURE
	}
	TimeType;

	Vector2 getVectorProjectionYZ(Vector3 s);
	Vector3 getCross(Vector3 left, Vector3 right);
	float degreesToRadians(float degree);
	float radiansToDegrees(float degree);
	Vector3 getRoll(ParameterType pt, TimeType tt);
	Vector3 getOmega(ParameterType pt, TimeType tt);
	Vector3 getDOmega(ParameterType pt, TimeType tt);
	SimpleMatrix getRotationX(float angle);
	SimpleMatrix getRotationY(float angle);
	SimpleMatrix getRotationZ(float angle);


	bool isForced_;
	size_t id_;
	size_t maxNeighbors_;
	int direction_;
	float acceleration_;
    float accelerationBuffer_;
    float relaxationTime_;
	float maxSpeed_;
    float neighborDist_;
    float radius_;
    float timeHorizonObst_;
    float accelerationCoefficient_;
    float repulsiveAgent_;
    float repulsiveAgentFactor_;
    float repulsiveObstacle_;
	float repulsiveObstacleFactor_;
	float platformFactor_;
    float perception_;
	float friction_;
	ForceAcceleration forceAcceleration_;
    Vector2 newVelocity_;
    Vector2 position_;
    Vector2 prefVelocity_;
	Vector2 previosPosition_;
    Vector2 velocity_;
	Vector3 oldPlatformVelocity_;
	std::vector<std::pair<float, const Obstacle*> > obstacleNeighbors_;
    std::vector<std::pair<float, const Agent*> > agentNeighbors_;
	std::vector<std::pair<size_t, float>> agentNeighborsIndexList_;
    std::map<int, float> speedList_;
	SFSimulator* sim_;
    
    friend class KdTree;
    friend class SFSimulator;
  };
}

#endif
