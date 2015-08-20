/*
 *  SFSimulator.h
 *  SF Library.
 */

/*!
 *  @file       SFSimulator.h
 *  @brief      Contains the SFSimulator class.
 */

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
  /*!
   *  @brief       Error value.
   *
   *  A value equal to the largest unsigned integer (or -1 for many compilers)
   *  that is returned in case of an error by functions in SF::SFSimulator.
   */
  static const size_t SF_ERROR = std::numeric_limits<size_t>::max();
  
  /*!
   *  @brief      Defines a directed line.
   */
  struct Line {
    /*!
     *  @brief     A point on the directed line.
     */
    Vector2 point;
        
    /*!
     *  @brief     The direction of the directed line.
     */
    Vector2 direction;
  };

  class Agent;
  class KdTree;
  class Obstacle;
  class AgentPropertyConfig;
  class RotationDegreeSet;

  /*!
   *  @brief      Defines the simulation.
   *
   * The main class of the library that contains all simulation functionality.
   */
  class SFSimulator
  {
  public:
    /*!
     *  @brief      Constructs a simulator instance.
     */
    SFSimulator();

    /*!
     *  @brief      Destroys this simulator instance.
     */
    ~SFSimulator();

    /*!
     *  @brief      Adds a new agent with default properties to the
     *              simulation.
     *  @param      position        The two-dimensional starting position of
     *                              this agent.
     *  @returns    The number of the agent, or SF::SF_ERROR when the agent
     *              defaults have not been set.
     */
    size_t addAgent(const Vector2& position);

    /*!
     *  @brief      Adds a new agent to the simulation.
     *  @param      position        The two-dimensional starting position of
     *                              this agent.
     *  @param      neighborDist    The maximal distance (center point to
     *                              center point) to other agents this agent
     *                              takes into account in the navigation. The
     *                              larger this number, the longer the running
     *                              time of the simulation. If the number is too
     *                              low, the simulation will not be safe.
     *                              Must be non-negative.
     *  @param      maxNeighbors    The maximal number of other agents this
     *                              agent takes into account in the navigation.
     *                              The larger this number, the longer the
     *                              running time of the simulation. If the
     *                              number is too low, the simulation will not
     *                              be safe.
     *  @param      timeHorizon     The minimal amount of time for which this
     *                              agent's velocities that are computed by the
     *                              simulation are safe with respect to other
     *                              agents. The larger this number, the sooner
     *                              this agent will respond to the presence of
     *                              other agents, but the less freedom this
     *                              agent has in choosing its velocities.
     *                              Must be positive.
     *  @param      timeHorizonObst The minimal amount of time for which this
     *                              agent's velocities that are computed by the
     *                              simulation are safe with respect to
     *                              obstacles. The larger this number, the
     *                              sooner this agent will respond to the
     *                              presence of obstacles, but the less freedom
     *                              this agent has in choosing its velocities.
     *                              Must be positive.
     *  @param      radius          The radius of this agent.
     *                              Must be non-negative.
     *  @param      maxSpeed        The maximal speed of this agent.
     *                              Must be non-negative.
     *  @param      velocity        The initial two-dimensional linear velocity
     *                              of this agent (optional).
     *  @returns    The number of the agent.
     */
    size_t addAgent(
		const Vector2& position, 
		float neighborDist,
		size_t maxNeighbors, 
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
		float platformFactor,
		float perception,
		float friction,
		const Vector2& velocity = Vector2()
	);

    /*!
     *  @brief      Adds a new obstacle to the simulation.
     *  @param      vertices        List of the vertices of the polygonal
     *              obstacle in counterclockwise order.
     *  @returns    The number of the first vertex of the obstacle,
     *              or SF::SF_ERROR when the number of vertices is less than two.
     *  @note       To add a "negative" obstacle, e.g. a bounding polygon around 
     *              the environment, the vertices should be listed in clockwise 
     *              order. 
     */
    size_t addObstacle(const std::vector<Vector2>& vertices);

    /*!
     *  @brief      Lets the simulator perform a simulation step and updates the
     *              two-dimensional position and two-dimensional velocity of
     *              each agent.
     */
    void doStep();

    /*!
     *  @brief      Returns the specified agent neighbor of the specified
     *              agent.
     *  @param      agentNo         The number of the agent whose agent
     *                              neighbor is to be retrieved.
     *  @param      neighborNo      The number of the agent neighbor to be
     *                              retrieved.
     *  @returns    The number of the neighboring agent.
     */
    size_t getAgentAgentNeighbor(size_t agentNo, size_t neighborNo) const;
    
    /*!
     *  @brief      Returns the maximum neighbor count of a specified agent.
     *  @param      agentNo         The number of the agent whose maximum
     *                              neighbor count is to be retrieved.
     *  @returns    The present maximum neighbor count of the agent.
     */
    size_t getAgentMaxNeighbors(size_t agentNo) const;

    /*!
     *  @brief      Returns the maximum speed of a specified agent.
     *  @param      agentNo         The number of the agent whose maximum speed
     *                              is to be retrieved.
     *  @returns    The present maximum speed of the agent.
     */
    float getAgentMaxSpeed(size_t agentNo) const;

    /*!
     *  @brief      Returns the maximum neighbor distance of a specified
     *              agent.
     *  @param      agentNo         The number of the agent whose maximum
     *                              neighbor distance is to be retrieved.
     *  @returns    The present maximum neighbor distance of the agent.
     */
    float getAgentNeighborDist(size_t agentNo) const;

    /*!
     *  @brief      Returns the count of agent neighbors taken into account to 
     *              compute the current velocity for the specified agent.
     *  @param      agentNo         The number of the agent whose count of agent
     *                              neighbors is to be retrieved.
     *  @returns    The count of agent neighbors taken into account to compute 
     *              the current velocity for the specified agent.
     */
    size_t getAgentNumAgentNeighbors(size_t agentNo) const;

    /*!
     *  @brief      Returns the count of obstacle neighbors taken into account
     *              to compute the current velocity for the specified agent.
     *  @param      agentNo         The number of the agent whose count of
     *                              obstacle neighbors is to be retrieved.
     *  @returns    The count of obstacle neighbors taken into account to 
     *              compute the current velocity for the specified agent.
     */
    size_t getAgentNumObstacleNeighbors(size_t agentNo) const;

    
    /*!
     *  @brief      Returns the count of ORCA constraints used to compute
     *              the current velocity for the specified agent.
     *  @param      agentNo         The number of the agent whose count of ORCA
     *                              constraints is to be retrieved.
     *  @returns    The count of ORCA constraints used to compute the current
     *              velocity for the specified agent.
     */
    size_t getAgentNumORCALines(size_t agentNo) const;

    /*!
     *  @brief      Returns the specified obstacle neighbor of the specified
     *              agent.
     *  @param      agentNo         The number of the agent whose obstacle
     *                              neighbor is to be retrieved.
     *  @param      neighborNo      The number of the obstacle neighbor to be
     *                              retrieved.
     *  @returns    The number of the first vertex of the neighboring obstacle 
     *              edge.
     */
    size_t getAgentObstacleNeighbor(size_t agentNo, size_t neighborNo) const;

    /*!
     *  @brief      Returns the specified ORCA constraint of the specified
     *              agent.
     *  @param      agentNo         The number of the agent whose ORCA
     *                              constraint is to be retrieved.
     *  @param      lineNo          The number of the ORCA constraint to be
     *                              retrieved.
     *  @returns    A line representing the specified ORCA constraint.
     *  @note       The halfplane to the left of the line is the region of 
     *              permissible velocities with respect to the specified 
     *              ORCA constraint.
     */
    const Line& getAgentORCALine(size_t agentNo, size_t lineNo) const;

    /*!
     *  @brief      Returns the two-dimensional position of a specified
     *              agent.
     *  @param      agentNo         The number of the agent whose
     *                              two-dimensional position is to be retrieved.
     *  @returns    The present two-dimensional position of the (center of the)
     *              agent.
     */
    const Vector2& getAgentPosition(size_t agentNo) const;

    /*!
     *  @brief      Returns the two-dimensional preferred velocity of a
     *              specified agent.
     *  @param      agentNo         The number of the agent whose
     *                              two-dimensional preferred velocity is to be
     *                              retrieved.
     *  @returns    The present two-dimensional preferred velocity of the agent.
     */
    const Vector2& getAgentPrefVelocity(size_t agentNo) const;

    /*!
     *  @brief      Returns the radius of a specified agent.
     *  @param      agentNo         The number of the agent whose radius is to
     *                              be retrieved.
     *  @returns    The present radius of the agent.
     */
    float getAgentRadius(size_t agentNo) const;

    /*!
     *  @brief      Returns the time horizon with respect to obstacles of a
     *              specified agent.
     *  @param      agentNo         The number of the agent whose time horizon
     *                              with respect to obstacles is to be
     *                              retrieved.
     *  @returns    The present time horizon with respect to obstacles of the
     *              agent.
     */
    float getAgentTimeHorizonObst(size_t agentNo) const;

    /*!
     *  @brief      Returns the two-dimensional linear velocity of a 
     *              specified agent.
     *  @param      agentNo         The number of the agent whose
     *                              two-dimensional linear velocity is to be
     *                              retrieved.
     *  @returns    The present two-dimensional linear velocity of the agent.
     */
    const Vector2& getAgentVelocity(size_t agentNo) const;

    /*!
     *  @brief      Returns the global time of the simulation.
     *  @returns    The present global time of the simulation (zero initially).
     */
    float getGlobalTime() const;

    /*!
     *  @brief      Returns the count of agents in the simulation.
     *  @returns    The count of agents in the simulation.
     */
    size_t getNumAgents() const;

    /*!
     *  @brief      Returns the count of obstacle vertices in the simulation.
     *  @returns    The count of obstacle vertices in the simulation.
     */
    size_t getNumObstacleVertices() const;

    /*!
     *  @brief      Returns the two-dimensional position of a specified obstacle
     *              vertex.
     *  @param      vertexNo        The number of the obstacle vertex to be
     *                              retrieved.
     *  @returns    The two-dimensional position of the specified obstacle
     *              vertex.
     */
    const Vector2& getObstacleVertex(size_t vertexNo) const;

    /*!
     *  @brief      Returns the number of the obstacle vertex succeeding the
     *              specified obstacle vertex in its polygon.
     *  @param      vertexNo        The number of the obstacle vertex whose
     *                              successor is to be retrieved.
     *  @returns    The number of the obstacle vertex succeeding the specified
     *              obstacle vertex in its polygon.
     */
    size_t getNextObstacleVertexNo(size_t vertexNo) const;

    /*!
     *  @brief      Returns the number of the obstacle vertex preceding the
     *              specified obstacle vertex in its polygon.
     *  @param      vertexNo        The number of the obstacle vertex whose
     *                              predecessor is to be retrieved.
     *  @returns    The number of the obstacle vertex preceding the specified
     *              obstacle vertex in its polygon.
     */
    size_t getPrevObstacleVertexNo(size_t vertexNo) const;

    /*!
     *  @brief      Returns the time step of the simulation.
     *  @returns    The present time step of the simulation.
     */
    float getTimeStep() const;

    /*!
     *  @brief      Processes the obstacles that have been added so that they
     *              are accounted for in the simulation.
     *  @note       Obstacles added to the simulation after this function has 
     *              been called are not accounted for in the simulation.
     */
    void processObstacles();

    /*!
     *  @brief      Performs a visibility query between the two specified
     *              points with respect to the obstacles
     *  @param      point1          The first point of the query.
     *  @param      point2          The second point of the query.
     *  @param      radius          The minimal distance between the line
     *                              connecting the two points and the obstacles
     *                              in order for the points to be mutually
     *                              visible (optional). Must be non-negative.
     *  @returns    A boolean specifying whether the two points are mutually
     *              visible. Returns true when the obstacles have not been
     *              processed.
     */
    bool queryVisibility(
		const Vector2& point1, 
		const Vector2& point2, 
		float radius = 0.0f
	) const;

	/*
	*	@brief		Sets default property of agent
	*	@param		apc				Property
	*/
	void setAgentDefaults(AgentPropertyConfig & apc);

    /*!
     *  @brief      Sets the maximum neighbor count of a specified agent.
     *  @param      agentNo         The number of the agent whose maximum
     *                              neighbor count is to be modified.
     *  @param      maxNeighbors    The replacement maximum neighbor count.
     */
    void setAgentMaxNeighbors(size_t agentNo, size_t maxNeighbors);

    /*!
     *  @brief      Sets the maximum speed of a specified agent.
     *  @param      agentNo         The number of the agent whose maximum speed
     *                              is to be modified.
     *  @param      maxSpeed        The replacement maximum speed. Must be 
     *                              non-negative.
     */
    void setAgentMaxSpeed(size_t agentNo, float maxSpeed);

    /*!
     *  @brief      Sets the maximum neighbor distance of a specified agent.
     *  @param      agentNo         The number of the agent whose maximum
     *                              neighbor distance is to be modified.
     *  @param      neighborDist    The replacement maximum neighbor distance.
     *                              Must be non-negative.
     */
    void setAgentNeighborDist(size_t agentNo, float neighborDist);

    /*!
     *  @brief      Sets the two-dimensional position of a specified agent.
     *  @param      agentNo         The number of the agent whose
     *                              two-dimensional position is to be modified.
     *  @param      position        The replacement of the two-dimensional
     *                              position.
     */
    void setAgentPosition(size_t agentNo, const Vector2& position);

    /*!
     *  @brief      Sets the two-dimensional preferred velocity of a
     *              specified agent.
     *  @param      agentNo         The number of the agent whose
     *                              two-dimensional preferred velocity is to be
     *                              modified.
     *  @param      prefVelocity    The replacement of the two-dimensional
     *                              preferred velocity.
     */
    void setAgentPrefVelocity(size_t agentNo, const Vector2& prefVelocity);

    /*!
     *  @brief      Sets the radius of a specified agent.
     *  @param      agentNo         The number of the agent whose radius is to
     *                              be modified.
     *  @param      radius          The replacement radius. 
     *                              Must be non-negative.
     */
    void setAgentRadius(size_t agentNo, float radius);

    /*!
     *  @brief      Sets the time horizon of a specified agent with respect
     *              to obstacles.
     *  @param      agentNo         The number of the agent whose time horizon
     *                              with respect to obstacles is to be modified.
     *  @param      timeHorizonObst The replacement time horizon with respect to
     *                              obstacles. Must be positive.
     */
    void setAgentTimeHorizonObst(size_t agentNo, float timeHorizonObst);

    /*!
     *  @brief      Sets the two-dimensional linear velocity of a specified
     *              agent.
     *  @param      agentNo         The number of the agent whose
     *                              two-dimensional linear velocity is to be
     *                              modified.
     *  @param      velocity        The replacement two-dimensional linear
     *                              velocity.
     */
    void setAgentVelocity(size_t agentNo, const Vector2& velocity);

    /*!
     *  @brief      Sets the time step of the simulation.
     *  @param      timeStep        The time step of the simulation.
     *                              Must be positive.
     */
    void setTimeStep(float timeStep);

	/*
	*	@brief		Sets the additional force
	*	@param		v			New value of velocity
	*	@param		friction	Value of friction
	*/
	void setAdditionalForce(Vector3 velocity, RotationDegreeSet set);

	 
	/*!
	*	@brief		Sets the velocity of platform
	*	@param		v	New value of velocity, default value is 1.0
	*/
	void setPlatformVelocity(Vector3 velocity);

	/*!
	*	@brief		Returns the velocity of platform
	*/
	Vector3 getPlatformVelocity();

	/*!
	*	@brief		Returns the friction of platform
	*/
	float getAgentFriction(size_t agentNo) const;

	/*!
	*	@brief		Sets the friction of platform
	*	@param		friction	New value of friction
	*/
	void setAgentFriction(size_t agentNo, float friction);

	/*!
	*	@brief		Sets the angle set
	*	@param		set		Angle set
	*/
	void setRotationDegreeSet(RotationDegreeSet set);

	/*!
	*	@brief		Returns the angle set
	*/
	RotationDegreeSet getRotationDegreeSet();

	/*!
	*	@brief		Some useful methods
	*/
	void addPlatformRotationXY(float value);
	void addPlatformRotationXZ(float value);
	void addPlatformRotationYZ(float value);

	float getPlatformRotationXY();
	float getPlatformRotationXZ();
	float getPlatformRotationYZ();

	std::vector<size_t> getAgentNeighboursIndexList(size_t index, float radius);

	Vector3 rotationPast_;
	Vector3 rotationPast2Now_;
	Vector3 rotationNow_;
	Vector3 rotationNow2Future_;
	Vector3 rotationFuture_;


  private:
    std::vector<Agent*> agents_;
    Agent* defaultAgent_;
    float globalTime_;
    KdTree* kdTree_;
    std::vector<Obstacle*> obstacles_;
    float timeStep_;
	Vector3 platformVelocity_;
	RotationDegreeSet angleSet_;
	float platformRotationXY_;
	float platformRotationXZ_;
	float platformRotationYZ_;
	

    friend class Agent;
    friend class KdTree;
    friend class Obstacle;
  };
}

#endif
