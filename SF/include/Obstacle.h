#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "Definitions.h"

namespace SF
{

	/// <summary> Defines static obstacles in the simulation </summary>
	class Obstacle
	{
	private:
		/// <summary> Constructs a static obstacle instance </summary>
		Obstacle();

		/// <summary> Destroys this static obstacle instance </summary>
		~Obstacle();

		bool isConvex_;			// mark convex
		Obstacle* nextObstacle;	// next obstacle
		Vector2 point_;			// position
		Obstacle* prevObstacle;	// previous obstacle
		Vector2 unitDir_;		// direction
		size_t id_;				// ID

		friend class Agent;
		friend class KdTree;
		friend class SFSimulator;
	};
}

#endif

