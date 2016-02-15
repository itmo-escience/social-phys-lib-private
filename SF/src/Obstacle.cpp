#include "../include/SFSimulator.h"
#include "../include/Obstacle.h"

namespace SF
{
	/// <summary> Constructs a static obstacle instance </summary>
	Obstacle::Obstacle() : 
		isConvex_(false), 
		nextObstacle(nullptr), 
		point_(), 
		prevObstacle(nullptr), 
		unitDir_(), 
		id_(0)
	{ }

	/// <summary> Destroys this static obstacle instance </summary>
	Obstacle::~Obstacle()
	{ }
}
