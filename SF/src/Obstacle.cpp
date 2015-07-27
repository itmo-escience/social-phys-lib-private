/*
 *  Obstacle.cpp
 *  SF Library.
 */

#include "../include/SFSimulator.h"

#include "../include/Obstacle.h"

namespace SF
{
  Obstacle::Obstacle() : isConvex_(false), nextObstacle(0), point_(), prevObstacle(0), unitDir_(), id_(0)
  {
  }

  Obstacle::~Obstacle()
  {
  }
}
