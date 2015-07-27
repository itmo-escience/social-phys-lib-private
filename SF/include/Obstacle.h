/*
 *  Obstacle.h
 *  SF Library
 */

/*!
 *  @file       Obstacle.h
 *  @brief      Contains the Obstacle class.
 */

#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "Definitions.h"

namespace SF
{
  /*!
   *  @brief      Defines static obstacles in the simulation.
   */
  class Obstacle
  {
  private:
    /*!
     *  @brief      Constructs a static obstacle instance.
     */
    Obstacle();

    /*!
     *  @brief      Destroys this static obstacle instance.
     */
    ~Obstacle();

    bool isConvex_;
    Obstacle* nextObstacle;
    Vector2 point_;
    Obstacle* prevObstacle;
    Vector2 unitDir_;

    size_t id_;

    friend class Agent;
    friend class KdTree;
    friend class SFSimulator;
  };
}

#endif

