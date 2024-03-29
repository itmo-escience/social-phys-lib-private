﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using SF2D;

namespace ConsoleApplication1
{
    class Program
    {
        static void Main(string[] args)
        {

            /* Create a new simulator instance. */
            SFSimulator sim = new SFSimulator();

            /* Set up the scenario. */
            setupScenario(sim);

            /* Perform (and manipulate) the simulation. */
            do
            {
                updateVisualization(sim);
                setPreferredVelocities(sim);
                sim.doStep();
            } while (!reachedGoal(sim));
        }

            /* Store the goals of the agents. */
            static List<SFVector2> goals = new  List<SFVector2>();

            static void setupScenario(SFSimulator sim)
            {
                /* Specify the global time step of the simulation. */
                sim.setTimeStep(0.25f);
                
                /* Specify the default parameters for agents that are subsequently added. */
                AgentProperty ap = new AgentProperty(15.0f, 10, 5.0f, 1.2f, 2.0f, 2.0f, 0.5f, 8, 0.6f, 100, 13.3f, 10, 0.000005f, 0.25f, 1.0f, new SFVector2());
                sim.setAgentDefaults(ap);

                /*
               * Add agents, specifying their start position, and store their goals on the
               * opposite side of the environment.
               */
               for (int i = 0; i < 5; ++i) {
                for (int j = 0; j < 5; ++j) {
                  sim.addAgent(new SFVector2(55.0f + i * 10.0f,  55.0f + j * 10.0f));
                  goals.Add(new SFVector2(-75.0f, -75.0f));
                    
                  sim.addAgent(new SFVector2(-55.0f - i * 10.0f,  55.0f + j * 10.0f));
                  goals.Add(new SFVector2(75.0f, -75.0f));

                  sim.addAgent(new SFVector2(55.0f + i * 10.0f, -55.0f - j * 10.0f));
                  goals.Add(new SFVector2(-75.0f, 75.0f));

                  sim.addAgent(new SFVector2(-55.0f - i * 10.0f, -55.0f - j * 10.0f));
                  goals.Add(new SFVector2(75.0f, 75.0f));
                }
              }

              /*
               * Add (polygonal) obstacles, specifying their vertices in counterclockwise
               * order.
               */
                List<SFVector2> obstacle1 = new List<SFVector2>();
                List<SFVector2>  obstacle2= new List<SFVector2>();
                List<SFVector2>  obstacle3= new List<SFVector2>();
                List<SFVector2>  obstacle4= new List<SFVector2>();

              obstacle1.Add(new SFVector2(-10.0f, 40.0f));
              obstacle1.Add(new SFVector2(-40.0f, 40.0f));
              obstacle1.Add(new SFVector2(-40.0f, 10.0f));
              obstacle1.Add(new SFVector2(-10.0f, 10.0f));

              obstacle2.Add(new SFVector2(10.0f, 40.0f));
              obstacle2.Add(new SFVector2(10.0f, 10.0f));
              obstacle2.Add(new SFVector2(40.0f, 10.0f));
              obstacle2.Add(new SFVector2(40.0f, 40.0f));

              obstacle3.Add(new SFVector2(10.0f, -40.0f));
              obstacle3.Add(new SFVector2(40.0f, -40.0f));
              obstacle3.Add(new SFVector2(40.0f, -10.0f));
              obstacle3.Add(new SFVector2(10.0f, -10.0f));

              obstacle4.Add(new SFVector2(-10.0f, -40.0f));
              obstacle4.Add(new SFVector2(-10.0f, -10.0f));
              obstacle4.Add(new SFVector2(-40.0f, -10.0f));
              obstacle4.Add(new SFVector2(-40.0f, -40.0f));

              sim.addObstacle(obstacle1);
              sim.addObstacle(obstacle2);
              sim.addObstacle(obstacle3);
              sim.addObstacle(obstacle4);

              /* Process the obstacles so that they are accounted for in the simulation. */
              sim.processObstacles();
            }


            static void updateVisualization(SFSimulator sim)
            {              
              /* Output the current position of all the agents. */
              for (int i = 0; i < sim.getNumAgents(); ++i) {
                Console.WriteLine(sim.getAgentPosition(i) );
              }

              Console.WriteLine("-------");
            }

            static Random r = new Random();
            static void setPreferredVelocities(SFSimulator sim)
            {
              /* 
               * Set the preferred velocity to be a vector of unit magnitude (speed) in the
               * direction of the goal.
               */
            #pragma omp parallel for
              for (int i = 0; i < sim.getNumAgents(); ++i) {
                Vector2 goalVector = new Vector2(goals[i].X - sim.getAgentPosition(i).X, goals[i].Y - sim.getAgentPosition(i).Y); //goals[i] - sim.getAgentPosition(i);

                if ((goalVector.LengthSquared()) > 1.0f) {
                  goalVector = Vector2.Normalize(goalVector);
                }

                sim.setAgentPrefVelocity(i, new SFVector2(goalVector.X, goalVector.Y));
    
                /*
                 * Perturb a little to avoid deadlocks due to perfect symmetry.
                 */

                float angle = (float)r.NextDouble() * 2.0f * (float)Math.PI ;
                float dist = (float)r.NextDouble() * 0.0001f ;

                  var distVector = new SFVector2((float) Math.Cos(angle)*dist, (float) Math.Sin(angle)*dist);

                sim.setAgentPrefVelocity(i, new SFVector2(sim.getAgentPrefVelocity(i).X + distVector.X, sim.getAgentPrefVelocity(i).Y + distVector.Y));
              }
            }

            static bool reachedGoal(SFSimulator sim)
            {
              /* Check if all agents have reached their goals. */
              for (int i = 0; i < sim.getNumAgents(); ++i) {
                if ((new Vector2(sim.getAgentPosition(i).X - goals[i].X, sim.getAgentPosition(i).Y - goals[i].Y).LengthSquared() > 20.0f * 20.0f)) {
                  return false;
                }
              }

              return true;
            }



    }
}
