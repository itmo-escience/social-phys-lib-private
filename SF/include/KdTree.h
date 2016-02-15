#ifndef KD_TREE_H
#define KD_TREE_H

#include "Definitions.h"

namespace SF
{
	/// <summary> Defines kd-trees for agents and static obstacles in the simulation </summary>
	class KdTree
	{
	private:
		/// <summary> Defines an agent kd-tree node </summary>
		struct AgentTreeNode 
		{
			size_t begin;			// The beginning node number
			size_t end;				// The ending node number
			size_t left;			// The left node number
			float maxX;				// The maximum x-coordinate
			float maxY;				// The maximum y-coordinate
			float minX;				// The minimum x-coordinate
			float minY;				// The minimum y-coordinate
			size_t right;			// The right node number
		};

		/// <summary> Defines an obstacle in kd-tree node </summary>
		struct ObstacleTreeNode {
			ObstacleTreeNode* left;		// The left obstacle tree node
			const Obstacle* obstacle;	// The obstacle number
			ObstacleTreeNode* right;	// The right obstacle tree node
		};

		/// <summary> Constructs a kd-tree instance </summary>
		/// <param name="sim"> The simulator instance </param>
		explicit KdTree(SFSimulator* sim);

		/// <summary> Destructor </summary>
		~KdTree();

		/// <summary> Builds an agent kd-tree </summary>
		void buildAgentTree();

		/// <summary> Builds an agent kd-tree </summary>
		/// <param name="begin"> Begin node  </param>
		/// <param name="end"> End node  </param>
		/// <param name="node"> Selected node  </param>
		void buildAgentTreeRecursive(size_t begin, size_t end, size_t node);

		/// <summary> Builds an obstacle kd-tree </summary>
		void buildObstacleTree();

		/// <summary> Builds an obstacle kd-tree </summary>
		/// <param name="obstacles"> Obstacles set  </param>
		/// <returns> The tree </returns>
		ObstacleTreeNode* buildObstacleTreeRecursive(const std::vector<Obstacle*>& obstacles);

		/// <summary> Computes the agent neighbors of the specified agent </summary>
		/// <param name="agent"> A pointer to the agent for which agent neighbors are to be computed </param>
		/// <param name="rangeSq"> The squared range around the agent </param>
		void computeAgentNeighbors(Agent* agent, float& rangeSq) const;

		/// <summary> Computes the obstacle neighbors of the specified agent </summary>
		/// <param name="agent"> A pointer to the obstacle for which agent neighbors are to be computed </param>
		/// <param name="rangeSq"> The squared range around the agent </param>
		void computeObstacleNeighbors(Agent* agent, float rangeSq) const;

		/// <summary> Deletes the specified obstacle tree node </summary>
		/// <param name="agent"> A pointer to the obstacle tree node to be deleted </param>
		void deleteObstacleTree(ObstacleTreeNode* node) const;

		/// <summary> Inserts the specified agent tree node </summary>
		/// <param name="agent"> A pointer to the agent for which agent neighbors are to be inserted </param>
		/// <param name="rangeSq"> The squared range around the agent </param>
		/// <param name="node"> The specified node </param>
		void queryAgentTreeRecursive(Agent* agent, float& rangeSq, size_t node) const;

		/// <summary> Inserts the specified agent ID tree node </summary>
		/// <param name="agent"> A pointer to the agent for which agent ID neighbors are to be inserted </param>
		/// <param name="rangeSq"> The squared range around the agent </param>
		/// <param name="node"> The specified node </param>
		void queryAgentNeighborsIndexListTreeRecursive(Agent* agent, float& rangeSq, size_t node) const;
    
		/// <summary> Inserts the specified obstacle tree node </summary>
		/// <param name="agent"> A pointer to the agent for which obstacle neighbors are to be computed </param>
		/// <param name="rangeSq"> The squared range around the agent </param>
		/// <param name="node"> The specified node </param>
		void queryObstacleTreeRecursive(Agent* agent, float rangeSq, const ObstacleTreeNode* node) const;

		/// <summary> Queries the visibility between two points within a specified radius </summary>
		/// <param name="q1"> The first point between which visibility is to be tested </param>
		/// <param name="q2"> The second point between which visibility is to be tested </param>
		/// <param name="radius"> The radius within which visibility is to be tested </param>
		/// <returns> True if q1 and q2 are mutually visible within the radius; false otherwise </returns>
		bool queryVisibility(const Vector2& q1, const Vector2& q2, float radius) const;

		/// <summary> Queries the visibility between two points within a specified radius </summary>
		/// <param name="q1"> The first point between which visibility is to be tested </param>
		/// <param name="q2"> The second point between which visibility is to be tested </param>
		/// <param name="node"> The selected node </param>
		/// <param name="radius"> The radius within which visibility is to be tested </param>
		/// <returns> True if q1 and q2 are mutually visible within the radius; false otherwise </returns>
		bool queryVisibilityRecursive(const Vector2& q1, const Vector2& q2, float radius, const ObstacleTreeNode* node) const;

		/// <summary> Computes the agent ID neighbors of the specified agent </summary>
		/// <param name="agent"> A pointer to the agent for which agent ID neighbors are to be computed </param>
		/// <param name="rangeSq"> The squared range around the agent </param>
		void computeAgentNeighborsIndexList(Agent* agent, float& rangeSq) const;

		std::vector<Agent*> agents_;				// agent list
		std::vector<AgentTreeNode> agentTree_;		// agent tree list
		ObstacleTreeNode* obstacleTree_;			// trivial obstacle node
		SFSimulator* sim_;							// simulator instance

		static const size_t MAX_LEAF_SIZE = 10;

		friend class Agent;
		friend class SFSimulator;
	};
}

#endif
