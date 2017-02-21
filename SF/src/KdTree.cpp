#include <algorithm>

#include "../include/SFSimulator.h"
#include "../include/KdTree.h"
#include "../include/Agent.h"
#include "../include/Obstacle.h"
#include <iostream>

namespace SF
{
	/// <summary> Constructs a kd-tree instance </summary>
	/// <param name="sim"> The simulator instance </param>
	KdTree::KdTree(SFSimulator* sim) : 
		agents_(), 
		agentTree_(), 
		obstacleTree_(NULL), 
		sim_(sim)
	{  }

	/// <summary> Destructor </summary>
	KdTree::~KdTree()
	{
		deleteObstacleTree(obstacleTree_);
	}

	size_t KdTree::Size()
	{		
		size_t size = sizeof(sim_);

		for(std::vector<Agent*>::const_iterator it = agents_.begin(); it != agents_.end(); ++it)
		{
			size += sizeof(it);
		}

		for(std::vector<AgentTreeNode>::const_iterator it = agentTree_.begin(); it != agentTree_.end(); ++it)
		{
			size += sizeof(it);
		}

		size += obstacleTree_->Size();

		return size;
	}

	/// <summary> Builds an agent kd-tree </summary>
	void KdTree::buildAgentTree()
	{
		try
		{
			agents_.clear();
			agentTree_.clear();
			//printf("agents count %d \n", sim_->agents_.size());

			for(std::map<size_t, Agent*>::iterator it = sim_->agents_.container.begin(); it != sim_->agents_.container.end(); ++it)
			{
				if (!it->second->isDeleted_)
						agents_.push_back(it->second);
			}

			//for (size_t i = 0; i < sim_->agents_.size(); ++i)
			//	if (sim_->agents_[i] != NULL)
			//	{
			//		if (!sim_->agents_[i]->isDeleted_)
			//			agents_.push_back(sim_->agents_[i]);
			//	}

			size_t id = sim_->agents_.maximalID + 1; //create new ID for tmp agents and add them to agents list
			for (size_t i = 0; i < sim_->tmpAgents_.size(); i++)
			{
				sim_->tmpAgents_[i]->id_ = id;
				id++;
				agents_.push_back(sim_->tmpAgents_[i]);
			}

			//for (size_t i = 0; i < sim_->tmpAgents_.size(); i++)
			////if(!sim_->tmpAgents_[i]->isDeleted_)
			//	agents_.push_back(sim_->tmpAgents_[i]);

			if (!agents_.empty())
			{
				agentTree_.resize(2 * agents_.size() - 1);
				buildAgentTreeRecursive(0, agents_.size(), 0);
			}
		}
		catch (std::bad_alloc& ba) 
		{
			std::cerr << ba.what() <<  " Memory overflow at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
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

	/// <summary> Builds an agent kd-tree </summary>
	/// <param name="begin"> Begin node  </param>
	/// <param name="end"> End node  </param>
	/// <param name="node"> Selected node  </param>
	void KdTree::buildAgentTreeRecursive(size_t begin, size_t end, size_t node)
	{
		//std::cerr<<" begin: " << begin << " end: " << end << " node: "  << node << std::endl; 
		try
		{
			//printf("begin: %d end: %d node: %d \n", begin, end, node);
			agentTree_[node].begin = begin;
			agentTree_[node].end = end;
			agentTree_[node].minX = agentTree_[node].maxX = agents_[begin]->position_.x();
			agentTree_[node].minY = agentTree_[node].maxY = agents_[begin]->position_.y();

			for (size_t i = begin + 1; i < end; ++i)
			{
				AgentTreeNode tree = agentTree_[node];
				Vector2 position = agents_[i]->position_;

				tree.maxX = std::max(tree.maxX, position.x());
				tree.minX = std::min(tree.minX, position.x());
				tree.maxY = std::max(tree.maxY, position.y());
				tree.minY = std::min(tree.minY, position.y());

				agentTree_[node] = tree;
			}

			if (end - begin > MAX_LEAF_SIZE)
			{
				// No leaf node
				const bool isVertical = (agentTree_[node].maxX - agentTree_[node].minX > agentTree_[node].maxY - agentTree_[node].minY);
				const float splitValue = (isVertical ? 0.5f * (agentTree_[node].maxX + agentTree_[node].minX) : 0.5f * (agentTree_[node].maxY + agentTree_[node].minY));

				size_t left = begin;
				size_t right = end;

				while (left < right)
				{
					while (left < right && (isVertical ? agents_[left]->position_.x() : agents_[left]->position_.y()) < splitValue)
						++left;

					while (right > left && (isVertical ? agents_[right - 1]->position_.x() : agents_[right - 1]->position_.y()) >= splitValue)
						--right;

					if (left < right)
					{
						std::swap(agents_[left], agents_[right - 1]);
						++left;
						--right;
					}
				}

				size_t leftSize = left - begin;

				if (leftSize == 0)
				{
					++leftSize;
					++left;
					++right;
				}

				agentTree_[node].left = node + 1;
				agentTree_[node].right = node + 1 + (2 * leftSize - 1);

				buildAgentTreeRecursive(begin, left, agentTree_[node].left);
				buildAgentTreeRecursive(left, end, agentTree_[node].right);
			}
		}
		catch (...)
		{
			std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			PRINT_STACK_TRACE
			exit(EXIT_FAILURE);
		}
	}

	/// <summary> Builds an obstacle kd-tree </summary>
	void KdTree::buildObstacleTree()
	{
		//mtx.lock();
		deleteObstacleTree(obstacleTree_);

		std::vector<Obstacle*> obstacles(sim_->obstacles_.size());

		for (size_t i = 0; i < sim_->obstacles_.size(); ++i)
			obstacles[i] = sim_->obstacles_[i];

		obstacleTree_ = buildObstacleTreeRecursive(obstacles);

		//mtx.unlock();
	}

	/// <summary> Builds an obstacle kd-tree </summary>
	/// <param name="obstacles"> Obstacles set  </param>
	/// <returns> The tree </returns>
	KdTree::ObstacleTreeNode* KdTree::buildObstacleTreeRecursive(const std::vector<Obstacle*>& obstacles)
	{
		if (obstacles.empty())
			return NULL;

		ObstacleTreeNode* node = new ObstacleTreeNode;

		size_t optimalSplit = 0;
		size_t minLeft = obstacles.size();
		size_t minRight = minLeft;

		for (size_t i = 0; i < obstacles.size(); ++i) 
		{
			size_t leftSize = 0;
			size_t rightSize = 0;

			const Obstacle* const obstacleI1 = obstacles[i];
			const Obstacle* const obstacleI2 = obstacleI1->nextObstacle;

			for (size_t j = 0; j < obstacles.size(); ++j) 
			{
				if (i == j)
					continue;
					
				const Obstacle* const obstacleJ1 = obstacles[j];
				const Obstacle* const obstacleJ2 = obstacleJ1->nextObstacle;

				const float j1LeftOfI = leftOf(obstacleI1->point_, obstacleI2->point_, obstacleJ1->point_);
				const float j2LeftOfI = leftOf(obstacleI1->point_, obstacleI2->point_, obstacleJ2->point_);

				if (j1LeftOfI >= -SF_EPSILON && j2LeftOfI >= -SF_EPSILON) 
					++leftSize;
				else if (j1LeftOfI <= SF_EPSILON && j2LeftOfI <= SF_EPSILON) 
					++rightSize;
				else 
				{
					++leftSize;
					++rightSize;
				}

				if (std::make_pair(std::max(leftSize, rightSize), std::min(leftSize, rightSize)) >= std::make_pair(std::max(minLeft, minRight), std::min(minLeft, minRight))) 
					break;
			}

			if (std::make_pair(std::max(leftSize, rightSize), std::min(leftSize, rightSize)) < std::make_pair(std::max(minLeft, minRight), std::min(minLeft, minRight))) 
			{
				minLeft = leftSize;
				minRight = rightSize;
				optimalSplit = i;
			}
		}

		std::vector<Obstacle*> leftObstacles(minLeft);
		std::vector<Obstacle*> rightObstacles(minRight);

		size_t leftCounter = 0;
		size_t rightCounter = 0;

		const size_t i = optimalSplit;
		const Obstacle* const obstacleI1 = obstacles[i];
		const Obstacle* const obstacleI2 = obstacleI1->nextObstacle;

		for (size_t j = 0; j < obstacles.size(); ++j) 
		{
			if (i == j)
				continue;

			Obstacle* obstacleJ1 = obstacles[j];
			Obstacle* obstacleJ2 = obstacleJ1->nextObstacle;

			const float j1LeftOfI = leftOf(obstacleI1->point_, obstacleI2->point_, obstacleJ1->point_);
			const float j2LeftOfI = leftOf(obstacleI1->point_, obstacleI2->point_, obstacleJ2->point_);

			if (j1LeftOfI >= -SF_EPSILON && j2LeftOfI >= -SF_EPSILON)
				leftObstacles[leftCounter++] = obstacleJ1;
			else if (j1LeftOfI <= SF_EPSILON && j2LeftOfI <= SF_EPSILON) 
				rightObstacles[rightCounter++] = obstacleJ1;
			else 
			{
				if(j1LeftOfI + j2LeftOfI > 0)
				{
					leftObstacles[leftCounter++] = obstacleJ1;
					rightObstacles[rightCounter++] = obstacleJ2;
				}
				else
				{
					leftObstacles[leftCounter++] = obstacleJ2;
					rightObstacles[rightCounter++] = obstacleJ1;
				}
			}
		}

		node->obstacle = obstacleI1;
		node->left = buildObstacleTreeRecursive(leftObstacles);
		node->right = buildObstacleTreeRecursive(rightObstacles);

		return node;
	}

	/// <summary> Computes the agent neighbors of the specified agent </summary>
	/// <param name="agent"> A pointer to the agent for which agent neighbors are to be computed </param>
	/// <param name="rangeSq"> The squared range around the agent </param>
	void KdTree::computeAgentNeighbors(Agent* agent, float& rangeSq) const
	{
		queryAgentTreeRecursive(agent, rangeSq, 0);
	}

	/// <summary> Computes the agent ID neighbors of the specified agent </summary>
	/// <param name="agent"> A pointer to the agent for which agent ID neighbors are to be computed </param>
	/// <param name="rangeSq"> The squared range around the agent </param>
	void KdTree::computeAgentNeighborsIndexList(Agent* agent, float& rangeSq) const
	{
		queryAgentNeighborsIndexListTreeRecursive(agent, rangeSq, 0);
	}

	/// <summary> Computes the obstacle neighbors of the specified agent </summary>
	/// <param name="agent"> A pointer to the obstacle for which agent neighbors are to be computed </param>
	/// <param name="rangeSq"> The squared range around the agent </param>
	void KdTree::computeObstacleNeighbors(Agent* agent, float rangeSq) const
	{
		try
		{
			//mtx.lock();
			queryObstacleTreeRecursive(agent, rangeSq, obstacleTree_);
			//mtx.unlock();
		}
		catch (...)
		{
			std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			PRINT_STACK_TRACE
			exit(EXIT_FAILURE);
		}
	}

	/// <summary> Deletes the specified obstacle tree node </summary>
	/// <param name="agent"> A pointer to the obstacle tree node to be deleted </param>
	void KdTree::deleteObstacleTree(ObstacleTreeNode* node) const
	{
		if (node != NULL) 
		{
			deleteObstacleTree(node->left);
			deleteObstacleTree(node->right);
			delete node;
		}
	}

	/// <summary> Inserts the specified agent tree node </summary>
	/// <param name="agent"> A pointer to the agent for which agent neighbors are to be inserted </param>
	/// <param name="rangeSq"> The squared range around the agent </param>
	/// <param name="node"> The specified node </param>
	void KdTree::queryAgentTreeRecursive(Agent* agent, float& rangeSq, size_t node) const
	{
		try
		{
			if (agentTree_[node].end - agentTree_[node].begin <= MAX_LEAF_SIZE)
			{
				for (size_t i = agentTree_[node].begin; i < agentTree_[node].end; ++i)
				{
					if (!(agents_[i]->isDeleted_))
						agent->insertAgentNeighbor(agents_[i], rangeSq);
				}
			}
			else
			{
				const float distSqLeft = sqr(std::max(0.0f, agentTree_[agentTree_[node].left].minX - agent->position_.x())) + sqr(std::max(0.0f, agent->position_.x() - agentTree_[agentTree_[node].left].maxX)) + sqr(std::max(0.0f, agentTree_[agentTree_[node].left].minY - agent->position_.y())) + sqr(std::max(0.0f, agent->position_.y() - agentTree_[agentTree_[node].left].maxY));

				const float distSqRight = sqr(std::max(0.0f, agentTree_[agentTree_[node].right].minX - agent->position_.x())) + sqr(std::max(0.0f, agent->position_.x() - agentTree_[agentTree_[node].right].maxX)) + sqr(std::max(0.0f, agentTree_[agentTree_[node].right].minY - agent->position_.y())) + sqr(std::max(0.0f, agent->position_.y() - agentTree_[agentTree_[node].right].maxY));

				if (distSqLeft < distSqRight)
				{
					if (distSqLeft < rangeSq)
					{
						queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].left);

						if (distSqRight < rangeSq)
							queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].right);
					}
				}
				else
				{
					if (distSqRight < rangeSq)
					{
						queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].right);

						if (distSqLeft < rangeSq)
							queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].left);
					}
				}
			}
		}
		catch (...)
		{
			std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			PRINT_STACK_TRACE
			exit(EXIT_FAILURE);
		}
	}

	/// <summary> Inserts the specified agent ID tree node </summary>
	/// <param name="agent"> A pointer to the agent for which agent ID neighbors are to be inserted </param>
	/// <param name="rangeSq"> The squared range around the agent </param>
	/// <param name="node"> The specified node </param>
	void KdTree::queryAgentNeighborsIndexListTreeRecursive(Agent* agent, float& rangeSq, size_t node) const
	{
		if (agentTree_[node].end - agentTree_[node].begin <= MAX_LEAF_SIZE) 
		{
			for (size_t i = agentTree_[node].begin; i < agentTree_[node].end; ++i) 
			{
				agent->insertAgentNeighborsIndex(agents_[i], rangeSq);
			}
		} 
		else 
		{
			const float distSqLeft = sqr(std::max(0.0f, agentTree_[agentTree_[node].left].minX - agent->position_.x())) + sqr(std::max(0.0f, agent->position_.x() - agentTree_[agentTree_[node].left].maxX)) + sqr(std::max(0.0f, agentTree_[agentTree_[node].left].minY - agent->position_.y())) + sqr(std::max(0.0f, agent->position_.y() - agentTree_[agentTree_[node].left].maxY));

			const float distSqRight = sqr(std::max(0.0f, agentTree_[agentTree_[node].right].minX - agent->position_.x())) + sqr(std::max(0.0f, agent->position_.x() - agentTree_[agentTree_[node].right].maxX)) + sqr(std::max(0.0f, agentTree_[agentTree_[node].right].minY - agent->position_.y())) + sqr(std::max(0.0f, agent->position_.y() - agentTree_[agentTree_[node].right].maxY));

			if (distSqLeft < distSqRight) 
			{
				if (distSqLeft < rangeSq) 
				{
					queryAgentNeighborsIndexListTreeRecursive(agent, rangeSq, agentTree_[node].left);

					if (distSqRight < rangeSq) 
						queryAgentNeighborsIndexListTreeRecursive(agent, rangeSq, agentTree_[node].right);
				}
			} 
			else 
			{
				if (distSqRight < rangeSq) 
				{
					queryAgentNeighborsIndexListTreeRecursive(agent, rangeSq, agentTree_[node].right);

					if (distSqLeft < rangeSq) 
						queryAgentNeighborsIndexListTreeRecursive(agent, rangeSq, agentTree_[node].left);
				}
			}
		}
	}

	/// <summary> Inserts the specified obstacle tree node </summary>
	/// <param name="agent"> A pointer to the agent for which obstacle neighbors are to be computed </param>
	/// <param name="rangeSq"> The squared range around the agent </param>
	/// <param name="node"> The specified node </param>
	void KdTree::queryObstacleTreeRecursive(Agent* agent, float rangeSq, const ObstacleTreeNode* node) const
	{
		try
		{
			if (node == NULL)
				return;

			const Obstacle* obstacle1 = node->obstacle;
			const Obstacle* obstacle2 = obstacle1->nextObstacle;
			const float agentLeftOfLine = leftOf(obstacle1->point_, obstacle2->point_, agent->position_);

			queryObstacleTreeRecursive(agent, rangeSq, (agentLeftOfLine >= 0.0f ? node->left : node->right));

			const float distSqLine = sqr(agentLeftOfLine) / absSq(obstacle2->point_ - obstacle1->point_);

			if (distSqLine < rangeSq)
			{
				agent->insertObstacleNeighbor(node->obstacle, rangeSq);
				queryObstacleTreeRecursive(agent, rangeSq, (agentLeftOfLine >= 0.0f ? node->right : node->left));
			}
		}
		catch (...)
		{
			std::cerr << " Error occured at file " << __FILE__ << " function: " << __FUNCTION__ << " line: " << __LINE__ << std::endl;
			PRINT_STACK_TRACE
			exit(EXIT_FAILURE);
		}
	}

	/// <summary> Queries the visibility between two points within a specified radius </summary>
	/// <param name="q1"> The first point between which visibility is to be tested </param>
	/// <param name="q2"> The second point between which visibility is to be tested </param>
	/// <param name="radius"> The radius within which visibility is to be tested </param>
	/// <returns> True if q1 and q2 are mutually visible within the radius; false otherwise </returns>
	bool KdTree::queryVisibility(const Vector2& q1, const Vector2& q2, float radius) const
	{
		//mtx.lock();
		bool visibility = queryVisibilityRecursive(q1, q2, radius, obstacleTree_);
		//mtx.unlock();
		return visibility;
	}

	/// <summary> Queries the visibility between two points within a specified radius </summary>
	/// <param name="q1"> The first point between which visibility is to be tested </param>
	/// <param name="q2"> The second point between which visibility is to be tested </param>
	/// <param name="node"> The selected node </param>
	/// <param name="radius"> The radius within which visibility is to be tested </param>
	/// <returns> True if q1 and q2 are mutually visible within the radius; false otherwise </returns>
	bool KdTree::queryVisibilityRecursive(const Vector2& q1, const Vector2& q2, float radius, const ObstacleTreeNode* node) const
	{
		if (node == NULL) 
			return true;
		else 
		{
			const Obstacle* obstacle1 = node->obstacle;
			const Obstacle* obstacle2 = obstacle1->nextObstacle;

			const float q1LeftOfI = leftOf(obstacle1->point_, obstacle2->point_, q1);
			const float q2LeftOfI = leftOf(obstacle1->point_, obstacle2->point_, q2);
			const float invLengthI = 1.0f / absSq(obstacle2->point_ - obstacle1->point_);

			if (q1LeftOfI >= 0.0f && q2LeftOfI >= 0.0f)
				return queryVisibilityRecursive(q1, q2, radius, node->left) && ((sqr(q1LeftOfI) * invLengthI >= sqr(radius) && sqr(q2LeftOfI) * invLengthI >= sqr(radius)) || queryVisibilityRecursive(q1, q2, radius, node->right));
			else if (q1LeftOfI <= 0.0f && q2LeftOfI <= 0.0f) 
				return queryVisibilityRecursive(q1, q2, radius, node->right) && ((sqr(q1LeftOfI) * invLengthI >= sqr(radius) && sqr(q2LeftOfI) * invLengthI >= sqr(radius)) || queryVisibilityRecursive(q1, q2, radius, node->left));
			else if (q1LeftOfI >= 0.0f && q2LeftOfI <= 0.0f)
				// One can see through obstacle from left to right
				return queryVisibilityRecursive(q1, q2, radius, node->left) && queryVisibilityRecursive(q1, q2, radius, node->right);
			else 
			{
				const float point1LeftOfQ = leftOf(q1, q2, obstacle1->point_);
				const float point2LeftOfQ = leftOf(q1, q2, obstacle2->point_);
				const float invLengthQ = 1.0f / absSq(q2 - q1);

				return (point1LeftOfQ * point2LeftOfQ >= 0.0f && sqr(point1LeftOfQ) * invLengthQ > sqr(radius) && sqr(point2LeftOfQ) * invLengthQ > sqr(radius) && queryVisibilityRecursive(q1, q2, radius, node->left) && queryVisibilityRecursive(q1, q2, radius, node->right));
			}
		}
	}
}
