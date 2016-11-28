#include <algorithm>

#include "../include/SFSimulator.h"
#include "../include/KdTree.h"
#include "../include/Agent.h"
#include "../include/Obstacle.h"

namespace SF
{
	/// <summary> Constructs a kd-tree instance </summary>
	/// <param name="sim"> The simulator instance </param>
	KdTree::KdTree(SFSimulator* sim) : 
		agents_(), 
		agentTree_(), 
		obstacleTree_(nullptr), 
		sim_(sim)
	{  }

	/// <summary> Destructor </summary>
	KdTree::~KdTree()
	{
		deleteObstacleTree(obstacleTree_);
	}

	/// <summary> Builds an agent kd-tree </summary>
	void KdTree::buildAgentTree()
	{
		//if (agents_.size() < sim_->agents_.size()) 
		//{
			agents_.clear();
			for (auto i = 0; i < sim_->agents_.size(); ++i) 
				if (!sim_->agents_[i]->isDeleted_)
					agents_.push_back(sim_->agents_[i]);

			for(auto i = 0; i < sim_->tmpAgents_.size(); i++)
				if(!sim_->tmpAgents_[i]->isDeleted_)
					agents_.push_back(sim_->tmpAgents_[i]);
			
			agentTree_.resize(2 * agents_.size() - 1);
		//}
    
		if (!agents_.empty()) 
			buildAgentTreeRecursive(0, agents_.size(), 0);
	}

	/// <summary> Builds an agent kd-tree </summary>
	/// <param name="begin"> Begin node  </param>
	/// <param name="end"> End node  </param>
	/// <param name="node"> Selected node  </param>
	void KdTree::buildAgentTreeRecursive(size_t begin, size_t end, size_t node)
	{
		agentTree_[node].begin = begin;
		agentTree_[node].end = end;
		agentTree_[node].minX = agentTree_[node].maxX = agents_[begin]->position_.x();
		agentTree_[node].minY = agentTree_[node].maxY = agents_[begin]->position_.y();
    
		for (auto i = begin + 1; i < end; ++i) 
		{
			auto tree = agentTree_[node];
			auto position = agents_[i]->position_;

			tree.maxX = std::max(tree.maxX, position.x());
			tree.minX = std::min(tree.minX, position.x());
			tree.maxY = std::max(tree.maxY, position.y());
			tree.minY = std::min(tree.minY, position.y());

			agentTree_[node] = tree;
		}

		if (end - begin > MAX_LEAF_SIZE) 
		{
			// No leaf node
			const auto isVertical = (agentTree_[node].maxX - agentTree_[node].minX > agentTree_[node].maxY - agentTree_[node].minY);
			const auto splitValue = (isVertical ? 0.5f * (agentTree_[node].maxX + agentTree_[node].minX) : 0.5f * (agentTree_[node].maxY + agentTree_[node].minY));

			auto left = begin;
			auto right = end;

			while (left < right) 
			{
				while (left < right && (isVertical ? agents_[left]->position_.x() : agents_[left]->position_.y()) < splitValue) 
					++left;
				
				while (right > left && (isVertical ? agents_[right-1]->position_.x() : agents_[right-1]->position_.y()) >= splitValue)
					--right;
				
				if (left < right) 
				{
					std::swap(agents_[left], agents_[right-1]);
					++left;
					--right;
				}
			}

			auto leftSize = left - begin;

			if (leftSize == 0) {
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
			return nullptr;

		const auto node = new ObstacleTreeNode;

		size_t optimalSplit = 0;
		auto minLeft = obstacles.size();
		auto minRight = minLeft;

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

				const auto j1LeftOfI = leftOf(obstacleI1->point_, obstacleI2->point_, obstacleJ1->point_);
				const auto j2LeftOfI = leftOf(obstacleI1->point_, obstacleI2->point_, obstacleJ2->point_);

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

			const auto obstacleJ1 = obstacles[j];
			const auto obstacleJ2 = obstacleJ1->nextObstacle;

			const auto j1LeftOfI = leftOf(obstacleI1->point_, obstacleI2->point_, obstacleJ1->point_);
			const auto j2LeftOfI = leftOf(obstacleI1->point_, obstacleI2->point_, obstacleJ2->point_);

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
		//mtx.lock();
		queryObstacleTreeRecursive(agent, rangeSq, obstacleTree_);
		//mtx.unlock();
	}

	/// <summary> Deletes the specified obstacle tree node </summary>
	/// <param name="agent"> A pointer to the obstacle tree node to be deleted </param>
	void KdTree::deleteObstacleTree(ObstacleTreeNode* node) const
	{
		if (node != nullptr) 
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
		if (agentTree_[node].end - agentTree_[node].begin <= MAX_LEAF_SIZE) 
		{
			for (auto i = agentTree_[node].begin; i < agentTree_[node].end; ++i) 
			{
				if(!(agents_[i]->isDeleted_))
				agent->insertAgentNeighbor(agents_[i], rangeSq);
			}
		} 
		else 
		{
			const auto distSqLeft = sqr(std::max(0.0f, agentTree_[agentTree_[node].left].minX - agent->position_.x())) + sqr(std::max(0.0f, agent->position_.x() - agentTree_[agentTree_[node].left].maxX)) + sqr(std::max(0.0f, agentTree_[agentTree_[node].left].minY - agent->position_.y())) + sqr(std::max(0.0f, agent->position_.y() - agentTree_[agentTree_[node].left].maxY));

			const auto distSqRight = sqr(std::max(0.0f, agentTree_[agentTree_[node].right].minX - agent->position_.x())) + sqr(std::max(0.0f, agent->position_.x() - agentTree_[agentTree_[node].right].maxX)) + sqr(std::max(0.0f, agentTree_[agentTree_[node].right].minY - agent->position_.y())) + sqr(std::max(0.0f, agent->position_.y() - agentTree_[agentTree_[node].right].maxY));

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

	/// <summary> Inserts the specified agent ID tree node </summary>
	/// <param name="agent"> A pointer to the agent for which agent ID neighbors are to be inserted </param>
	/// <param name="rangeSq"> The squared range around the agent </param>
	/// <param name="node"> The specified node </param>
	void KdTree::queryAgentNeighborsIndexListTreeRecursive(Agent* agent, float& rangeSq, size_t node) const
	{
		if (agentTree_[node].end - agentTree_[node].begin <= MAX_LEAF_SIZE) 
		{
			for (auto i = agentTree_[node].begin; i < agentTree_[node].end; ++i) 
			{
				agent->insertAgentNeighborsIndex(agents_[i], rangeSq);
			}
		} 
		else 
		{
			const auto distSqLeft = sqr(std::max(0.0f, agentTree_[agentTree_[node].left].minX - agent->position_.x())) + sqr(std::max(0.0f, agent->position_.x() - agentTree_[agentTree_[node].left].maxX)) + sqr(std::max(0.0f, agentTree_[agentTree_[node].left].minY - agent->position_.y())) + sqr(std::max(0.0f, agent->position_.y() - agentTree_[agentTree_[node].left].maxY));

			const auto distSqRight = sqr(std::max(0.0f, agentTree_[agentTree_[node].right].minX - agent->position_.x())) + sqr(std::max(0.0f, agent->position_.x() - agentTree_[agentTree_[node].right].maxX)) + sqr(std::max(0.0f, agentTree_[agentTree_[node].right].minY - agent->position_.y())) + sqr(std::max(0.0f, agent->position_.y() - agentTree_[agentTree_[node].right].maxY));

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
		if (node == nullptr)
			return;
		
		const auto obstacle1 = node->obstacle;
		const auto obstacle2 = obstacle1->nextObstacle;
		const auto agentLeftOfLine = leftOf(obstacle1->point_, obstacle2->point_, agent->position_);

		queryObstacleTreeRecursive(agent, rangeSq, (agentLeftOfLine >= 0.0f ? node->left : node->right));
			
		const auto distSqLine = sqr(agentLeftOfLine) / absSq(obstacle2->point_ - obstacle1->point_);

		if (distSqLine < rangeSq) 
		{
			agent->insertObstacleNeighbor(node->obstacle, rangeSq);
			queryObstacleTreeRecursive(agent, rangeSq, (agentLeftOfLine >= 0.0f ? node->right : node->left));
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
		auto visibility = queryVisibilityRecursive(q1, q2, radius, obstacleTree_);
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
		if (node == nullptr) 
			return true;
		else 
		{
			const auto obstacle1 = node->obstacle;
			const auto obstacle2 = obstacle1->nextObstacle;

			const auto q1LeftOfI = leftOf(obstacle1->point_, obstacle2->point_, q1);
			const auto q2LeftOfI = leftOf(obstacle1->point_, obstacle2->point_, q2);
			const auto invLengthI = 1.0f / absSq(obstacle2->point_ - obstacle1->point_);

			if (q1LeftOfI >= 0.0f && q2LeftOfI >= 0.0f)
				return queryVisibilityRecursive(q1, q2, radius, node->left) && ((sqr(q1LeftOfI) * invLengthI >= sqr(radius) && sqr(q2LeftOfI) * invLengthI >= sqr(radius)) || queryVisibilityRecursive(q1, q2, radius, node->right));
			else if (q1LeftOfI <= 0.0f && q2LeftOfI <= 0.0f) 
				return queryVisibilityRecursive(q1, q2, radius, node->right) && ((sqr(q1LeftOfI) * invLengthI >= sqr(radius) && sqr(q2LeftOfI) * invLengthI >= sqr(radius)) || queryVisibilityRecursive(q1, q2, radius, node->left));
			else if (q1LeftOfI >= 0.0f && q2LeftOfI <= 0.0f)
				// One can see through obstacle from left to right
				return queryVisibilityRecursive(q1, q2, radius, node->left) && queryVisibilityRecursive(q1, q2, radius, node->right);
			else 
			{
				const auto point1LeftOfQ = leftOf(q1, q2, obstacle1->point_);
				const auto point2LeftOfQ = leftOf(q1, q2, obstacle2->point_);
				const auto invLengthQ = 1.0f / absSq(q2 - q1);

				return (point1LeftOfQ * point2LeftOfQ >= 0.0f && sqr(point1LeftOfQ) * invLengthQ > sqr(radius) && sqr(point2LeftOfQ) * invLengthQ > sqr(radius) && queryVisibilityRecursive(q1, q2, radius, node->left) && queryVisibilityRecursive(q1, q2, radius, node->right));
			}
		}
	}
}
