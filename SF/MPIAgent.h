#pragma once
#include "include/Agent.h"
namespace SF
{
	class MPIAgent
	{

	public:
		Agent* agent;
		MPIAgent();
		MPIAgent(Agent* agent);
		~MPIAgent();

		unsigned char* SerializeAgent() const;
		void DeserializeAgent(unsigned char*);
		void PrintAgentInfo() const;
	};

}