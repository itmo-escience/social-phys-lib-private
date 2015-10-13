// This is the main DLL file.

#include <vector>

#include "../include/SF2DWrapper.h"

using namespace SF2D;

AgentProperty::AgentProperty() {};

AgentProperty::AgentProperty(
	float neighborDist, 
	int maxNeighbors, 
	float timeHorizon,
	float obsHorizon,
	float radius, 
	float maxSpeed, 
	float accelerationCoefficient, 
	float relaxationTime,
	float repulsiveAgent, 
	float repulsiveAgentFactor, 
	float repulsiveObstacle, 
	float repulsiveObstacleFactor,
	float obstacleRadius,
	float platformFactor,
	float perception, 
	float friction,
	Microsoft::Xna::Framework::Vector2 velocity
  ): 
	NeighborDist(neighborDist),
	MaxNeighbors(maxNeighbors),
	TimeHorizon(timeHorizon),
	ObsHorizon(obsHorizon),
	Radius(radius),
	MaxSpeed(maxSpeed),
	AccelerationCoefficient(accelerationCoefficient),
	RelaxationTime(relaxationTime),
	RepulsiveAgent(repulsiveAgent),
	RepulsiveAgentFactor(repulsiveAgentFactor),
	RepulsiveObstacle(repulsiveObstacle),
	RepulsiveObstacleFactor(repulsiveObstacleFactor),
	ObstacleRadius(obstacleRadius),
	PlatformFactor(platformFactor),
	Perception(perception),
	Friction(friction),
	Velocity(velocity)
{ }

SFSimulator::SFSimulator()
{	
	_sim = new SF::SFSimulator() ;
}

int SFSimulator::addAgent(Microsoft::Xna::Framework::Vector2 position)
{
		 SF::Vector2 pos = SF::Vector2(position.X,position.Y);	
		 return _sim->addAgent(pos);
}

int SFSimulator::addAgent(
	Microsoft::Xna::Framework::Vector2 position, 
	float neighborDist, 
	int maxNeighbors, 
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
	float obstacleRadius,
	float platformFactor,
	float perception, 
	float friction,
	Microsoft::Xna::Framework::Vector2 velocity 
)
{
	SF::Vector2 pos = SF::Vector2(position.X,position.Y);	
	SF::Vector2 vel = SF::Vector2(velocity.X,velocity.Y);
	
	return _sim->addAgent(
		pos,
		neighborDist, 
		maxNeighbors, 
		timeHorizon, 
		timeHorizonObst,
		radius, 
		maxSpeed, 
		accelerationCoefficient, 
		relaxationTime,
		repulsiveAgent, 
		repulsiveAgentFactor, 
		repulsiveObstacle, 
		repulsiveObstacleFactor, 
		obstacleRadius,
		platformFactor,
		perception, 
		friction,
		vel
	); 
}

void SFSimulator::doStep()
{
	_sim->doStep();
}

SFSimulator::~SFSimulator()
{
	delete _sim;
}

int SFSimulator::getAgentAgentNeighbor(int agentNo, int neighborNo)
{
	return _sim->getAgentAgentNeighbor(agentNo, neighborNo);
}

int SFSimulator::getAgentMaxNeighbors(int agentNo)
{
	return _sim->getAgentMaxNeighbors(agentNo);
}

float SFSimulator::getAgentMaxSpeed(int agentNo)
{
	return _sim->getAgentMaxSpeed(agentNo);
}

float SFSimulator::getAgentNeighborDist(int agentNo) 
{
	return _sim->getAgentNeighborDist(agentNo);
}

int SFSimulator::getAgentNumAgentNeighbors(int agentNo) 
{
	return _sim->getAgentNumAgentNeighbors(agentNo);
}

void SFSimulator::getAgentPositions(
	array<int>^ agentNo, 
	array<Microsoft::Xna::Framework::Vector2>^ positions
) 
{
	positions = gcnew array<Microsoft::Xna::Framework::Vector2>(agentNo->Length);
	for(int i =0 ; i < agentNo->Length; i++)
	{
		SF::Vector2 v = _sim->getAgentPosition(agentNo[i]);
		positions[i] = Microsoft::Xna::Framework::Vector2( v.x(),v.y());		
	}
}

void SFSimulator::getAgentPrefVelocities(
	array<int>^ agentNo,
	array<Microsoft::Xna::Framework::Vector2>^ prefVelocities
) 
{
	prefVelocities = gcnew array<Microsoft::Xna::Framework::Vector2>(agentNo->Length);
	for(int i =0 ; i < agentNo->Length; i++)
	{
		SF::Vector2 v = _sim->getAgentPrefVelocity(agentNo[i]);
		prefVelocities[i] = Microsoft::Xna::Framework::Vector2( v.x(),v.y());		
	}
}

Microsoft::Xna::Framework::Vector2  SFSimulator::getAgentPosition(int agentNo) 
{
	 SF::Vector2 v = _sim->getAgentPosition(agentNo);
	 Microsoft::Xna::Framework::Vector2 vec = Microsoft::Xna::Framework::Vector2( v.x(),v.y());
	 return vec;
}

Microsoft::Xna::Framework::Vector2  SFSimulator::getAgentPrefVelocity(int agentNo) 
{
	 SF::Vector2 v = _sim->getAgentPrefVelocity(agentNo);
	 Microsoft::Xna::Framework::Vector2 vec = Microsoft::Xna::Framework::Vector2( v.x(),v.y());
	 return vec;
}

float SFSimulator::getAgentRadius(int agentNo) 
{
	return _sim->getAgentRadius(agentNo);
}

Microsoft::Xna::Framework::Vector2 SFSimulator::getAgentVelocity(int agentNo) 
{
	 SF::Vector2 v = _sim->getAgentVelocity(agentNo);
	 Microsoft::Xna::Framework::Vector2 vec = Microsoft::Xna::Framework::Vector2( v.x(),v.y());
	 return vec;
}

float SFSimulator::getGlobalTime() 
{
	return _sim->getGlobalTime();
}

int SFSimulator::getNumAgents() 
{
	return _sim->getNumAgents();
}

float SFSimulator::getTimeStep() 
{
	return _sim->getTimeStep();
}

void SFSimulator::setAgentDefaults(AgentProperty^ ap)
{
	SF::AgentPropertyConfig ac(
		ap->NeighborDist, 
		ap->MaxNeighbors,
		ap->TimeHorizon,
		ap->ObsHorizon,
		ap->Radius,
		ap->MaxSpeed,
		ap->AccelerationCoefficient,
		ap->RelaxationTime,
		ap->RepulsiveAgent,
		ap->RepulsiveAgentFactor,
		ap->RepulsiveObstacle,
		ap->RepulsiveObstacleFactor,
		ap->ObstacleRadius,
		ap->PlatformFactor,
		ap->Perception,
		ap->Friction,
		SF::Vector2(ap->Velocity.X, ap->Velocity.Y)
	);
	
	_sim->setAgentDefaults(ac);
}

void SFSimulator::setAgentMaxNeighbors(int agentNo, int maxNeighbors)
{
	_sim->setAgentMaxNeighbors(agentNo,maxNeighbors);
}

void SFSimulator::setAgentMaxSpeed(int agentNo, float maxSpeed)
{
	_sim->setAgentMaxSpeed(agentNo,maxSpeed);
}

void SFSimulator::setAgentNeighborDist(int agentNo, float neighborDist)
{
	_sim->setAgentNeighborDist(agentNo,neighborDist);
}

void SFSimulator::setAgentPosition(
	int agentNo,  
	Microsoft::Xna::Framework::Vector2 position
)
{
	SF::Vector2 pos= SF::Vector2(position.X,position.Y);	
	_sim->setAgentPosition(agentNo,pos);
}

void SFSimulator::setAgentPrefVelocity(
	int agentNo,  
	Microsoft::Xna::Framework::Vector2 velocity
)
{
	SF::Vector2 vel= SF::Vector2(velocity.X,velocity.Y);	
	_sim->setAgentPrefVelocity(agentNo,vel);
}

void SFSimulator::setAgentPrefVelocity(
	array<int>^ agentNo,  
	array<Microsoft::Xna::Framework::Vector2>^ prefVelocity
)
{
	for(int i =0 ; i < agentNo->Length; i++)
	{
		SF::Vector2 vel= SF::Vector2(prefVelocity[i].X,prefVelocity[i].Y);	
		_sim->setAgentPrefVelocity(agentNo[i],vel);
	}
}

void SFSimulator::setAgentRadius(int agentNo, float radius)
{
	_sim->setAgentRadius(agentNo,radius);
}

void SFSimulator::setAgentVelocity(
	int agentNo,  
	Microsoft::Xna::Framework::Vector2 velocity
)
{
	SF::Vector2 vel= SF::Vector2(velocity.X,velocity.Y);	
	_sim->setAgentVelocity(agentNo,vel);
}

void SFSimulator::setTimeStep(float timeStep)
{
	_sim->setTimeStep(timeStep);
}

void SFSimulator::setAgentVelocity(
	array<int>^ agentNo,  
	array<Microsoft::Xna::Framework::Vector2>^ velocity
)
{
	for(int i =0 ; i < agentNo->Length; i++)
	{
		SF::Vector2 vel= SF::Vector2(velocity[i].X,velocity[i].Y);	
		_sim->setAgentVelocity(agentNo[i],vel);
	}
}

int SFSimulator::addObstacle(System::Collections::Generic::List<Microsoft::Xna::Framework::Vector2>^ vertices)
{
	std::vector<SF::Vector2> vert;	
	for(int i =0 ; i < vertices->Count; i++)
	{		
		vert.push_back(SF::Vector2(vertices[i].X,vertices[i].Y));
	}
	return _sim->addObstacle(vert);	
}

int SFSimulator::addObstacle(array<Microsoft::Xna::Framework::Vector2>^ vertices)
{
	std::vector<SF::Vector2> vert;	
	for(int i =0 ; i < vertices->Length; i++)
	{		
		vert.push_back(SF::Vector2(vertices[i].X,vertices[i].Y));
	}
	return _sim->addObstacle(vert);

}

void SFSimulator::processObstacles()
{
	_sim->processObstacles();
}

bool SFSimulator::queryVisibility(
	Microsoft::Xna::Framework::Vector2 point1, 
	Microsoft::Xna::Framework::Vector2 point2, 
	float radius
) 
{
	SF::Vector2 p1= SF::Vector2(point1.X,point1.Y);	
	SF::Vector2 p2= SF::Vector2(point2.X,point2.Y);	
	return _sim->queryVisibility(p1,p2,radius) ;
}

float SFSimulator::getAgentTimeHorizonObst(int agentNo) 
{
	return _sim->getAgentTimeHorizonObst(agentNo) ;
}

void SFSimulator::setAgentTimeHorizonObst(int agentNo, float timeHorizonObst)
{
	_sim->setAgentTimeHorizonObst(agentNo,timeHorizonObst) ;
}

double SFSimulator::getAgentPressure(int index)
{
	return _sim->getAgentPressure(index);
}

double SFSimulator::getObstaclePressure(int index)
{
	return _sim->getObstaclePressure(index);
}