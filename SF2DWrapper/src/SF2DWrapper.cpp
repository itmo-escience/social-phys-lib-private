// This is the main DLL file.

#include <vector>

#include "../include/SF2DWrapper.h"

using namespace SF2D;

AgentProperty::AgentProperty() {};

AgentProperty::AgentProperty(
	float neighborDist, 
	int maxNeighbors, 
	float timeHorizon,
	float radius, 
	float maxSpeed,
	float force,
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
	SFVector2 velocity
  ) :
	NeighborDist(neighborDist),
	MaxNeighbors(maxNeighbors),
	TimeHorizon(timeHorizon),
	Radius(radius),
	MaxSpeed(maxSpeed),
	Force(force),
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

int SFSimulator::addAgent(SFVector2 position)
{
	SF::Vector2 pos = SF::Vector2(position.X,position.Y);	
	return _sim->addAgent(pos);
}


int SFSimulator::addAgent(
	SFVector2 position, 
	float neighborDist, 
	int maxNeighbors, 
	float timeHorizonObst,
	float radius, 
	float maxSpeed,
	float force,
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
	SFVector2 velocity 
)
{
	SF::Vector2 pos = SF::Vector2(position.X,position.Y);	
	SF::Vector2 vel = SF::Vector2(velocity.X,velocity.Y);
	
	return _sim->addAgent(
		pos,
		neighborDist, 
		maxNeighbors, 
		timeHorizonObst,
		radius, 
		maxSpeed,
		force,
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

void SFSimulator::getAgentPositions(array<int>^ agentNo, array<SFVector2>^ positions) 
{
	positions = gcnew array<SFVector2>(agentNo->Length);
	for(size_t i = 0 ; i < agentNo->Length; i++)
	{
		SF::Vector2 v = _sim->getAgentPosition(agentNo[i]);
		positions[i] = SFVector2( v.x(),v.y());		
	}
}

void SFSimulator::getAgentPrefVelocities(array<int>^ agentNo,array<SFVector2>^ prefVelocities) 
{
	prefVelocities = gcnew array<SFVector2>(agentNo->Length);
	for(size_t i = 0 ; i < agentNo->Length; i++)
	{
		SF::Vector2 v = _sim->getAgentPrefVelocity(agentNo[i]);
		prefVelocities[i] = SFVector2( v.x(),v.y());		
	}
}

SFVector2  SFSimulator::getAgentPosition(int agentNo) 
{
	 SF::Vector2 v = _sim->getAgentPosition(agentNo);
	 SFVector2 vec = SFVector2( v.x(),v.y());
	 return vec;
}

SFVector2  SFSimulator::getAgentPrefVelocity(int agentNo) 
{
	 SF::Vector2 v = _sim->getAgentPrefVelocity(agentNo);
	 SFVector2 vec = SFVector2( v.x(),v.y());
	 return vec;
}

float SFSimulator::getAgentRadius(int agentNo) 
{
	return _sim->getAgentRadius(agentNo);
}

SFVector2 SFSimulator::getAgentVelocity(int agentNo) 
{
	 SF::Vector2 v = _sim->getAgentVelocity(agentNo);
	 SFVector2 vec = SFVector2( v.x(),v.y());
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
		ap->Radius,
		ap->MaxSpeed,
		ap->Force,
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

void SFSimulator::setAgentForce(int agentNo, float force)
{
	_sim->setAgentForce(agentNo, force);
}

void SFSimulator::setAgentNeighborDist(int agentNo, float neighborDist)
{
	_sim->setAgentNeighborDist(agentNo,neighborDist);
}

void SFSimulator::setAgentPosition(int agentNo,  SFVector2 position)
{
	SF::Vector2 pos= SF::Vector2(position.X,position.Y);	
	_sim->setAgentPosition(agentNo,pos);
}

void SFSimulator::setAgentPrefVelocity(int agentNo,  SFVector2 velocity)
{
	SF::Vector2 vel= SF::Vector2(velocity.X,velocity.Y);	
	_sim->setAgentPrefVelocity(agentNo,vel);
}

void SFSimulator::setAgentPrefVelocity(array<int>^ agentNo,  array<SFVector2>^ prefVelocity)
{
	for(size_t i = 0 ; i < agentNo->Length; i++)
	{
		SF::Vector2 vel= SF::Vector2(prefVelocity[i].X,prefVelocity[i].Y);	
		_sim->setAgentPrefVelocity(agentNo[i],vel);
	}
}

void SFSimulator::setAgentRadius(int agentNo, float radius)
{
	_sim->setAgentRadius(agentNo,radius);
}

void SFSimulator::setAgentVelocity(int agentNo,  SFVector2 velocity)
{
	SF::Vector2 vel= SF::Vector2(velocity.X,velocity.Y);	
	_sim->setAgentVelocity(agentNo,vel);
}

void SFSimulator::setTimeStep(float timeStep)
{
	_sim->setTimeStep(timeStep);
}

void SFSimulator::setAgentVelocity(array<int>^ agentNo,  array<SFVector2>^ velocity)
{
	for(size_t i = 0 ; i < agentNo->Length; i++)
	{
		SF::Vector2 vel= SF::Vector2(velocity[i].X,velocity[i].Y);	
		_sim->setAgentVelocity(agentNo[i],vel);
	}
}

int SFSimulator::addObstacle(System::Collections::Generic::List<SFVector2>^ vertices)
{
	std::vector<SF::Vector2> vert;	
	for(size_t i = 0 ; i < vertices->Count; i++)
		vert.push_back(SF::Vector2(vertices[i].X,vertices[i].Y));
	
	return _sim->addObstacle(vert);	
}

int SFSimulator::addObstacle(array<SFVector2>^ vertices)
{
	std::vector<SF::Vector2> vert;	
	for(size_t i = 0 ; i < vertices->Length; i++)
		vert.push_back(SF::Vector2(vertices[i].X,vertices[i].Y));
	
	return _sim->addObstacle(vert);
}

bool SFSimulator::deleteObstacle(size_t objectId)
{
	return _sim->deleteObstacle(objectId);
}

void SFSimulator::processObstacles()
{
	_sim->processObstacles();
}

bool SFSimulator::queryVisibility(SFVector2 point1, SFVector2 point2, float radius) 
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

void SFSimulator::setPlatformVelocity(SF3D::SFVector3 velocity)
{
	SF::Vector3 v = SF::Vector3(velocity.X, velocity.Y, velocity.Z);
	_sim->setPlatformVelocity(v);
}

SF3D::SFVector3 SFSimulator::getPlatformVelocity()
{
	SF::Vector3 v = _sim->getPlatformVelocity();
	SF3D::SFVector3 result = SF3D::SFVector3(v.x(), v.y(), v.z());

	return result;
}

void SFSimulator::setAgentFriction(int agentNo, float friction)
{
	_sim->setAgentFriction(agentNo, friction);
}

float SFSimulator::getAgentFriction(int agentNo)
{
	return _sim->getAgentFriction(agentNo);
}

void SFSimulator::setAdditionalForce(SF3D::SFVector3 velocity, SF3D::SFRotationDegreeSet set)
{
	setPlatformVelocity(velocity);
	setRotationDegreeSet(set);
}

void SFSimulator::setAttractiveForce(
	float attractiveStrength, 
	float repulsiveStrength, 
	float attractiveRange, 
	float repulsiveRange
)
{	
	_sim->setAttractiveForce(attractiveStrength, repulsiveStrength, attractiveRange, repulsiveRange);
}

void SFSimulator::setAttractiveIdList(
	int id,
	System::Collections::Generic::List<int>^ attractiveIds
)
{
	std::vector<int> to;

	for (int i = 0; i < attractiveIds->Count; i++)
		to.push_back(attractiveIds[i]);

	_sim->setAttractiveIdList(id, to);
}

void SFSimulator::addAttractiveId(int id, int newId)
{
	_sim->addAttractiveId(id, newId);
}

void SFSimulator::addAttractiveIdList(
	int id,
	System::Collections::Generic::List<int>^ attractiveIds
)
{
	std::vector<int> to;

	for (int i = 0; i < attractiveIds->Count; i++)
		to.push_back(attractiveIds[i]);

	_sim->addAttractiveIdList(id, to);
}

void SFSimulator::deleteAttractiveId(int id, int idForDelete)
{
	_sim->deleteAttractiveId(id, idForDelete);
}

void SFSimulator::deleteAttractiveIdList(
	int id,
	System::Collections::Generic::List<int>^ attractiveIds
)
{
	std::vector<int> to;

	for (int i = 0; i < attractiveIds->Count; i++)
		to.push_back(attractiveIds[i]);

	_sim->deleteAttractiveIdList(id, to);
}

void SFSimulator::setRotationDegreeSet(SF3D::SFRotationDegreeSet set)
{
	SF::Vector3 c = SF::Vector3(set.center.X, set.center.Y, set.center.Z);
	SF::RotationDegreeSet s = SF::RotationDegreeSet(set.OX, set.OY, set.OZ, c);
	_sim->setRotationDegreeSet(s);
}

SF3D::SFRotationDegreeSet SFSimulator::getRotationDegreeSet()
{
	SF::RotationDegreeSet s = _sim->getRotationDegreeSet();
	SF::Vector3 c = s.getCenter();
	SF3D::SFRotationDegreeSet result = SF3D::SFRotationDegreeSet(s.getRotationOX(), s.getRotationOY(), s.getRotationOZ(), SF3D::SFVector3(c.x(), c.y(), c.z()));

	return result;
}

System::Collections::Generic::List<int>^ SFSimulator::getAgentNeighboursIndexList(int agentNo, float radius)
{
	std::vector<size_t> in = _sim->getAgentNeighboursIndexList(agentNo, radius);
	System::Collections::Generic::List<int>^ out = gcnew System::Collections::Generic::List<int>(); 
	
	for(size_t i = 0; i < in.size(); i++)
		out->Add(in[i]);

	return out;
}

double SFSimulator::getAgentPressure(int index)
{
	return _sim->getAgentPressure(index);
}

void SFSimulator::deleteAgent(int index)
{
	_sim->deleteAgent(index);
}

double SFSimulator::getObstaclePressure(int index)
{
	return _sim->getObstaclePressure(index);
}

SF2D::SFVector2 SFSimulator::getObstacleTrajectory(int index)
{
	SF::Vector2 ot = _sim->getObstacleTrajectory(index);

	return SF2D::SFVector2(ot.x(), ot.y());
}

SF2D::SFVector2 SFSimulator::getAgentRepulsiveForce(int index)
{
	SF::Vector2 ot = _sim->getAgentRepulsiveForce(index);

	return SF2D::SFVector2(ot.x(), ot.y());
}


System::Collections::Generic::List<double>^ SFSimulator::getAgentPressureList()
{
	System::Collections::Generic::List<double>^ out = gcnew System::Collections::Generic::List<double>();
	
	for (size_t i = 0; i < getNumAgents(); i++)
		out->Add(getAgentPressure(i));
	
	return out;
}

System::Collections::Generic::List<double>^ SFSimulator::getObstaclePressureList()
{
	System::Collections::Generic::List<double>^ out = gcnew System::Collections::Generic::List<double>();

	for (size_t i = 0; i < getNumAgents(); i++)
		out->Add(getObstaclePressure(i));

	return out;
}

System::Collections::Generic::List<SFVector2>^ SFSimulator::getPositionList()
{
	System::Collections::Generic::List<SFVector2>^ out = gcnew System::Collections::Generic::List<SFVector2>();

	for (size_t i = 0; i < getNumAgents(); i++)
		out->Add(getAgentPosition(i));

	return out;
}

System::Collections::Generic::List<int>^ SFSimulator::getDeletedIDList()
{
	auto in = _sim->getDeletedIDList();
	System::Collections::Generic::List<int>^ out = gcnew System::Collections::Generic::List<int>();
	
	for each (auto del in in)
		out->Add(del);
	
	return out;
}

System::Collections::Generic::List<int>^ SFSimulator::getCountOfAliveAndDead()
{
	auto in = _sim->getCountOfAliveAndDead();
	System::Collections::Generic::List<int>^ out = gcnew System::Collections::Generic::List<int>();

	for each (auto del in in)
		out->Add(del);

	return out;
}

void SFSimulator::updateSFParameters(float newRepulsiveAgent, float newRepulsiveAgentFactor, float newRepulsiveObstacle, float newRepulsiveObstacleFactor)
{
	_sim->updateSFParameters(newRepulsiveAgent, newRepulsiveAgentFactor, newRepulsiveObstacle, newRepulsiveObstacleFactor);
}
