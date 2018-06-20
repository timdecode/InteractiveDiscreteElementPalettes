// Copyright (c) 2018 Timothy Davison. All rights reserved.

#include "RegionGrowing.h"

#include "FlexElements.h"
#include "ShipEditorSimulation/MeshSimulation.h"

#include "Utility.h"









UFlexElements::UFlexElements()
{
	bWantsInitializeComponent = true;
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_EndPhysics;
	bIsActive = true;
}

auto UFlexElements::_initFlex() -> void
{
	_library = NvFlexInit();

	NvFlexSetSolverDescDefaults( &_solverDescription );
	_solverDescription.maxParticles = _maxParticles;
	_solverDescription.maxNeighborsPerParticle = _maxNeighbors;
	_solverDescription.maxDiffuseParticles = 0;

	_solver = NvFlexCreateSolver( _library, &_solverDescription );

	_particleBuffer = NvFlexAllocBuffer( _library, _maxParticles, sizeof( FVector4 ), eNvFlexBufferHost );
	_velocityBuffer = NvFlexAllocBuffer( _library, _maxParticles, sizeof( FVector4 ), eNvFlexBufferHost );
	_phaseBuffer = NvFlexAllocBuffer( _library, _maxParticles, sizeof( int ), eNvFlexBufferHost );
	_activeBuffer = NvFlexAllocBuffer( _library, _maxParticles, sizeof( int ), eNvFlexBufferHost );

	_geometryBuffer = NvFlexAllocBuffer( _library, nGeometries, sizeof( NvFlexCollisionGeometry ), eNvFlexBufferHost );
	_shapePositionsBuffer = NvFlexAllocBuffer( _library, nGeometries, sizeof( FVector4 ), eNvFlexBufferHost );
	_shapeRotationsBuffer = NvFlexAllocBuffer( _library, nGeometries, sizeof( FVector4 ), eNvFlexBufferHost );
	_shapeFlagsBuffer = NvFlexAllocBuffer( _library, nGeometries, sizeof( int ), eNvFlexBufferHost );

	{
		const int neighboursSize = _solverDescription.maxParticles * _solverDescription.maxNeighborsPerParticle;

		_neighborsIndicesBuffer = std::make_unique<NvFlexVector<int>>( _library, neighboursSize );
		_neighborsCountsBuffer = std::make_unique<NvFlexVector<int>>( _library, _solverDescription.maxParticles );
		_neighborsAPIToInternal = std::make_unique<NvFlexVector<int>>( _library, neighboursSize );
		_neighborsInternalToAPI = std::make_unique<NvFlexVector<int>>( _library, neighboursSize );
	}

	_loadMesh( _meshComponent );


	_initParams();
	NvFlexSetParams( _solver, &_params );
}

void UFlexElements::TickComponent( float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction )
{
	Super::TickComponent( DeltaTime, TickType, ThisTickFunction );

	if(!_playing)
		return;

	if(_needsFlexReset)
	{
		if(_didInitFlex)
			_uninit();

		_initFlex();
		_didInitFlex = true;

		_needsFlexReset = false;
	}

	if(!_didInitFlex)
		return;

	// map
	FVector4 * particles = (FVector4*)NvFlexMap( _particleBuffer, eNvFlexMapWait );
	FVector * velocities = (FVector*)NvFlexMap( _velocityBuffer, eNvFlexMapWait );
	int * phases = (int*)NvFlexMap( _phaseBuffer, eNvFlexMapWait );
	int * active = (int*)NvFlexMap( _activeBuffer, eNvFlexMapWait );

	NvFlexCollisionGeometry * geometry = (NvFlexCollisionGeometry*)NvFlexMap( _geometryBuffer, eNvFlexMapWait );
	FVector4 * shapePositions = (FVector4*)NvFlexMap( _shapePositionsBuffer, eNvFlexMapWait );
	FQuat * shapeRotations = (FQuat*)NvFlexMap( _shapeRotationsBuffer, eNvFlexMapWait );
	int * shapeFlags = (int*)NvFlexMap( _shapeFlagsBuffer, eNvFlexMapWait );


	// spawn
	if(!_didSpawnShapesAndMesh)
	{
		_spawnShapes( geometry, shapePositions, shapeRotations, shapeFlags );
		_spawnMesh( geometry, shapePositions, shapeRotations, shapeFlags );

		_didSpawnShapesAndMesh = true;
	}

	_spawnDespawnParticles( particles, velocities, phases, active );

	(FVector4&)shapePositions[0] = _spherePosition;
	shapePositions[0].W = 1.0f;

	// Read Flex state into the simulation
	_readFlexState( particles, velocities, phases );

	// simulate
	_tickGraph( DeltaTime );

	UATPSynthaseSimulation * synthaseSimulation = graphSimulation.simulation<UATPSynthaseSimulation>();
	synthaseSimulation->flexTick( 
		DeltaTime, 
		*_neighborsIndicesBuffer.get(), 
		*_neighborsCountsBuffer.get(),
		*_neighborsAPIToInternal.get(), 
		*_neighborsInternalToAPI.get(), 
		_solverDescription.maxParticles 
	);

	// simulate particle rotation outside of Flex
	_integrateRotations( DeltaTime );

	// Write simulation state back into Flex
	_writeFlexState( particles, velocities, phases, active );

	// unmap
	NvFlexUnmap( _particleBuffer ); particles = nullptr;
	NvFlexUnmap( _velocityBuffer ); velocities = nullptr;
	NvFlexUnmap( _phaseBuffer ); phases = nullptr;
	NvFlexUnmap( _activeBuffer ); active = nullptr;

	NvFlexUnmap( _geometryBuffer ); 
	NvFlexUnmap( _shapePositionsBuffer );
	NvFlexUnmap( _shapeRotationsBuffer );
	NvFlexUnmap( _shapeFlagsBuffer );

	// write to device (async)
	NvFlexSetParticles( _solver, _particleBuffer, nullptr );
	NvFlexSetVelocities( _solver, _velocityBuffer, nullptr );
	NvFlexSetPhases( _solver, _phaseBuffer, nullptr );
	NvFlexSetActive( _solver, _activeBuffer, nullptr );
	NvFlexSetActiveCount( _solver, nParticles );

	NvFlexSetShapes( _solver, _geometryBuffer, _shapePositionsBuffer, _shapeRotationsBuffer, nullptr, nullptr, _shapeFlagsBuffer, nGeometries );

	// tick
	NvFlexSetParams( _solver, &_params );

	NvFlexUpdateSolver( _solver, DeltaTime, 1, false );

	// read back (async)
	NvFlexGetParticles( _solver, _particleBuffer, nullptr );
	NvFlexGetVelocities( _solver, _velocityBuffer, nullptr );
	NvFlexGetPhases( _solver, _phaseBuffer, nullptr );
	NvFlexGetActive( _solver, _activeBuffer, nullptr );
	NvFlexGetNeighbors( 
		_solver, 
		_neighborsIndicesBuffer->buffer, 
		_neighborsCountsBuffer->buffer,
		_neighborsAPIToInternal->buffer, 
		_neighborsInternalToAPI->buffer 
	);
}

auto UFlexElements::_uninit() -> void
{
	// we already released
	if(!_solver)
		return;

	NvFlexFreeBuffer( _particleBuffer ); _particleBuffer = nullptr;
	NvFlexFreeBuffer( _velocityBuffer ); _velocityBuffer = nullptr;
	NvFlexFreeBuffer( _phaseBuffer ); _phaseBuffer = nullptr;
	NvFlexFreeBuffer( _activeBuffer ); _activeBuffer = nullptr;

	NvFlexFreeBuffer( _geometryBuffer ); _geometryBuffer = nullptr;
	NvFlexFreeBuffer( _shapePositionsBuffer ); _shapePositionsBuffer = nullptr;
	NvFlexFreeBuffer( _shapeRotationsBuffer ); _shapeRotationsBuffer = nullptr;
	NvFlexFreeBuffer( _shapeFlagsBuffer ); _shapeFlagsBuffer = nullptr;

	_mesh0_indices = nullptr;
	_mesh0_positions = nullptr;

	_neighborsIndicesBuffer = nullptr;
	_neighborsCountsBuffer = nullptr;
	_neighborsAPIToInternal = nullptr;
	_neighborsInternalToAPI = nullptr;

	NvFlexDestroySolver( _solver ); _solver = nullptr;
	NvFlexShutdown( _library ); _library = nullptr;

	_didSpawnShapesAndMesh = false;
}

void UFlexElements::_tickGraph( float DeltaTime )
{
	if(_didInitGraphSimulation)
	{
		// tick the engines
		for(auto engine : graphSimulation.simulations)
		{
			engine->tick( DeltaTime );
		}
	}
}

void UFlexElements::BeginDestroy()
{
	_uninit();

	Super::BeginDestroy();
}

void UFlexElements::InitializeComponent()
{
	Super::InitializeComponent();

}

void UFlexElements::BeginPlay()
{
	Super::BeginPlay();

	_initGraphSimulationsAndControllers();
}

auto UFlexElements::clear() -> void
{
	graphSimulation.clear();

	_needsFlexReset = true;
	_didInitFlex = false;
	_didSpawnShapesAndMesh = false;
}

auto UFlexElements::pause() -> void
{
	_playing = false;
}

auto UFlexElements::play() -> void
{
	_playing = true;
}

auto UFlexElements::updateSphereWorldSpace( FVector position ) -> void
{
	FTransform transform = GetOwner()->GetRootComponent()->GetComponentTransform();

	_spherePosition = transform.InverseTransformPosition( position );
}

auto UFlexElements::loadExportedDomainInfo( OutputDomainExport& exportInfo ) -> void
{
	graphSimulation.clear();

	graphSimulation.beginTransaction();

	_meshComponent = exportInfo.staticMeshComponent;

	_meshInterface = exportInfo.meshInterface;

	// setup the instance manager
	for(auto& type_meshInfo_Pair : exportInfo.meshInfo())
	{
		auto elementType = type_meshInfo_Pair.first;
		auto& meshInfo = type_meshInfo_Pair.second;

	}

	auto meshInfoByTypes = exportInfo.meshInfo();

	FTransform worldToUsTransform = GetOwner()->GetRootComponent()->GetComponentTransform().Inverse();

	FTransform toWorld = GetOwner()->GetRootComponent()->GetComponentTransform();

	// setup the positions
	for(auto& pair : exportInfo.elementsAndTransforms())
	{
		FTransform transform = pair.transform; // world transform

		// convert the world-element positions to our local space
		transform = transform * worldToUsTransform;

		FVector position = transform.GetLocation();
		FQuat rotation = transform.GetRotation();

		int32 nodeIndex = graphSimulation.addNode( position, rotation, transform.GetScale3D().X );
		FGraphNode& node = graphSimulation.node( nodeIndex );

		// set mesh info
		if(!node.hasComponent<FGraphMesh>())
		{
			FGraphMesh& mesh = node.addComponent<FGraphMesh>( graphSimulation );

			OutputDomainExport::MeshInformation meshInfo = meshInfoByTypes[pair.element->type];

			mesh.staticMesh = meshInfo.mesh;
			mesh.material = meshInfo.materials.Num() > 0 ? meshInfo.materials[0] : nullptr;
		}

		// velocity
		if(!node.hasComponent<FVelocityGraphObject>())
		{
			node.addComponent<FVelocityGraphObject>( graphSimulation );
		}

		// cached element
		if(!node.hasComponent<FCachedElementGraphObject>())
		{
			FCachedElementGraphObject& cachcedElement = node.addComponent<FCachedElementGraphObject>( graphSimulation );

			cachcedElement.element = *pair.element;
		}

		// add graph objects from the element
		for(FTimStructBox& box : pair.element->graphObjects)
		{
			if(!box.IsValid())
				continue;

			// create the graph object in the simulation
			ComponentType type = FGraphObject::componentType( box.scriptStruct );

			FGraphObject * object = node.addComponent( graphSimulation, type );

			// then copy the memory from the element
			box.scriptStruct->CopyScriptStruct( object, box.structMemory );

			object->nodeIndex = node.id;
		}

		bool onSurface = false;

		// constrain objects to the surface if they have a face
		if(pair.element->faceIndex > 0)
		{
			FSurfaceBoundGraphObject& boundObject = node.addComponent<FSurfaceBoundGraphObject>( graphSimulation );

			FVector p = toWorld.TransformPosition( node.position );

			auto rotationAndNormal = _meshInterface->rotationAndNormalAtIndex( pair.element->faceIndex );

			boundObject.lastSurfaceRotation = rotationAndNormal.first;
			boundObject.faceIndex = pair.element->faceIndex;

			onSurface = true;
		}

		// set particle
		if(node.hasComponent<FFlexParticleObject>())
		{
			FFlexParticleObject& particleObject = node.component<FFlexParticleObject>( graphSimulation );
			particleObject.group = 0;

			// don't interact with the surface
			if(onSurface)
				particleObject.channel = eNvFlexPhaseShapeChannel1;
			else
				particleObject.channel = eNvFlexPhaseShapeChannel0;
		}

		if(node.hasComponent<FStaticPositionObject>())
		{
			node.component<FStaticPositionObject>( graphSimulation ).position = node.position;
		}
	}

	graphSimulation.endTransaction();

	URandomWalkSimulation * randomWalkSim = graphSimulation.simulation<URandomWalkSimulation>();

	randomWalkSim->meshInterface = _meshInterface;

	_needsFlexReset = true;
}

auto UFlexElements::exportElementDomain() -> std::vector<FElement>
{
	std::vector<FElement> elements;

	elements.reserve( graphSimulation.numNodes() );

	// The simulation coordinates are in local space, the element framework works in world-space.
	// So, we need to map.
	FTransform toWorld = GetOwner()->GetRootComponent()->GetComponentTransform();

	const ComponentType tCachedElementGraphObjectScriptStruct = componentType<FCachedElementGraphObject>();

	for(FGraphNode& node : graphSimulation.allNodes)
	{
		if(!node.isValid())
			continue;

		elements.emplace_back();
		
		FElement& element = elements.back();

		// rebuild Region Growing parameters
		if(node.hasComponent<FCachedElementGraphObject>())
		{
			FCachedElementGraphObject& cachcedElement = node.component<FCachedElementGraphObject>( graphSimulation );

			FElement toAssign = cachcedElement.element;
			toAssign.graphObjects.Empty();

			element = toAssign;
		}

		element.position = eigen( toWorld.TransformPosition(node.position) );
		element.rotation = eigen( toWorld.TransformRotation(node.orientation) );
		element.hackScale = toWorld.GetScale3D().X * node.scale;

		auto componentTypes = node.components;

		for(auto intType : componentTypes)
		{
			ComponentType componentType( intType );

			// don't copy cached elements
			if(tCachedElementGraphObjectScriptStruct == componentType)
				continue;

			FGraphObject * object = node.component( graphSimulation, componentType );

			FTimStructBox structBox;

			structBox.scriptStruct = FGraphObject::componentStruct( componentType );
			structBox.initStruct();

			structBox.scriptStruct->CopyScriptStruct( structBox.structMemory, object );

			element.graphObjects.Add( structBox );
		}


	}

	return elements;
}

auto UFlexElements::setInstanceManagerBounds( FBox bounds ) -> void
{
	for(int i = 0; i < 3; ++i)
	{
		FVector n_up( 0.0f );
		n_up[i] = 1.0f;

		FVector n_down( 0.0f );
		n_down[i] = -1.0f;

		float w_up = -(bounds.Min[i] * n_up[i]);
		float w_down = -(bounds.Max[i] * n_down[i]);

		flexParams.planes[i * 2 + 0] = FVector4( n_up.X, n_up.Y, n_up.Z, w_up );
		flexParams.planes[i * 2 + 1] = FVector4( n_down.X, n_down.Y, n_down.Z, w_down );
	}

	flexParams.numPlanes = 6;
}

auto UFlexElements::_initGraphSimulationsAndControllers() -> void
{
	_FFlexParticleObjectType = FGraphObject::componentType( FFlexParticleObject::StaticStruct() );

	// nuke any null simulations
	graphSimulation.simulations.Remove( nullptr );

	// the order is important
	// disconnect all listeners
	// controllers register their simulations
	// then the simulations are initialized by the graph
	// and finally, we let the simulations know about us
	graphSimulation.resetListeners();
	//_initObjectControllers();

	graphSimulation.instantiateSimulations();

	for(UObjectSimulation* simulation : graphSimulation.simulations)
	{
		simulation->actor = GetOwner();
	}

	URandomWalkSimulation * randomWalkSim = graphSimulation.simulation<URandomWalkSimulation>();

	randomWalkSim->toWorld = GetOwner()->GetRootComponent()->GetComponentTransform();
	
	// setup listener
	graphSimulation.addListener<FFlexParticleObject>( this );

	graphSimulation.init( this );

	_didInitGraphSimulation = true;
}

auto UFlexElements::_initParams() -> void
{
	// sim params
	_params = flexParams.toNvFlexParams();

	if(_params.solidRestDistance == 0.0f)
		_params.solidRestDistance = _params.radius;

	// if fluid present then we assume solid particles have the same radius
	if(_params.fluidRestDistance > 0.0f)
		_params.solidRestDistance = _params.fluidRestDistance;

	// set collision distance automatically based on rest distance if not already set
	if(_params.collisionDistance == 0.0f)
		_params.collisionDistance = std::max( _params.solidRestDistance, _params.fluidRestDistance )*0.5f;

	// default particle friction to 10% of shape friction
	if(_params.particleFriction == 0.0f)
		_params.particleFriction = _params.dynamicFriction*0.1f;

	// add a margin for detecting contacts between particles and shapes
	if(_params.shapeCollisionMargin == 0.0f)
		_params.shapeCollisionMargin = _params.collisionDistance*0.5f;
}

auto UFlexElements::_instanceCount() -> size_t
{
	size_t sum = 0;
	for(auto& pair : _instanceManager.elementTypesToInstancedMeshes)
	{
		sum += pair.second->GetInstanceCount();
	}

	return sum;
}

auto UFlexElements::_spawnDespawnParticles( FVector4 * positions, FVector * velocities, int * phases, int * active ) -> void
{
	FRandomStream rand;
	rand.GenerateNewSeed();

	float r = _params.radius;

	nParticles = graphSimulation.componentStorage<FFlexParticleObject>().size();

	if(_flexParticlesToAdd.size() == 0 && _flexParticlesToRemove.size() == 0)
		return;

	// spawn particles
	for(FGraphNodeHandle handle : _flexParticlesToAdd)
	{
		FGraphNode& node = handle( graphSimulation );

		if(!node.isValid())
			continue;

		if(	!node.hasComponent<FVelocityGraphObject>() ||
			!node.hasComponent<FFlexParticleObject>() )
			continue;

		FFlexParticleObject& particleObject = node.component<FFlexParticleObject>( graphSimulation );
		FVelocityGraphObject& velocityObject = node.component<FVelocityGraphObject>( graphSimulation );

		FVector p = node.position;
		FVector v = velocityObject.linearVelocity;

		positions[node.id] = p;
		positions[node.id].W = 0.125;
		
		velocities[node.id] = v;
		
		phases[node.id] = NvFlexMakePhaseWithChannels( particleObject.group, eNvFlexPhaseSelfCollide, particleObject.channel );
	}

	// We don't need to remove particles, just deactivate them.
	// We do this by update the active set for the entire simulation---brute force, 
	// but we do more expensive things like reading every particle position.
	{
		auto& flexParticles = graphSimulation.componentStorage<FFlexParticleObject>();

		size_t i = 0;
		for( FFlexParticleObject& particle : flexParticles )
		{
			active[i] = particle.nodeIndex;

			++i;
		}
	}

	_flexParticlesToAdd.clear();
	_flexParticlesToRemove.clear();
}


void UFlexElements::_spawnShapes( NvFlexCollisionGeometry * geometry, FVector4 * shapePositions, FQuat * shapeRotations, int * shapeFlags )
{
	shapeFlags[0] = NvFlexMakeShapeFlagsWithChannels( eNvFlexShapeSphere, true, eNvFlexPhaseShapeChannel0 );
	geometry[0].sphere.radius = 20.0f;

	(FVector4&)shapePositions[0] = _spherePosition; 	
	shapePositions[0].W = 1.0f;

	shapeRotations[0] = FQuat::Identity;
}

void UFlexElements::_spawnMesh( NvFlexCollisionGeometry * geometry, FVector4 * shapePositions, FQuat * shapeRotations, int * shapeFlags )
{


	_mesh0_Id = NvFlexCreateTriangleMesh( _library );

	NvFlexUpdateTriangleMesh(
		_library,
		_mesh0_Id,
		_mesh0_positions->buffer,
		_mesh0_indices->buffer,
		_mesh0_positions->size(),
		_mesh0_indices->size() / 3, // num triangles?
		&_mesh0_bounds.Min[0],
		&_mesh0_bounds.Max[0]
	);

	shapeFlags[1] = NvFlexMakeShapeFlagsWithChannels( eNvFlexShapeTriangleMesh, false, eNvFlexPhaseShapeChannel0 | eNvFlexPhaseShapeChannel1 );

	geometry[1].triMesh.mesh = _mesh0_Id;

	geometry[1].triMesh.scale[0] = 1.0f;
	geometry[1].triMesh.scale[1] = 1.0f;
	geometry[1].triMesh.scale[2] = 1.0f;

	shapePositions[1] = FVector4( 0.0f );
	shapeRotations[1] = FQuat::Identity;


}

auto UFlexElements::_readFlexState( FVector4 * positions, FVector * velocities, int * phases ) -> void
{
	// read positions
	auto& particles = graphSimulation.componentStorage<FFlexParticleObject>();

	for(FFlexParticleObject& particle : particles)
	{
		FGraphNode& node = graphSimulation.node( particle.nodeIndex );

		FVector4 p4 = positions[node.id];
		FVector p( p4.X, p4.Y, p4.Z );

		node.position = p;
	}

	// read velocities
	auto& velocityObjects = graphSimulation.componentStorage<FVelocityGraphObject>();

	for(FVelocityGraphObject& velocityObject : velocityObjects)
	{
		auto id = velocityObject.nodeIndex;

		FVector v = velocities[id];

		velocityObject.linearVelocity = v;
	}
}




auto UFlexElements::_writeFlexState( FVector4 * positions, FVector * velocities, int * phases, int * active ) -> void
{
	// write positions
	auto& particles = graphSimulation.componentStorage<FFlexParticleObject>();

	size_t i = 0;

	for(FFlexParticleObject& particle : particles)
	{
		FGraphNode& node = graphSimulation.node( particle.nodeIndex );

		FVector4& p = positions[node.id];

		float w = p.W;

		p = FVector4( node.position, w );

		active[i] = particle.nodeIndex;

		++i;
	}

	// write velocities
	auto& velocityObjects = graphSimulation.componentStorage<FVelocityGraphObject>();

	for(FVelocityGraphObject& velocityObject : velocityObjects)
	{
		auto id = velocityObject.nodeIndex;

		velocities[id] = velocityObject.linearVelocity;
	}
}

auto UFlexElements::_integrateRotations(float deltaT) -> void
{
	auto& velocityObjects = graphSimulation.componentStorage<FVelocityGraphObject>();

	for(FVelocityGraphObject& velocityObject : velocityObjects)
	{
		// https://gamedev.stackexchange.com/questions/108920/applying-angular-velocity-to-quaternion
		// q = q_0 + dt/2 * w * q_0

		FGraphNodeHandle node = FGraphNodeHandle(velocityObject.nodeIndex);

		FVector angularVelocity = velocityObject.angularVelocity;

		FQuat w = FQuat( angularVelocity.X, angularVelocity.Y, angularVelocity.Z, 0.0f );

		FQuat& rotation = node( graphSimulation ).orientation;

		w *= 0.5f * deltaT;
		w *= rotation;

		rotation = rotation + (w);

		rotation.Normalize();
	}
}


auto UFlexElements::_loadMesh( UStaticMeshComponent * meshComponent ) -> void
{
	FTransform worldToUsTransform = GetOwner()->GetRootComponent()->GetComponentTransform().Inverse();


	FTransform toUsTransform = meshComponent->GetComponentTransform();
	toUsTransform = toUsTransform * worldToUsTransform;

	UStaticMesh& uMesh = *meshComponent->GetStaticMesh();

	if(uMesh.RenderData == nullptr) return;
	if(uMesh.RenderData->LODResources.Num() == 0) return;

	// data to be consumed by flex
	std::vector<FVector4> flexPositions;
	std::vector<int> flexIndices;

	FStaticMeshLODResources& resource = uMesh.RenderData->LODResources[meshComponent->PreviousLODLevel];

	FPositionVertexBuffer& positionBuffer = resource.PositionVertexBuffer;
	FRawStaticIndexBuffer& indexBuffer = resource.IndexBuffer;
	FStaticMeshVertexBuffer& vertexBuffer = resource.VertexBuffer;

	uint32 vCount = positionBuffer.GetNumVertices();

	// Unreal stores duplicate vertices, we need to resolve this with a map
	std::unordered_map<FVector, int32> vertexLookup;

	std::vector<uint32> vertexIndices;
	vertexIndices.resize( vCount );

	_mesh0_bounds = FBox();

	for(uint32 i = 0; i < vCount; ++i)
	{
		const FVector& v = positionBuffer.VertexPosition( i );


		auto found = vertexLookup.find( v );
		if(found == vertexLookup.end())
		{
			vertexLookup[v] = flexPositions.size();

			vertexIndices[i] = flexPositions.size();

			FVector vTransformed = toUsTransform.TransformPosition( v );

			_mesh0_bounds += vTransformed;

			flexPositions.push_back( FVector4( vTransformed, 0.0f) );
		}
		else
			vertexIndices[i] = found->second;
	}

	FIndexArrayView indexArrayView = indexBuffer.GetArrayView();
	int32 iCount = indexArrayView.Num();
	for(int32 i = 0; i + 2 < iCount; i += 3)
	{
		int i0 = vertexIndices[indexArrayView[i + 0]];
		int i1 = vertexIndices[indexArrayView[i + 1]];
		int i2 = vertexIndices[indexArrayView[i + 2]];

		// double-sided geometry
		flexIndices.push_back( i0 );
		flexIndices.push_back( i1 );
		flexIndices.push_back( i2 );

		flexIndices.push_back( i0 );
		flexIndices.push_back( i2 );
		flexIndices.push_back( i1 );
	}

	_mesh0_positions = std::make_unique< NvFlexVector<FVector4> >( _library, &flexPositions[0], flexPositions.size() );
	_mesh0_indices = std::make_unique< NvFlexVector<int> >( _library, &flexIndices[0], flexIndices.size() );
}

void UFlexElements::componentAdded( FGraphNodeHandle node, ComponentType type )
{
	if(type != _FFlexParticleObjectType)
		return;

	_flexParticlesToAdd.insert( node );
	_flexParticlesToRemove.erase( node );
}

void UFlexElements::componentRemoved( FGraphNodeHandle node, ComponentType type )
{
	if(type != _FFlexParticleObjectType)
		return;

	_flexParticlesToRemove.insert( node );
	_flexParticlesToAdd.erase( node );
}

void UFlexElements::componentUpdated( FGraphNodeHandle node, ComponentType type )
{
}

void UFlexElements::connectionAdded( int32 edgeIndex, FGraphEdge& edge, ComponentType type )
{
}

void UFlexElements::connectionRemoved( int32 edgeIndex, FGraphEdge& oldEdge, ComponentType type )
{
}



































void URandomWalkSimulation::init()
{

}

void URandomWalkSimulation::tick( float deltaT )
{
	auto& walkers = graph->componentStorage<FRandomWalkGraphObject>();

	FRandomStream rand;
	rand.GenerateNewSeed();

	auto& surfaceDwellers = graph->componentStorage<FSurfaceBoundGraphObject>();


	for(FSurfaceBoundGraphObject& dweller : surfaceDwellers)
	{
		FGraphNode& node = graph->node( dweller.nodeIndex );

		FVector p = toWorld.TransformPosition( node.position );

		auto nearest = meshInterface->nearestPointOnMesh( p );

		auto rotationAndNormal = meshInterface->rotationAndNormalAtIndex( nearest.faceIndex );

		FQuat surfaceRotation = toWorld.InverseTransformRotation(rotationAndNormal.first);

		FQuat rotation = surfaceRotation * dweller.lastSurfaceRotation.Inverse();

		node.position = toWorld.InverseTransformPosition( nearest.point );
		node.orientation = rotation * node.orientation;

		dweller.lastSurfaceRotation = surfaceRotation;

		// project linear velocity back onto the surface
		if(node.hasComponent<FVelocityGraphObject>())
		{
			FVelocityGraphObject& velocityObject = node.component<FVelocityGraphObject>(*graph);

			FVector& v = velocityObject.linearVelocity;

			// rotate the vector onto the surface
			v = rotation.RotateVector( v );

			// trip any residual vertical motion

			FVector normal = toWorld.InverseTransformVectorNoScale(rotationAndNormal.second);

			v = v - (FVector::DotProduct( v, normal ) * normal);
		}
	}

	for(FRandomWalkGraphObject& walker : walkers)
	{
		walker.timeLeft -= deltaT;

		// times' up, choose a random direction
		if(walker.timeLeft >= 0.0f )
			continue;

		FGraphNode& node = graph->node( walker.nodeIndex );

		if(!node.hasComponent<FVelocityGraphObject>())
			continue;

		FVelocityGraphObject& velocityObject = node.component<FVelocityGraphObject>( *graph );

		walker.timeLeft = rand.GetFraction() * 0.3f;


		// surface walker
		if(node.hasComponent<FSurfaceBoundGraphObject>())
		{
			FSurfaceBoundGraphObject& boundObject = node.component<FSurfaceBoundGraphObject>( *graph );

			// calculate random unit circle vector
			float t = rand.GetFraction() * 2.0f * M_PI;

			FVector v = FVector::ZeroVector;

			FMath::SinCos( &v.Y, &v.X, t );

			v = v *  (walker.baseVelocity + walker.maxVelocityOffset * rand.GetFraction());

			// now, we need to rotate it from surface space into sim space
			auto rotationAndNormal = meshInterface->rotationAndNormalAtIndex( boundObject.faceIndex );

			FQuat r = toWorld.InverseTransformRotation(rotationAndNormal.first);

			v = r.RotateVector( v );

			velocityObject.linearVelocity = v;
		}
		else // space walker
		{
			FVector v = rand.GetUnitVector() * (walker.baseVelocity + walker.maxVelocityOffset * rand.GetFraction());

			velocityObject.linearVelocity += v;
		}
	}

	auto& spinners = graph->componentStorage<FSpinnerGraphObject>();

	for(FSpinnerGraphObject& spinner : spinners)
	{
		FGraphNodeHandle node( spinner.nodeIndex );

		if(!node( graph ).hasComponent<FVelocityGraphObject>())
			continue;

 		FVelocityGraphObject& velocityObject = node( graph ).component<FVelocityGraphObject>( *graph );


		FQuat& orientation = node( graph ).orientation;

		FVector up = FVector::UpVector;

		up = orientation.RotateVector( up );

		up *= spinner.angularVelocityMagnitude;

		velocityObject.angularVelocity = up;
	}

	auto& staticPositions = graph->componentStorage<FStaticPositionObject>();

	for(FStaticPositionObject& staticObject : staticPositions)
	{
		FGraphNode& node = graph->node( staticObject.nodeIndex );

		node.position = staticObject.position;

		if(node.hasComponent<FVelocityGraphObject>())
			node.component<FVelocityGraphObject>(*graph ).linearVelocity = FVector::ZeroVector;
	}
}




void UATPSynthaseSimulation::init()
{

}

void UATPSynthaseSimulation::tick( float deltaT )
{

}

void UATPSynthaseSimulation::flexTick( 
	float deltaT, 
	NvFlexVector<int>& neighbourIndices, 
	NvFlexVector<int>& neighbourCounts, 
	NvFlexVector<int>& apiToInternal, 
	NvFlexVector<int>& internalToAPI, 
	int maxParticles 
)
{
	auto& synthases = graph->componentStorage<FATPSynthaseGraphObject>();

	neighbourIndices.map();
	neighbourCounts.map();
	apiToInternal.map();
	internalToAPI.map();

	const int stride = maxParticles;

	// If we have hydrogen below us, spin and teleport it across the membrane
	// If we are spinning and have ADP above us, destroy it and produce ADP

	// we'll be adding and removing components, so start a transaction
	graph->beginTransaction();

	for(FATPSynthaseGraphObject& synthase : synthases)
	{
		auto nodeIndex = synthase.nodeIndex;
		FGraphNode& atpSynthaseNode = graph->node(nodeIndex);

		if(!atpSynthaseNode.hasComponent<FFlexParticleObject>())
			continue;

		FVector atpSynthaseUp = FVector::UpVector;
		atpSynthaseUp = atpSynthaseNode.orientation.RotateVector( atpSynthaseUp );

		const float interactionRadiusSqrd = synthase.hyrogenInteractionRadius * synthase.hyrogenInteractionRadius;

		int flexInternalIndex = apiToInternal[nodeIndex];

		// use hydrogen to spin
		if(synthase.timerH < 0.0f)
		{
			// check neighbours for hydrogen
			int neighborCount = neighbourCounts[flexInternalIndex];

			for(int ni = 0; ni < neighborCount; ++ni)
			{
				int neighborIndex = internalToAPI[neighbourIndices[ ni*stride + flexInternalIndex ]];

				FGraphNode& hydrogenNode = graph->node( neighborIndex );

				if(!hydrogenNode.hasComponent<FHydrogenGraphObject>())
					continue;

				FVector direction = hydrogenNode.position - atpSynthaseNode.position;

				bool isAbove = FVector::DotProduct( direction, atpSynthaseUp ) > 0;

				// hydrogen enters through the bottom of ATPSynthase, through it and out the top
				// so, only interact with hydrogen below
				if(isAbove)
					continue;

				if(direction.SizeSquared() > interactionRadiusSqrd)
					continue;

				// teleport hydrogen and spin
				// ---

				// setup spin state
				synthase.state = EATPSynthaseState::Spinning;

				synthase.timerSpin = synthase.timerSpinDuration;
				synthase.timerH = synthase.refractoryPeriodH;

				if( !atpSynthaseNode.hasComponent<FSpinnerGraphObject>() )
				{
					atpSynthaseNode.addComponent<FSpinnerGraphObject>(*graph);
				}

				FSpinnerGraphObject& spinner = atpSynthaseNode.component<FSpinnerGraphObject>(*graph);

				spinner.angularVelocityMagnitude = 5.0f;

				// teleport hydrogen
				hydrogenNode.position = atpSynthaseNode.position + atpSynthaseUp * 4.0;

				if(hydrogenNode.hasComponent<FVelocityGraphObject>())
					hydrogenNode.component<FVelocityGraphObject>( *graph ).linearVelocity = atpSynthaseUp * 4.0f;

				if(hydrogenNode.hasComponent<FRandomWalkGraphObject>())
					hydrogenNode.component<FRandomWalkGraphObject>( *graph ).timeLeft = 2.0f;

				break;
			}
		}

		// convert ADP to ATP
		if( synthase.state == EATPSynthaseState::Spinning && synthase.timerADP < 0.0f )
		{
			int neighborCount = neighbourCounts[flexInternalIndex];

			for(int ni = 0; ni < neighborCount; ++ni)
			{
				int adpIndex = internalToAPI[neighbourIndices[ni*stride + flexInternalIndex]];

				FGraphNode& adpNode = graph->node( adpIndex );

				if(adpNode.hasComponent<FADPGraphObject>() && adpNode.hasComponent<FFlexParticleObject>() )
				{
					FVector direction = adpNode.position - atpSynthaseNode.position;

					bool isAbove = FVector::DotProduct( direction, atpSynthaseUp ) > 0;

					if(!isAbove)
						continue;

					if(direction.SizeSquared() > interactionRadiusSqrd)
						continue;

					FGraphNode& atpNode = _spawnATP( adpNode.position, adpNode.orientation, adpNode.scale );

					// launch the atp out
					atpNode.addComponent<FVelocityGraphObject>( *graph ).linearVelocity = atpSynthaseUp * 3.0f;

					if(atpNode.hasComponent<FFlexParticleObject>())
					{
						FFlexParticleObject& atpParticle = atpNode.component<FFlexParticleObject>( *graph );
						FFlexParticleObject& adpParticle = adpNode.component<FFlexParticleObject>( *graph );

						atpParticle.channel = adpParticle.channel;
						atpParticle.group = adpParticle.group;
					}

					if(atpNode.hasComponent<FRandomWalkGraphObject>())
						atpNode.component<FRandomWalkGraphObject>( *graph ).timeLeft = 2.0f;

					// kill the adp
					graph->removeNode( adpIndex );

					synthase.timerADP = synthase.refractoryPeriodADP;

					break;
				}
			}
		}

		if(synthase.timerSpin < 0.0f)
		{
			synthase.state = EATPSynthaseState::Inactive;

			if(atpSynthaseNode.hasComponent<FSpinnerGraphObject>())
			{
				atpSynthaseNode.component<FSpinnerGraphObject>(*graph).angularVelocityMagnitude = 0.0f;
			}

			if(atpSynthaseNode.hasComponent<FVelocityGraphObject>())
			{
				atpSynthaseNode.component<FVelocityGraphObject>( *graph ).angularVelocity = FVector::ZeroVector;
			}
		}

		// update timers
		synthase.timerH -= deltaT;
		synthase.timerADP -= deltaT;
		synthase.timerSpin -= deltaT;
	}

	_tickHydrogenPumps( deltaT, neighbourIndices, neighbourCounts, apiToInternal, internalToAPI, maxParticles );

	graph->endTransaction();

	neighbourIndices.unmap();
	neighbourCounts.unmap();
	apiToInternal.unmap();
	internalToAPI.unmap();
}

void UATPSynthaseSimulation::_tickHydrogenPumps( 
	float deltaT,
	NvFlexVector<int>& neighbourIndices,
	NvFlexVector<int>& neighbourCounts,
	NvFlexVector<int>& apiToInternal,
	NvFlexVector<int>& internalToAPI,
	int maxParticles )
{
	auto& protonPumps = graph->componentStorage<FProtonPumpGraphObject>();

	const int stride = maxParticles;

	for(FProtonPumpGraphObject& pump : protonPumps)
	{
		auto nodeIndex = pump.nodeIndex;
		FGraphNode& pumpNode = graph->node( nodeIndex );

		if(!pumpNode.hasComponent<FFlexParticleObject>())
			continue;

		FVector pumpUp = FVector::UpVector;
		pumpUp = pumpNode.orientation.RotateVector( pumpUp );

		const float interactionRadiusSqrd = pump.hyrogenInteractionRadius * pump.hyrogenInteractionRadius;

		int flexInternalIndex = apiToInternal[nodeIndex];

		// use hydrogen to spin
		if(pump.timerH < 0.0f)
		{
			// check neighbours for hydrogen
			int neighborCount = neighbourCounts[flexInternalIndex];

			for(int ni = 0; ni < neighborCount; ++ni)
			{
				int neighborIndex = internalToAPI[neighbourIndices[ni*stride + flexInternalIndex]];

				FGraphNode& hydrogenNode = graph->node( neighborIndex );

				if(!hydrogenNode.hasComponent<FHydrogenGraphObject>())
					continue;

				FVector direction = hydrogenNode.position - pumpNode.position;

				bool isAbove = FVector::DotProduct( direction, pumpUp ) > 0;

				// hydrogen through the top
				if(!isAbove)
					continue;

				if(direction.SizeSquared() > interactionRadiusSqrd)
					continue;

				// teleport hydrogen
				hydrogenNode.position = pumpNode.position - pumpUp * 4.0;

				if(hydrogenNode.hasComponent<FVelocityGraphObject>())
					hydrogenNode.component<FVelocityGraphObject>( *graph ).linearVelocity = -pumpUp * 4.0f;

				if(hydrogenNode.hasComponent<FRandomWalkGraphObject>())
					hydrogenNode.component<FRandomWalkGraphObject>( *graph ).timeLeft = 2.0f;

				pump.timerH = pump.refractoryPeriodH;

				break;
			}
		}

		// update timers
		pump.timerH -= deltaT;
	}
}

FGraphNode& UATPSynthaseSimulation::_spawnATP( FVector position, FQuat orientation, float scale )
{
	FGraphNode& node = graph->node(graph->addNode( position, orientation, scale ));

	// spawn our components
	for(FTimStructBox& box : atpTemplate)
	{
		if(!box.IsValid())
			continue;

		// create the graph object in the simulation
		ComponentType type = FGraphObject::componentType( box.scriptStruct );

		FGraphObject * object = node.addComponent( *graph, type );

		// then copy the memory from the element
		box.scriptStruct->CopyScriptStruct( object, box.structMemory );

		object->nodeIndex = node.id;
	}

	return node;
}
