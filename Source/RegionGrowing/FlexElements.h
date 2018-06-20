// Copyright (c) 2018 Timothy Davison. All rights reserved.

#pragma once

#include "Components/ActorComponent.h"

#include "Flex/include/NvFlex.h"
#include "Flex/include/NvFlexExt.h"
#include "Flex/include/NvFlexDevice.h"

#include "ShipEditorSimulation/Graph.h"

#include "InstanceManager.h"
#include "RegionGrowingComponent.h"

#include "FlexElements.generated.h"

USTRUCT( BlueprintType )
struct REGIONGROWING_API FCachedElementGraphObject : public FGraphObject
{
	GENERATED_BODY()

public:
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	FElement element;
};

UENUM( BlueprintType )
enum class EATPSynthaseState : uint8
{
	Spinning UMETA( DisplayName = "Active" ),
	Inactive UMETA( DisplayName = "Inactive" ),
};

USTRUCT( BlueprintType )
struct REGIONGROWING_API FProtonPumpGraphObject : public FGraphObject
{
	GENERATED_BODY()

public:
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	float timerH = -1.0f;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	float refractoryPeriodH = 1.0f;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	float hyrogenInteractionRadius = 5.0f;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	EATPSynthaseState state = EATPSynthaseState::Inactive;
};

USTRUCT( BlueprintType )
struct REGIONGROWING_API FATPSynthaseGraphObject : public FGraphObject
{
	GENERATED_BODY()

public:
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	float timerH = -1.0f;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	float refractoryPeriodH = 1.0f;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	float timerADP = -1.0f;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	float timerSpin = 0.0f;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	float timerSpinDuration = 2.0f;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	float refractoryPeriodADP = 1.0f;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	float hyrogenInteractionRadius = 5.0f;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	EATPSynthaseState state = EATPSynthaseState::Inactive;
};

USTRUCT( BlueprintType )
struct REGIONGROWING_API FFlexParticleObject : public FGraphObject
{
	GENERATED_BODY()

public:
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	int group;

	int channel;
};

USTRUCT( BlueprintType )
struct REGIONGROWING_API FHydrogenGraphObject : public FGraphObject
{
	GENERATED_BODY()

};

USTRUCT( BlueprintType )
struct REGIONGROWING_API FADPGraphObject : public FGraphObject
{
	GENERATED_BODY()

};

USTRUCT( BlueprintType )
struct REGIONGROWING_API FATPGraphObject : public FGraphObject
{
	GENERATED_BODY()

};

UCLASS( BlueprintType )
class REGIONGROWING_API UATPSynthaseSimulation : public UObjectSimulation
{
	GENERATED_BODY()
public:
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	TArray<FTimStructBox> atpTemplate;

public:
	virtual void init();

	virtual void tick( float deltaT ) override;

	void flexTick( float deltaT, NvFlexVector<int>& neighbourIndices, NvFlexVector<int>& neighbourCounts, NvFlexVector<int>& apiToInternal, NvFlexVector<int>& internalToAPI, int maxParticles );

	std::shared_ptr<tcodsMeshInterface> meshInterface;

	FTransform toWorld;

protected:
	void _tickHydrogenPumps( float deltaT, NvFlexVector<int>& neighbourIndices, NvFlexVector<int>& neighbourCounts, NvFlexVector<int>& apiToInternal, NvFlexVector<int>& internalToAPI, int maxParticles );

	FGraphNode& _spawnATP( FVector position, FQuat orientation, float scale );
};


USTRUCT( BlueprintType )
struct REGIONGROWING_API FSpinnerGraphObject : public FGraphObject
{
	GENERATED_BODY()

public:
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	float angularVelocityMagnitude = 1.0f;
};


USTRUCT( BlueprintType )
struct REGIONGROWING_API FVelocityGraphObject : public FGraphObject
{
	GENERATED_BODY()

public:
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	FVector linearVelocity = FVector::ZeroVector;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	FVector angularVelocity = FVector::ZeroVector;
};

USTRUCT( BlueprintType )
struct REGIONGROWING_API FStaticPositionObject : public FGraphObject
{
	GENERATED_BODY()

public:
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	FVector position = FVector::ZeroVector;
};

USTRUCT( BlueprintType )
struct REGIONGROWING_API FRandomWalkGraphObject : public FGraphObject
{
	GENERATED_BODY()

public:
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	float timeLeft = 0.0f;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	float maxVelocityOffset = 1.0f;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	float baseVelocity = 1.0f;
};

USTRUCT( BlueprintType )
struct REGIONGROWING_API FSurfaceBoundGraphObject : public FGraphObject
{
	GENERATED_BODY()

public:
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	FQuat lastSurfaceRotation;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	int32 faceIndex = -1;
};

UCLASS()
class REGIONGROWING_API URandomWalkSimulation : public UObjectSimulation
{
	GENERATED_BODY()
public:
	virtual void init();

	virtual void tick( float deltaT ) override;

	std::shared_ptr<tcodsMeshInterface> meshInterface;

	FTransform toWorld;
};

USTRUCT( BlueprintType )
struct FNvFlexParameters
{
	GENERATED_BODY()

public:
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	int numIterations = 3;					//!< Number of solver iterations to perform per-substep

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	FVector gravity = FVector(0.0f,0.0f,-9.81f);					//!< Constant acceleration applied to all particles
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float radius = 1.0f;						//!< The maximum interaction radius for particles
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float solidRestDistance = 0.0f;			//!< The distance non-fluid particles attempt to maintain from each other, must be in the range (0, radius]
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float fluidRestDistance = 0.5f;			//!< The distance fluid particles are spaced at the rest density, must be in the range (0, radius], for fluids this should generally be 50-70% of mRadius, for rigids this can simply be the same as the particle radius

	// common params
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float dynamicFriction = 0.0f;				//!< Coefficient of friction used when colliding against shapes
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float staticFriction = 0.0f;				//!< Coefficient of static friction used when colliding against shapes
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float particleFriction = 0.0f;				//!< Coefficient of friction used when colliding particles
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float restitution = 0.0f;					//!< Coefficient of restitution used when colliding against shapes, particle collisions are always inelastic
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float adhesion = 0.0f;						//!< Controls how strongly particles stick to surfaces they hit, default 0.0, range [0.0, +inf]
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float sleepThreshold = 0.0f;				//!< Particles with a velocity magnitude < this threshold will be considered fixed
	
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float maxSpeed = FLT_MAX;						//!< The magnitude of particle velocity will be clamped to this value at the end of each step
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float maxAcceleration = 100.0f;				//!< The magnitude of particle acceleration will be clamped to this value at the end of each step (limits max velocity change per-second), useful to avoid popping due to large interpenetrations
	
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float shockPropagation = 0.0f;				//!< Artificially decrease the mass of particles based on height from a fixed reference point, this makes stacks and piles converge faster
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float dissipation = 0.0f;					//!< Damps particle velocity based on how many particle contacts it has
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float damping = 0.0f;						//!< Viscous drag force, applies a force proportional, and opposite to the particle velocity

	// cloth params
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	FVector wind = FVector::ZeroVector;						//!< Constant acceleration applied to particles that belong to dynamic triangles, drag needs to be > 0 for wind to affect triangles
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float drag  = 0.0f;							//!< Drag force applied to particles belonging to dynamic triangles, proportional to velocity^2*area in the negative velocity direction
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float lift = 0.0f;							//!< Lift force applied to particles belonging to dynamic triangles, proportional to velocity^2*area in the direction perpendicular to velocity and (if possible), parallel to the plane normal

	// fluid params
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float cohesion = 0.25f;						//!< Control how strongly particles hold each other together, default: 0.025, range [0.0, +inf]
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float surfaceTension = 0.01f;				//!< Controls how strongly particles attempt to minimize surface area, default: 0.0, range: [0.0, +inf]    
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float viscosity = 0.1f;					//!< Smoothes particle velocities using XSPH viscosity
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float vorticityConfinement = 40.0f;			//!< Increases vorticity by applying rotational forces to particles
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float anisotropyScale = 20.0f;				//!< Control how much anisotropy is present in resulting ellipsoids for rendering, if zero then anisotropy will not be calculated, see NvFlexGetAnisotropy()
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float anisotropyMin = 0.1f;				//!< Clamp the anisotropy scale to this fraction of the radius
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float anisotropyMax = 2.0f;				//!< Clamp the anisotropy scale to this fraction of the radius
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float smoothing = 1.0f;					//!< Control the strength of Laplacian smoothing in particles for rendering, if zero then smoothed positions will not be calculated, see NvFlexGetSmoothParticles()
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float solidPressure = 1.0f;				//!< Add pressure from solid surfaces to particles
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float freeSurfaceDrag = 0.0f;				//!< Drag force applied to boundary fluid particles
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float buoyancy = 1.0f;						//!< Gravity is scaled by this value for fluid particles

	// diffuse params
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float diffuseThreshold = 100.0f;				//!< Particles with kinetic energy + divergence above this threshold will spawn new diffuse particles
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float diffuseBuoyancy = 1.0f;				//!< Scales force opposing gravity that diffuse particles receive
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float diffuseDrag = 0.8f;					//!< Scales force diffuse particles receive in direction of neighbor fluid particles
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	int diffuseBallistic = 16;				//!< The number of neighbors below which a diffuse particle is considered ballistic
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float diffuseLifetime = 2.0f;				//!< Time in seconds that a diffuse particle will live for after being spawned, particles will be spawned with a random lifetime in the range [0, diffuseLifetime]

	// collision params
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float collisionDistance = 0.0f;			//!< Distance particles maintain against shapes, note that for robust collision against triangle meshes this distance should be greater than zero
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float particleCollisionMargin = 0.0f;		//!< Increases the radius used during neighbor finding, this is useful if particles are expected to move significantly during a single step to ensure contacts aren't missed on subsequent iterations
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float shapeCollisionMargin = 0.0f;			//!< Increases the radius used during contact finding against kinematic shapes

	UPROPERTY( EditAnywhere,  Category = "Flex" )
	FVector4 planes[8];					//!< Collision planes in the form ax + by + cz + d = 0
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	int numPlanes = 0;						//!< Num collision planes

	//NvFlexRelaxationMode relaxationMode;//!< How the relaxation is applied inside the solver

	// 0 - eNvFlexRelaxationGlobal
	// 1 - eNvFlexRelaxationLocal
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	int relaxationMode = 1;//!< How the relaxation is applied inside the solver

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Flex" )
	float relaxationFactor = 1.0f;	

	NvFlexParams toNvFlexParams()
	{
		NvFlexParams params;

		params.numIterations = numIterations;

		params.gravity[0] = gravity.X;
		params.gravity[1] = gravity.Y;
		params.gravity[2] = gravity.Z;
		
		params.radius = radius;
		params.solidRestDistance = solidRestDistance;
		params.fluidRestDistance = fluidRestDistance;

		// common params
		params.dynamicFriction = dynamicFriction;
		params.staticFriction = staticFriction;
		params.particleFriction = particleFriction;
		params.restitution = restitution;
		params.adhesion = adhesion;
		params.sleepThreshold = sleepThreshold;

		params.maxSpeed = maxSpeed;
		params.maxAcceleration = maxAcceleration;

		params.shockPropagation = shockPropagation;
		params.dissipation = dissipation;
		params.damping = damping;

		// cloth params
		params.wind[0] = wind.X;
		params.wind[1] = wind.Y;
		params.wind[2] = wind.Z;

		params.drag = drag;
		params.lift = lift;

		// fluid params
		params.cohesion = cohesion;
		params.surfaceTension = surfaceTension;
		params.viscosity = viscosity; 
		params.vorticityConfinement = vorticityConfinement;
		params.anisotropyScale = anisotropyScale;
		params.anisotropyMin = anisotropyMin;
		params.anisotropyMax = anisotropyMax;
		params.smoothing = smoothing;
		params.solidPressure = solidPressure;
		params.freeSurfaceDrag = freeSurfaceDrag;
		params.buoyancy = buoyancy;

		// diffuse params
		params.diffuseThreshold = diffuseThreshold;
		params.diffuseBuoyancy = diffuseBuoyancy;
		params.diffuseDrag = diffuseDrag;
		params.diffuseBallistic = diffuseBallistic;
		params.diffuseLifetime = diffuseLifetime;

		// collision params
		params.collisionDistance = collisionDistance;
		params.particleCollisionMargin = particleCollisionMargin;
		params.shapeCollisionMargin = shapeCollisionMargin;

		for( int i = 0; i < 8; ++i )
		{
			for(int j = 0; j < 4; ++j)
				params.planes[i][j] = planes[i][j];
		}
		
		params.numPlanes = numPlanes;

		params.relaxationMode = relaxationMode == 0 ? NvFlexRelaxationMode::eNvFlexRelaxationGlobal : NvFlexRelaxationMode::eNvFlexRelaxationLocal;
		params.relaxationFactor = relaxationFactor;

		return params;	
	}
};

UCLASS( ClassGroup = (Custom), meta = (BlueprintSpawnableComponent) )
class REGIONGROWING_API UFlexElements : public UActorComponent, public ComponentListener
{
	GENERATED_BODY()

public:
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	FNvFlexParameters flexParams;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Mitochondria" )
	FGraph graphSimulation;

public:
	UFlexElements();

	virtual void TickComponent( UPROPERTY() float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction ) override;


	virtual void BeginDestroy() override;

	virtual void InitializeComponent();

	virtual void BeginPlay();

	auto clear() -> void;
	auto pause() -> void;
	auto play() -> void;

	auto updateSphereWorldSpace( FVector position ) -> void;

	auto loadExportedDomainInfo( OutputDomainExport& exportInfo ) -> void;
	auto exportElementDomain() -> std::vector<FElement>;

	auto setInstanceManagerBounds( FBox instanceManagerRelativeBounds ) -> void;

protected:
	auto _initFlex() -> void;

	auto _initGraphSimulationsAndControllers() -> void;

	auto _initParams() -> void;

	auto _uninit() -> void;

	void _tickGraph( float DeltaTime );

	auto _instanceCount()->size_t;

	auto _spawnDespawnParticles( FVector4 * positions, FVector * velocities, int * phases, int * active ) -> void;
	auto _spawnShapes( NvFlexCollisionGeometry * geometry, FVector4 * shapePositions, FQuat * shapeRotations, int * shapeFlags ) -> void;
	auto _spawnMesh( NvFlexCollisionGeometry * geometry, FVector4 * shapePositions, FQuat * shapeRotations, int * shapeFlags ) -> void;

	auto _readFlexState( FVector4 * positions, FVector * velocities, int * phases ) -> void;
	auto _writeFlexState( FVector4 * positions, FVector * velocities, int * phases, int * active ) -> void;

	auto _integrateRotations(float deltaT) -> void;
	
	auto _loadMesh( UStaticMeshComponent * meshComponent ) -> void;

protected: // ComponentListener
	virtual void componentAdded( FGraphNodeHandle node, ComponentType type ) override;
	virtual void componentRemoved( FGraphNodeHandle node, ComponentType type ) override;
	virtual void componentUpdated( FGraphNodeHandle node, ComponentType type ) override;
	virtual void connectionAdded( int32 edgeIndex, FGraphEdge& edge, ComponentType type ) override;
	virtual void connectionRemoved( int32 edgeIndex, FGraphEdge& oldEdge, ComponentType type ) override;

protected:
	NvFlexLibrary * _library = nullptr;

	NvFlexSolverDesc _solverDescription;
	NvFlexParams _params;

	NvFlexSolver * _solver = nullptr;

	NvFlexBuffer * _particleBuffer = nullptr;
	NvFlexBuffer * _velocityBuffer = nullptr;
	NvFlexBuffer * _phaseBuffer = nullptr;
	NvFlexBuffer * _activeBuffer = nullptr;

	NvFlexBuffer * _geometryBuffer = nullptr;
	NvFlexBuffer * _shapePositionsBuffer = nullptr;
	NvFlexBuffer * _shapeRotationsBuffer = nullptr;
	NvFlexBuffer * _shapeFlagsBuffer = nullptr;

	std::unique_ptr< NvFlexVector<int> > _neighborsIndicesBuffer;
	std::unique_ptr< NvFlexVector<int> > _neighborsCountsBuffer;
	std::unique_ptr< NvFlexVector<int> > _neighborsAPIToInternal;
	std::unique_ptr< NvFlexVector<int> > _neighborsInternalToAPI;



	std::unique_ptr< NvFlexVector<FVector4> > _mesh0_positions;
	std::unique_ptr< NvFlexVector<int> > _mesh0_indices;
	NvFlexTriangleMeshId _mesh0_Id;
	FBox _mesh0_bounds;

	bool _needsFlexReset = false;
	bool _didInitFlex = false;
	bool _playing = false;

	bool _didInitGraphSimulation = false;

	InstanceManager _instanceManager;
	bool _didSpawnShapesAndMesh = false;

	FVector _spherePosition = FVector::ZeroVector;

	const size_t _maxParticles = 80000;
	const size_t _maxNeighbors = 32;

	size_t nParticles = 0;

	const size_t nGeometries = 2;

	UStaticMeshComponent * _meshComponent = nullptr;

	std::shared_ptr<tcodsMeshInterface> _meshInterface;

	std::set<FGraphNodeHandle> _flexParticlesToAdd;
	std::set<FGraphNodeHandle> _flexParticlesToRemove;

	ComponentType _FFlexParticleObjectType;
};

UCLASS( ClassGroup = (Custom), meta = (BlueprintSpawnableComponent) )
class REGIONGROWING_API UFlexInstanceStaticMeshComponent : public UInstancedStaticMeshComponent
{
	GENERATED_BODY()

public:
	UPROPERTY( EditAnywhere ) int32 index = 0;
};

