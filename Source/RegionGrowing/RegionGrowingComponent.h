// Copyright (c) 2017 Timothy Davison. All rights reserved.

#pragma once

#include "Components/ActorComponent.h"
#include "Algorithm/Algorithm.h"
#include "Algorithm/Algorithm_RegionGrowing.h"
#include "Algorithm/Algorithm_Clustered.h"
#include "Algorithm/Algorithm_PatchCopy.h"
#include "Algorithm/ElementKDTree.h"

#include "Algorithm/ctpl_stl.h"

#include <map>
#include <memory>
#include <ctime>

#include "tcodsMeshInterface.h"
#include "DepthReader.h"
#include "AlgorithmEnums.h"
#include "InstanceManager.h"
#include "NeighbourhoodParameters.h"

#include "RegionGrowingComponent.generated.h"

class URuntimeMeshComponent;
class AElementActor;

namespace tcods { class Mesh; }

static const FName TCODSMarker = FName(TEXT("TCODSMarker"));

struct OutputDomainExport
{
	struct ElementAndTransform
	{
		FElement * element;

		FTransform transform;
	};

	struct MeshInformation
	{
		UStaticMesh * mesh;
		TArray<UMaterialInterface*> materials;
	};

	void addElement( FElement * element, FTransform transform )
	{
		_elementsAndTransforms.emplace_back();
	
		ElementAndTransform& pair = _elementsAndTransforms.back();
		pair.element = element;
		pair.transform = transform;
	}

	void setMeshInfo( int16 elementType, UStaticMesh* staticMesh, TArray<UMaterialInterface*> materials )
	{
		MeshInformation info;

		info.mesh = staticMesh;
		info.materials = materials;
		
		if(_meshInfo.find( elementType ) == _meshInfo.end())
			_meshInfo[elementType] = info;
	}

	std::vector<ElementAndTransform> elementsAndTransforms() 
	{
		return _elementsAndTransforms; 
	}

	std::unordered_map<int16, MeshInformation> meshInfo()
	{
		return _meshInfo;
	}

	UStaticMeshComponent * staticMeshComponent = nullptr;

	std::shared_ptr<tcodsMeshInterface> meshInterface;


	std::vector<ElementAndTransform> _elementsAndTransforms;
	std::unordered_map<int16, MeshInformation> _meshInfo;
};


USTRUCT()
struct FSingularity
{
    GENERATED_USTRUCT_BODY()
    
    FSingularity() {}
    FSingularity(uint32 vertexIndex_, float k_) { vertexIndex = vertexIndex_; k = k_; }
    FSingularity(const FSingularity& other) { vertexIndex = other.vertexIndex; k = other.k; }
    
    UPROPERTY(EditAnywhere) uint32 vertexIndex;
    UPROPERTY(EditAnywhere) float k;
};



UENUM( BlueprintType )
enum class EAlgorithm : uint8
{
	RegionGrowing UMETA(DisplayName="Region Growing"),
	ClusterGeneration UMETA(DisplayName="Cluster Generation"),
	PatchCopy UMETA(DisplayName="Patch Copy"),
};

class RGOcclusionTester : public OcclusionBase
{
public:
	virtual ~RGOcclusionTester() {}

	virtual bool isVisible( PositionFace point, float radius ) override;


public:
    UDepthReader * reader = nullptr;
    FVector cameraLocation;
	std::shared_ptr<tcodsMeshInterface> _meshInterface = nullptr;
};



USTRUCT()
struct FElementType
{
	GENERATED_BODY()

public:
	FElementType() {}

	FElementType( AActor * actorTemplate )
	{

	}

	FElementType( UStaticMesh * mesh, UMaterialInterface * material )
	{

	}

	UPROPERTY() UStaticMesh * mesh = nullptr;
	UPROPERTY() UMaterialInterface * material = nullptr;

	UPROPERTY() int16 integerType = 0;



	struct InferredElementComparator
	{
		bool operator()( const FElementType& a, const FElementType& b ) const
		{
			if(a.mesh != b.mesh)
				return a.mesh < b.mesh;

			if(a.material != b.material)
				return a.material < b.material;

			return false;
		}
	};
};



USTRUCT()
struct FElementTypeLookup
{
	GENERATED_BODY()

public:
	int16 getOrGenerateType( AActor * actor )
	{
		FElementType query( actor );
		
		auto found = typeSet.find( query );

		if(found == typeSet.end())
		{
			auto type = elementTypes.Num();

			query.integerType = type;

			typeSet.insert( query );

			return type;
		}
		else
			return found->integerType;
	} 

	void PostSerialize( const FArchive& Ar )
	{
		for(auto type : elementTypes)
			typeSet.insert(type);
	}

protected: 
	std::set<FElementType, FElementType::InferredElementComparator> typeSet;

	UPROPERTY() TArray<FElementType> elementTypes;
};

template<>
struct TStructOpsTypeTraits<FElementTypeLookup> : public TStructOpsTypeTraitsBase2<FElementTypeLookup>
{
	enum
	{
		WithPostSerialize = true,
	};
};

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class REGIONGROWING_API URegionGrowingComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UPROPERTY( EditAnywhere ) TArray<FElement> archivedElements;

	UPROPERTY( EditAnywhere ) bool perElementParameters = false;


    UPROPERTY( EditAnywhere ) bool pauseSynthesis = false;

	UPROPERTY( EditAnywhere ) bool enableRoundSummaries = false;
	UPROPERTY( EditAnywhere ) bool calculate_kCoherenceEnergy = false;
	UPROPERTY( EditAnywhere ) bool calculate_bruteForceEnergy = false;
	UPROPERTY( EditAnywhere ) bool useCinpactEnergy = false;
	UPROPERTY( EditAnywhere ) float cinpactCellSize = 1.0f;


	// The duration at which to pause synthesis. If this is negative, it is ignored.
	UPROPERTY( EditAnywhere ) float stopDuration = -1.0f;

	// A value of 1.0 will not relax overlaps
	// A value < 1.0 will increase the amount of overlap allowed
	// The comparison in Algorithm::_overlaps is something like
	//     bool overlaps = e.position - ex.position < (e.radius + ex.radius) * relaxation;
	UPROPERTY( EditAnywhere ) float relaxation = 0.8f;




    UPROPERTY(EditAnywhere) FNeighbourhoodParameters generationParameters; // search parameters
    UPROPERTY(EditAnywhere) FNeighbourhoodParameters optimizationParameters;

	UPROPERTY( EditAnywhere ) bool useTypeVoting = true;
	UPROPERTY( EditAnywhere ) bool removeOverlaps = false;

    UPROPERTY(EditAnywhere) float generationRadius; // the radius out to which elements are generated around the seeds
	UPROPERTY(EditAnywhere) float frozenElementRadius = 0.0f;

    UPROPERTY(EditAnywhere) int32 kCoherence = 5;

	UPROPERTY(EditAnywhere) int32 nThreads = 2;
    
    UPROPERTY(EditAnywhere) EGenerationMode generationMode;	

	UPROPERTY(EditAnywhere) EAlgorithm algorithmType = EAlgorithm::RegionGrowing;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite) AActor * exemplar = nullptr;
	UPROPERTY(EditAnywhere) AActor * targetExemplar = nullptr;

    UPROPERTY(EditAnywhere) bool multiSampleEntities = true;
    
    UPROPERTY(EditAnywhere) float brushSize = 30.0f;

	UPROPERTY(EditAnywhere) float selectionRadius = 30.0f;

	UPROPERTY(EditAnywhere) bool enableVolumeSurfaceInteraction = true;


	UPROPERTY(EditAnywhere) float voidSize = 10.0f; // the size of the void brush (the void brush prevents elements from being generated in the painted regions)
    
    UPROPERTY(EditAnywhere) float hideSurfaceDistance = 0.0f;
	UPROPERTY(EditAnywhere) float resetDistance = 0.0f;

    // Synthesize elements when they are synthesizeDistance from the camera.
    // Use a negative number to synthesize to infinity on manual stepping.
    UPROPERTY(EditAnywhere) float synthesizeDistance;
    
    UPROPERTY(EditAnywhere) float innerQuellingDistance = 0.0f;
	UPROPERTY(EditAnywhere) float outerQuellingDistance = 0.0f;
    
    UPROPERTY(EditAnywhere) FVector generationLimitsMin = {-1000.0f, -1000.0f, -1000.0f};
    UPROPERTY(EditAnywhere) FVector generationLimitsMax = { 1000.0f,  1000.0f,  1000.0f};
    
    // Used in the closest point assignment, when points are too far from each other
    // then they are not paired
    UPROPERTY(EditAnywhere) float minAssignmentDistance = 50.0f;
    UPROPERTY(EditAnywhere) float freespaceRadius = 15.0f;
    UPROPERTY(EditAnywhere) float typeCost = 30.0f;
	UPROPERTY(EditAnywhere) float gradientTerm = 1.0f;

	UPROPERTY(EditAnywhere) float kCoherenceClusteringPenalty = 0.0f;
	UPROPERTY(EditAnywhere) float kCoherenceClusteringPenaltyRadius = 5.0f;


    

    
    // Place the elements slightly offset from the surface along the normal:
    // p' = p + n_p * grow
    // This parameter does not affect the region growing algorithm
    UPROPERTY(EditAnywhere) float grow = 0.0f;
    
    UPROPERTY(EditAnywhere) bool ignoreBadSuggestions = false;
	UPROPERTY(EditAnywhere) float ignoreBadSuggestionsDistanceFactor = 10.0f;

    UPROPERTY(EditAnywhere) int32 batchSize = 1;
    
    UPROPERTY(EditAnywhere) int32 sampleSubdivisions = 64;
    UPROPERTY(EditAnywhere) bool flipSurfaceNormals = false;

	UPROPERTY( EditAnywhere ) bool forceRotation = false;
	UPROPERTY( EditAnywhere ) FQuat forcedRotation = FQuat::Identity;
    
    UPROPERTY(EditAnywhere) float sourceHistogramRadius = 0.0f;
    UPROPERTY(EditAnywhere) float sourceHistogramWeight = 0.0f;
    UPROPERTY(EditAnywhere) float samplingDistanceWeight = 0.0f;
    
    UPROPERTY( EditAnywhere ) float stretchRadius = 100.0f;
    
    UPROPERTY( EditAnywhere ) bool rayCastSeeds = true;
    UPROPERTY( EditAnywhere ) float rayCastDivisions = 9.0f;

	UPROPERTY( EditAnywhere ) bool sketchBasedForceTangentPlane = false;

	UPROPERTY( EditAnywhere ) FIntVector patchInitializationExemplarDivisions = FIntVector( 1, 1, 1 ); 
	// The percentage of the calculate patch extents to over-copy elements (leads to overlaps in the margins around patches, which is necessary for some exemplars).
	UPROPERTY( EditAnywhere ) float patchMarginPercentage = 0.0f;

    UPROPERTY(EditAnywhere) AActor * depthReaderActor = nullptr;
    UDepthReader * depthReader()
    {
        if( !depthReaderActor )
            return nullptr;
        
        return (UDepthReader*)depthReaderActor->GetComponentByClass(UDepthReader::StaticClass());
    }
    
    // singularities and seeds are the in the coordinate space of the untransformed mesh
    UPROPERTY(EditAnywhere) TArray< FSingularity > singularities;

	UPROPERTY( EditAnywhere ) bool useSeeds = false;
	UPROPERTY( EditAnywhere ) bool seedsIgnoreMeshInBoundary = false;
    UPROPERTY(EditAnywhere) TArray< FVector > seeds;
    
    UPROPERTY(EditAnywhere) AActor * _output = nullptr;
    AActor * output()
    {
        if( _output == nullptr )
        {
            _output = GetWorld()->SpawnActor<class AActor>(this->GetOwner()->GetClass(), FVector::ZeroVector, FRotator::ZeroRotator, FActorSpawnParameters());
            _output->SetRootComponent(NewObject<USceneComponent>(_output));
            _output->GetRootComponent()->SetMobility(EComponentMobility::Static);
			_output->AttachToActor( this->GetOwner(), FAttachmentTransformRules::KeepRelativeTransform );
#if WITH_EDITOR
            _output->SetActorLabel(FString(TEXT("Output")));
#endif
        }
        
        return _output;
    }
	UPROPERTY(EditAnywhere) bool buildTrivialConnections = true;

    
    UPROPERTY(EditAnywhere) bool drawDistanceField = false;
    UPROPERTY(EditAnywhere) bool drawHitSamples = false;
    UPROPERTY(EditAnywhere) bool drawNearest = false;
	UPROPERTY(EditAnywhere) bool drawSurface = false;
	UPROPERTY(EditAnywhere) bool drawHalfEdge = false;
	UPROPERTY(EditAnywhere) bool drawBounds = false;

	UPROPERTY( EditAnywhere ) bool disableOptimization = false;
	UPROPERTY( EditAnywhere ) int optimizationRounds = 1;
	UPROPERTY( EditAnywhere ) bool useGlobalOptimization = false;

    UPROPERTY() TArray<UStaticMeshComponent*> singularityDebugSpheres;
    UPROPERTY() TArray<UStaticMeshComponent*> seedDebugSpheres;
    
    UPROPERTY(EditAnywhere) bool debugHorizon = false;
    UPROPERTY(EditAnywhere) UMaterialInstance * debugHorizonMaterial = nullptr;

	UPROPERTY( EditAnywhere ) UMaterialInterface * directionFieldMaterial = nullptr;
	UPROPERTY( EditAnywhere ) UMaterialInterface * debugSurfaceMaterial = nullptr;

    

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly) int32 statElementCount = 0;
    
    UStaticMeshComponent * staticMeshComponent();
	URuntimeMeshComponent * debugMeshComponent(); // for debug drawing
    
    URuntimeMeshComponent * runtimeMesh = nullptr;


    
	// Sets default values for this component's properties
	URegionGrowingComponent();

	// Called when the game starts
	virtual void InitializeComponent() override;
    
	virtual void BeginDestroy() override;
	
	// Called every frame
	virtual void TickComponent( float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction ) override;

	void _tickVisibility();

	virtual void SetActive( bool newActive, bool reset = false)
	{
		Super::SetActive( newActive, reset );

		if(newActive == false && reset == true)
		{
			LoadExemplar();
		}
	}

    UFUNCTION(BlueprintCallable, Category=Generation) void LoadExemplar();
    UFUNCTION(BlueprintCallable, Category=Generation) void Generate();
    UFUNCTION(BlueprintCallable, Category=Generation) void GlobalOptimization();
	UFUNCTION(BlueprintCallable, Category=Generation) void ClearOutput();
	UFUNCTION(BlueprintCallable, Category=Generation) void ClearElementsKeepPaths();

	UFUNCTION( BlueprintCallable, Category = Generation ) void OutputStats();
	UFUNCTION( BlueprintCallable, Category = Generation ) void OutputSpaceSeparatedStats();
	UFUNCTION( BlueprintCallable, Category = Generation ) void OutputParameters();



#if WITH_EDITOR
    void PostEditChangeProperty(struct FPropertyChangedEvent& e) override;
#endif    

    void addSingularity(const FVector& vertex, float k);
    void removeSingularityAt(unsigned int index);
    
    void addSeed(const FVector& localPoint);
    void removeSeedAt(unsigned int index);
    
    bool isMeshInterfaceReady() { return _didInit; };
    std::shared_ptr<tcodsMeshInterface> meshInterface() { return _meshInterface; };

    class ASimulationSnapshotActor * createSimulationSnapshot();
    
	OutputDomainExport exportOutputDomain();
	void loadElementDomain( std::vector<FElement> newElements );

	void setElementVisibility( bool visible );

    // ----------------------------------------------------------
    // Stretching
    // ----------------------------------------------------------
    void startStretch( FVector point );
    void updateStretch( FVector point );
    void endStretch( FVector point );
    
    // ----------------------------------------------------------
    // Exemplar Painting
    // ----------------------------------------------------------
    void startPaint();
    void addBrushPoint( FVector point, int32 face = -1 );
	void addBrushPointWithRadius( FVector point, float radius, int32 face = -1 );
    void endPaint();

	void eraseAt( FVector point, int32 face = -1 );
	void eraseInRadiusAt( FVector point, float radius, int32 face = -1 );


	void startVoid();
	void addVoid( FVector point, int32 face = -1 );
	void endVoid();

	Algorithm::ExampleSelectionPtr addExampleSelection( std::vector<AElementActor*> newSelection, float weight );
	void updateExampleSelection( Algorithm::ExampleSelectionPtr exampleSelectionHandle, std::vector<AElementActor*> newSelection, float weight );
	void removeExampleSelection( Algorithm::ExampleSelectionPtr exampleSelectionHandle );

	void generateExemplar();




	FVector toExemplar( FVector worldSpace );
	FVector toWorld( FVector exemplarSpace );

	InstanceManager * getInstanceManager() { return &_instanceManager; }
    
	// Subclasser's API
public:
	virtual Algorithm* getOrCreateAlgorithm();

	virtual void loadParameters();

protected:
	virtual void initAlgorithms();

public:
	struct ElementTypeDescription
	{
		ElementTypeDescription( AElementActor * fromActor )
		{
			UStaticMeshComponent * staticMeshComponent = fromActor->GetStaticMeshComponent();

			if(!staticMeshComponent)
				return;

			mesh = staticMeshComponent->GetStaticMesh();

			for(int i = 0; i < staticMeshComponent->GetNumMaterials(); ++i)
				materials.Add( staticMeshComponent->GetMaterial( i ) );

			if(staticMeshComponent->GetNumMaterials())
				material = staticMeshComponent->GetMaterial( 0 );
		}

		ElementTypeDescription( AActor * fromActor )
		{
			UStaticMeshComponent * staticMeshComponent = fromActor->FindComponentByClass<UStaticMeshComponent>();

			if(!staticMeshComponent)
				return;

			mesh = staticMeshComponent->GetStaticMesh();

			for(int i = 0; i < staticMeshComponent->GetNumMaterials(); ++i)
				materials.Add( staticMeshComponent->GetMaterial( i ) );

			if( staticMeshComponent->GetNumMaterials() )
				material = staticMeshComponent->GetMaterial( 0 );
		}

		ElementTypeDescription( const ElementTypeDescription& other )
		{
			mesh = other.mesh;
			material = other.material;
			materials = other.materials;
		}

		ElementTypeDescription()
		{
			mesh = nullptr;
			material = nullptr;
		}

		UStaticMesh * mesh = nullptr;
		UMaterialInterface * material = nullptr;

		TArray<UMaterialInterface*> materials;
	};

protected:
    void _init();


    void _initRuntimeMesh();

    void _loadResult(const AlgorithmResult& result);

	void _privateClear();

	// clears the output entity/element to instanced static mesh mappings
	void _clearInstancesData();

    void _updateInstance(FElement * element);

	void _updateEntity( Entity * entity );
    
	auto _computeInstanceTransform( FElement * element )->FTransform;

    void _destroyChildren(AActor* actor);
    
    void _updateSeeds();
    void _drawSeeds();
    void _destroySeeds();
    
    void _updateSamples();
    void _drawSamples();
    void _destroySamples();
    
	void _updateGenerationLimits();
	void _updateDrawBounds();
	FBox _generationBoundsWorld();
	void _drawBounds();
	void _undrawBounds();

	void _updateDrawHalfEdge();
	void _drawHalfEdge();
	void _undrawHalfEdge();

	void _updateDrawDirectionField();
    void _drawDirectionField();
    void _undrawDirectionField();
    
    bool _buildMeshInterface();
    void _updateDirectionField();
    
    auto _getGenerationLimits() -> Algorithm::AABB;
	void _loadNearestInLimits( Algorithm::AABB& limits );
    
    // Called during LoadExemplar
    void _loadInstancedStaticMeshes();
    
    void _loadMultiSampleExemplar(Domain& exemplarDomain);
    void _loadSingleSampleExemplar(Domain& exemplarDomain);

	void _loadTypeDescriptionsFromNewElementsAndSetTypes( std::vector<FElement*>& elements );
	void _reassignFaceIndices( std::vector<FElement*>& elements );

	// Generation
    auto _spaceFillingStartingPositions(Algorithm::AABB& limits, FVector& cameraLocation) -> std::vector<PositionFace>;
    auto _surfaceStartingPositions(Algorithm::AABB& limits, FVector& cameraLocation, FVector& cameraDirection) -> std::vector<PositionFace>;
    
    void _readContext();
    
	void _removeElements( std::vector<FElement*> toRemove );

	FString _optimazationParametersString( FNeighbourhoodParameters& params );

	std::vector<FElement*> _actorsToElements( std::vector<AElementActor*>& actors );

    
protected:
	double _duration = 0.0f;

    int tickCount;
	Algorithm_RegionGrowing _regionGrowing;
	Algorithm_Clustered _clusterGeneration;
	Algorithm_PatchCopy _patchCopyGeneration;

	Algorithm * algorithm = nullptr;
    
	std::shared_ptr<ctpl::thread_pool> _threadPool = nullptr;


	// Example selection
	std::unordered_map<AElementActor*, FElement*> _exampleActorToElement;
    std::map<int16, ElementTypeDescription> _elementTypesToDescriptions;
   
    std::map<int16, AActor*> _entityIDsToActors; // if an entity has a mesh, this table looks up the actor with that mesh
    std::map<int16, UInstancedStaticMeshComponent*> _entityIDsToInstancedMeshes;
    std::map<int16, int32> _entityIndexForID; // for each actor assigned to an entity id, we also assign the element samples take from that actor
    std::map<Entity*, int32> _entitiesToInstanceIndices;
    std::map<int16, FVector> _entityIDsToMeshOffsets;
    
    

    
    struct CachedElement
    {
        FTransform transform;
		int16 type;
        int32 instance;
    };

	

	InstanceManager _instanceManager;
    
    std::unordered_map<FElement*, CachedElement> _cache;
    ElementKDTree _elementKDTree;   // because algorithm is running on a background thread, we use this as a cache
                                    // of the last state of the element positions
    
    std::vector<std::vector<FElement*>> _previouslyCulled;
    
    UStaticMeshComponent * _nearestPointDebugMesh = nullptr;
    UStaticMeshComponent* nearestPointDebugMesh();
    
    UInstancedStaticMeshComponent * _seedPointsInstancedMeshComponent = nullptr;
    UInstancedStaticMeshComponent * seedPointsInstancedMeshComponent();
    

    std::shared_ptr<tcodsMeshInterface> _meshInterface = std::make_shared<tcodsMeshInterface>();
    bool _didInit = false;
    
    std::atomic<bool> _readyToSubmitNextGenerateCall = {true};
    ctpl::thread_pool _generationWorker;
	ctpl::detail::Queue<std::function<void()>> _mainThreadWork;

    std::function<void()> _generationWorkDone;
    
    double generationTime;
    
    EigenPointCloud _nearestInLimitsCloud;                        // the nearest sample points from the mesh interface that fall in the limits
	EigenDynamicPointCloudIndexAdaptor* _nearestInLimitsIndex = nullptr; // the kd-tree for the above
    
    FVector _exemplarPosition = FVector(0.0f, 0.0f, 0.0f);
    
    std::shared_ptr<RGOcclusionTester> _occlusionTester = nullptr;
    
	int _resetGuard = 0;

    // -----------------
    // Stretching
    // -----------------
    FVector _stretchStart;
    FVector _stretchLast;

	FVector * _exemplarPoint;

    struct StretchElement
    {
        size_t index;
        FVector originalPosition;
        float scale;
    };

    std::vector<StretchElement> _stretchVertices; // tcods::Vertex index and start position for a given vertex
    URuntimeMeshComponent * _debugMesh = nullptr;

	// Painting
	std::vector<PositionFace> _accumulatedStartingPositions;

	std::vector< PositionRadiusFace > _voidPoints;
	std::vector< PositionRadiusFace > _toRemove;



	UPROPERTY(EditAnywhere, BlueprintReadOnly)
	UBoxComponent * _boundsBox = nullptr;
};
