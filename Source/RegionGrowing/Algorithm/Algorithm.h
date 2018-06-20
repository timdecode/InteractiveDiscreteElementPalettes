//
//  Algorithm.h
//  RegionGrowing
//
//  Created by Timothy Davison on 2015-07-20.
//  Copyright (c) 2016 Timothy Davison. All rights reserved.
//

#pragma once

#include <map>
#include <memory>
#include <set>

#include "Domain.h"

#include "tcods/HalfEdge.h"

#include "tcodsMeshInterface.h"
#include "Algorithm/SpaceMapping.hpp"
#include "Eigen/Geometry"
#include "Algorithm/RoundRecord.h"

#include "ElementKDTree.h"
#include "AlgorithmEnums.h"

#include "ctpl_stl.h"

struct AlgorithmResult
{
    std::vector<FElement*> generated;
    std::vector<FElement*> modified;
    
    std::vector<FElement*> frozen;

	std::vector<FElement*> removed;
    
    void append(AlgorithmResult& other)
    {
        std::set<FElement*> generatedSet(generated.begin(), generated.end());
        std::set<FElement*> modifiedSet(modified.begin(), modified.end());
        std::set<FElement*> frozenSet(frozen.begin(), frozen.end());
		std::set<FElement*> removedSet(removed.begin(), removed.end());
        
        for( auto e : other.generated )
            generatedSet.insert(e);
        for( auto e : other.modified )
            modifiedSet.insert(e);
        for( auto e : other.frozen )
			frozenSet.insert(e);
		for(auto e : other.removed)
			removedSet.insert( e );

		for(auto e : other.removed)
		{
			generatedSet.erase( e );
			modifiedSet.erase( e );
			frozenSet.erase( e );
		}
        
        generated = std::vector<FElement*>(generatedSet.begin(), generatedSet.end());
        modified = std::vector<FElement*>(modifiedSet.begin(), modifiedSet.end());
        frozen = std::vector<FElement*>(frozenSet.begin(), frozenSet.end());
		removed = std::vector<FElement*>( removedSet.begin(), removedSet.end() );
    }
};

struct PositionFace
{
	PositionFace() {}
	PositionFace( Eigen::Vector3f position_in, int32_t face_in ) : position( position_in ), face( face_in ) {}

	Eigen::Vector3f position;
	int32_t face = -1;			// no face by default
};

struct PositionRadiusFace
{
	PositionRadiusFace() {}
	PositionRadiusFace( Eigen::Vector3f position_in, float radius_in, int32_t face_in ) : position( position_in ), radius( radius_in ), face( face_in ) {}

	Eigen::Vector3f position;
	int32_t face = -1;			// no face by default
	float radius = 0.0f;
};

struct PositionFacePairCloud
{
	std::vector<PositionRadiusFace>  pts;

	// Must return the number of data points
	inline size_t kdtree_get_point_count() const { return pts.size(); }

	// Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
	inline float kdtree_distance( const float *p1, const size_t idx_p2, size_t /*size*/ ) const
	{
		const auto& p2_3f = pts[idx_p2].position;
		const auto& p1_3f = *reinterpret_cast<const Eigen::Vector3f*>(p1);

		return (p2_3f - p1_3f).squaredNorm();
	}

	inline float kdtree_get_pt( const size_t idx, int dim ) const
	{
		return pts[idx].position( dim );
	}

	template <class BBOX>
	bool kdtree_get_bbox( BBOX& /*bb*/ ) const { return false; }
};

typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
	nanoflann::L2_Simple_Adaptor<float, PositionFacePairCloud>,
	PositionFacePairCloud,
	3 /* dim */
> PositionFacePairCloudAdaptor;



class OcclusionBase
{
public:
	virtual bool isVisible( PositionFace point, float radius )
	{
		return true;
	}
};

class Algorithm
{
public:
    struct SimilarElement
    {
        FElement * element;
        
        std::vector<std::pair<FElement*, FElement*> > fullPairings;
        std::vector<std::pair<FElement*, FElement*> > leftPartialPairings;
        std::vector<std::pair<FElement*, FElement*> > rightPartialPairings;
        
        float cost;
    };

	struct SimilarElement_NoPairings
	{
		FElement * element = nullptr;
		float cost = 0.0f;
	};
    
    struct AABB
    {
        Eigen::AlignedBox3f aabb;
        
        AABB()
        {
            const Eigen::Vector3f one = Eigen::Vector3f(1.0f, 1.0f, 1.0f);
            aabb = Eigen::AlignedBox3f(-1000000.0f * one, 1000000.0f * one);
        }
        
        bool contains(const Eigen::Vector3f& position)
        {
            return aabb.contains(position);
        }

		bool contains( const Eigen::Vector3f& position, float radius )
		{
			const Eigen::Vector3f halfExtents( radius, radius, radius );

			const Eigen::AlignedBox3f radiusBox( position - halfExtents, position + halfExtents );

			return aabb.contains( radiusBox );
		}
        
        struct Hit
        {
            float tMin;
            float tMax;
            bool hit;
        };
        
        
        Hit intersection(Eigen::Vector3f origin, Eigen::Vector3f direction)
        {
            // from http://psgraphics.blogspot.ca/2016/02/new-simple-ray-box-test-from-andrew.html
            
            Eigen::Vector3f dirInv = direction.cwiseInverse();
            
            Eigen::Vector3f t0 = (aabb.min() - origin).cwiseProduct(dirInv);
            Eigen::Vector3f t1 = (aabb.max() - origin).cwiseProduct(dirInv);
            
            Hit result;
            
            result.tMin = t0.minCoeff();
            result.tMax = t1.maxCoeff();
            result.hit = false;
            
            for( int c = 0; c < 3; c++ )
            {
                float t0_ = t0(c);
                float t1_ = t1(c);
                
                if( dirInv(c) < 0.0f )
                    std::swap(t0_, t1_);
                
                result.tMin = t0_ > result.tMin ? t0_ : result.tMin;
                result.tMax = t1_ < result.tMax ? t1_ : result.tMax;
                
                if( result.tMax <= result.tMin )
                    return result;
            }
            
            result.hit = true;
            return result;
        }
    };

	// Freespace Clustering
	struct Freespace
	{
		std::unordered_map<FElement*, std::vector<Eigen::Vector3f>> _freespacePoints;
		EigenPointCloud _freespaceCloud;
		EigenDynamicPointCloudIndexAdaptor* _freespaceIndex = nullptr;
		std::set<size_t> _freespaceRemovedIndices;

		size_t _pointCount = 0;

		void clear()
		{
			_freespacePoints.clear();
			_freespaceRemovedIndices.clear();
			_freespaceCloud.pts.clear();

			_pointCount = 0;

			if(_freespaceIndex)
				delete _freespaceIndex;

			_freespaceIndex = new EigenDynamicPointCloudIndexAdaptor( 3, _freespaceCloud, nanoflann::KDTreeSingleIndexAdaptorParams( 10 /* max leaf */ ) );
		}
		
		void rebuild()
		{
			std::vector<Eigen::Vector3f> points;

			points.reserve( _freespaceCloud.pts.size() - _freespaceRemovedIndices.size() );

			for(int i = 0; i < _freespaceCloud.pts.size(); ++i)
			{
				if(_freespaceRemovedIndices.find( i ) == _freespaceRemovedIndices.end())
					points.push_back( _freespaceCloud.pts[i] );
			}

			_freespaceCloud.pts.clear();

			if(_freespaceIndex)
				delete _freespaceIndex;

			_freespaceIndex = new EigenDynamicPointCloudIndexAdaptor( 3, _freespaceCloud, nanoflann::KDTreeSingleIndexAdaptorParams( 10 /* max leaf */ ) );

			_freespaceCloud.pts = std::move( points );

			_freespaceRemovedIndices.clear();

			if(_freespaceCloud.pts.size() > 0)
				_freespaceIndex->addPoints( 0, _freespaceCloud.pts.size() - 1 );
		}

		void remove( std::vector<FElement*>& elements )
		{
			for(auto e : elements)
			{
				auto found = _freespacePoints.find( e );

				if(found != _freespacePoints.end())
				{
					_freespacePoints.erase( found );
				}
			}

			// do we have to remove points from the _freespaceCloud?
			// they'll still be in the cloud, but not accessible via the kd-tree
		}

		auto outputSize() -> size_t
		{
			return _pointCount;
		}

		void addOutputPoint( Eigen::Vector3f point )
		{
			size_t newIndex = _freespaceCloud.kdtree_get_point_count();

			_freespaceCloud.pts.emplace_back( point );
			_freespaceIndex->addPoints( newIndex, newIndex );

			_pointCount++;
		}

		void removeOutputPoint( size_t index )
		{
			_freespaceIndex->removePoint( index );

			_freespaceRemovedIndices.insert( index );

			_pointCount--;
		}

		auto outputPoint( size_t index ) -> Eigen::Vector3f
		{
			return _freespaceCloud.pts[index];
		}

		auto outputIndicesInRadius( Eigen::Vector3f& atPosition, float inRadius ) -> std::vector< std::pair<size_t, float> >
		{
			std::vector<std::pair<size_t, float> > dynoResult;

			nanoflann::SearchParams searchParams;
			searchParams.sorted = false;
			float searchRadiusSqrd = inRadius * inRadius;

			_freespaceIndex->radiusSearch( &atPosition( 0 ), searchRadiusSqrd, dynoResult, searchParams );

			return dynoResult;
		}

		auto outputPointsInRadius( Eigen::Vector3f& atPosition, float inRadius ) -> std::vector < Eigen::Vector3f >
		{
			auto indices = outputIndicesInRadius( atPosition, inRadius );

			std::vector < Eigen::Vector3f > points;
			points.reserve( indices.size() );

			for(auto& pair : indices)
				points.push_back( outputPoint( pair.first ) );

			return points;
		}

		auto freespacePoints( FElement * exemplarElement ) -> std::vector<Eigen::Vector3f>&
		{
			return _freespacePoints[exemplarElement];
		}
	};

	



public:
	virtual ~Algorithm() {}

	virtual void init( std::shared_ptr<ctpl::thread_pool> threadPool = nullptr );

	virtual AlgorithmResult generate( std::vector<PositionFace>& positions, float radius = -1.0f, AABB limits = AABB() );
	virtual AlgorithmResult horizonOptimization(); // local optimization (of the horizon)
	virtual AlgorithmResult globalOptimization();	

	virtual void clear();

	virtual void loadExemplar();

	
	void setThreadPool(std::shared_ptr<ctpl::thread_pool> threadPool) { _threadPool = threadPool; }
    void setOcclusionTester(std::shared_ptr<OcclusionBase> const& occlusionTester);

	
	Domain& exemplar() { return _exemplar; }
    Domain& output() { return _output; }


    void setMeshInterface(std::shared_ptr<tcodsMeshInterface> meshInterface);
    
    void setToMeshTransform(const Eigen::Affine3f& transform);
    
    std::set<FElement*> horizon() { return _horizon; }
    

    

	void clearOutput();
	void clearPainting();

	void loadOutputElements( std::vector<FElement>& toLoad );

	void removeElements( std::vector<FElement*>& elements );
    void addToHorizon( std::vector<FElement*>& elements );

	void addBrushPoint( PositionRadiusFace& point );    
	void clearBrushPoints();

	std::vector<FElement*> eraseAt( PositionRadiusFace brushPoint );

	void addVoidPoint( PositionRadiusFace& point );

	void setCurrentBrushPoint( PositionRadiusFace& point );
	void clearCurrentBrushPoint();

	typedef struct ExampleSelection *ExampleSelectionPtr;
	ExampleSelectionPtr addExampleSelection( std::vector<FElement*> selection, float weight = 1.0f );
	void removeExampleSelection( ExampleSelectionPtr ptr );
	void updateExampleSelection( ExampleSelectionPtr ptr, std::vector<FElement*> selection, float weight = 1.0f );

	SimilarElement distance( FElement& element, std::vector<FElement*>& elementNeighbours,
                            FElement& example, std::vector<FElement*>& exampleNeighbours,
                            rg::Mapping* mapping);

	SimilarElement distance_densityBased( FElement& element, std::vector<FElement*>& elementNeighbours, 
		FElement& example, std::vector<FElement*>& exampleNeighbours,
		rg::Mapping* mapping );

	SimilarElement distance_hungarian( FElement& element, std::vector<FElement*>& elementNeighbours,
		FElement& example, std::vector<FElement*>& exampleNeighbours,
		rg::Mapping* mapping );
	
	std::unique_ptr<RecordSummary> _totalSummary;
	virtual auto roundSummary() -> RecordSummary&
	{
		if(!_totalSummary)
			_totalSummary = std::make_unique<RecordSummary>();

		return *(_totalSummary.get());
	}

	void beginRound();
	void endRound(AlgorithmResult& result);

	auto totalEnergy_kCoherence( FNeighbourhoodParameters parameters ) -> double;
	auto totalEnergy_bruteForce( FNeighbourhoodParameters parameters ) -> double;

public:
    EGenerationMode generationMode = EGenerationMode::SpaceFilling;

	bool perElementParameters = false;

	bool sketchBasedForceTangentPlane = false;

	bool enableRoundSummaries = false;
	bool calculate_kCoherenceEnergy = false;
	bool calculate_bruteForceEnergy = false;
	bool useCinpactEnergy = false;
    
    float typeCost = 300.0f;

	bool useTypeVoting = false;

	bool usePatchSelection = false;

	float gradientTerm = 1.0f;

    float minAssignmentDistance = 2.0f;
    
    float freespaceRadius = 0.0f;
    
    uint32_t kCoherence = 5;
    float kCoherenceClusteringPenalty = 0.0f;
    float kCoherenceClusteringPenaltyRadius = 5.0f;
    
    float relaxation = 0.8f;

	bool enableVolumeSurfaceInteraction = true;
    
    bool ignoreBadSuggestions = false;
	float ignoreBadSuggestionsDistanceFactor = 10.0f;

	bool forceRotation = false;
	FQuat forcedRotation = FQuat::Identity;

	bool disableOptimization = false;
    
    FNeighbourhoodParameters generationParameters = {10.0f, -1};
    FNeighbourhoodParameters optimizationParameters = {10.0f, -1};
    float generationInnerRadius = 50000.0f;
	float frozenElementRadius = 0.0f;

    int seedSeparation = 6;

	bool seedsIgnoreMeshInBoundary = false;

	int optimizationRounds = 1;
	bool useGlobalOptimization = false;
    
    float brushSize = 30.0f;
    
    float sourceHistogramWeight = 1.0f;
    float sourceHistogramRadius = 5.0f;
    
    float samplingDistanceWeight = 1.0f;
    
    bool flipSurfaceNormals = false;

	bool removeOverlaps = false;

	FIntVector patchInitializationExemplarDivisions = FIntVector( 1, 1, 1 ); // a huge polymorphism hack, i know
	float patchMarginPercentage = 0.0f;

	int32 nThreads = 2;

	float voidSize = 10.0f;

	float cellExtents = 1.0f;

protected:
	void _conditionally_initializeThreadPool();
	virtual void _initialize();
	void _initElementParameters();

	void _initDefaultSelection();

	void _initSelections();
	void _initExemplarTypeHistogram();
    void _initFreespacePoints();
	void _initLSA();

	void _removeBrushPoint( PositionRadiusFace& point );
	void _removeVoidPoint( PositionRadiusFace& point  );
    
	// forceSpaceFilling is an option so that we can setup the KCoherentNeighbours with a normal mapping
    auto _mapping(Eigen::Vector3f const & surfaceOrigin, Eigen::Vector3f const & exemplarOrigin, bool forceSpaceFilling = false ) -> std::unique_ptr<rg::Mapping>
    {
		if(generationMode == EGenerationMode::SpaceFilling || generationMode == EGenerationMode::SpacePainting || forceSpaceFilling)
			return std::unique_ptr<rg::Mapping>( new rg::SpaceMapping( _meshInterface.get(), surfaceOrigin, exemplarOrigin ) );
		else if(generationMode == EGenerationMode::SurfaceProjection || sketchBasedForceTangentPlane)
			return std::unique_ptr<rg::Mapping>( new rg::SurfaceMapping( _meshInterface.get(), surfaceOrigin, exemplarOrigin ) );
		else
			return std::unique_ptr<rg::SurfaceWalk>( new rg::SurfaceWalk( _meshInterface.get(), surfaceOrigin, exemplarOrigin ) );
    }

	auto _mapping( FElement& element, FElement& exemplar, bool forceSpaceFilling = false ) -> std::unique_ptr<rg::Mapping>
	{
		if((generationMode == EGenerationMode::SurfaceWalking || generationMode == EGenerationMode::SurfacePainting) && !(forceSpaceFilling || sketchBasedForceTangentPlane))
			return std::unique_ptr<rg::SurfaceWalk>( new rg::SurfaceWalk( _meshInterface.get(), element.position, element.faceIndex, exemplar.position ) );
		else
			return _mapping( element.position, exemplar.position );
	}

	auto _mapping( FElement& element, bool forceSpaceFilling = false ) -> std::unique_ptr<rg::Mapping>
	{
		if((generationMode == EGenerationMode::SurfaceWalking || generationMode == EGenerationMode::SurfacePainting) && !(forceSpaceFilling|| sketchBasedForceTangentPlane))
			return std::unique_ptr<rg::SurfaceWalk>( new rg::SurfaceWalk( _meshInterface.get(), element.position, element.faceIndex, Eigen::Vector3f::Zero() ) );
		else
			return _mapping( element.position, Eigen::Vector3f::Zero(), forceSpaceFilling );
	}

	auto _mapping( Eigen::Vector3f & surfaceOrigin, uint32_t faceIndex, Eigen::Vector3f exemplarOrigin = Eigen::Vector3f::Zero() )->std::unique_ptr<rg::Mapping>
	{
		if((generationMode == EGenerationMode::SurfaceWalking || generationMode == EGenerationMode::SurfacePainting) && !sketchBasedForceTangentPlane)
			return std::unique_ptr<rg::SurfaceWalk>( new rg::SurfaceWalk( _meshInterface.get(), surfaceOrigin, faceIndex, exemplarOrigin ) );
		else
			return _mapping( surfaceOrigin, exemplarOrigin );
	}


    // Initializes the output domain at the given position by copying elements from
    // the exemplar to the output domain.
    // not called by _initialize()
    void _initializeOutput(PositionFace& position, AABB& limits, AlgorithmResult& result);
    
    /*
     Returns the elements that can still generate new elements. The elements that can no longer generate new elements
     are placed in the frozen_out vector.
     \param elements The elements to filter.
     \param domain The domain to search for neighbours in.
     \param frozen_out The elements removed from elements.
     \return The remaining elements that can predict new elements.
     */
    std::set<FElement*> _removeFrozen(const std::set<FElement*>& elements, Domain& domain, std::vector<FElement*>& frozen_out);
    std::set<FElement*> _removeFrozen_usingFreespacePoints(const std::set<FElement*>& elements, Domain& domain, std::vector<FElement*>& frozen_out);

    std::vector<FElement*> _seeds(const std::set<FElement*>& elements);

    float _closestPoint(const std::vector<Eigen::Vector3f>& aPositions, const std::vector<FElement*>& aNeighbours,
                        const std::vector<Eigen::Vector3f>& bPositions, const std::vector<FElement*>& bNeighbours,
                        std::vector<std::pair<int,int>>& pairings_out);
    
    float _sourceHistogramCost(const std::vector<FElement*>& outputElements, const std::vector<FElement*>& exemplarElements);
    std::map<FElement*, int> _histogram(FElement * element, std::vector<FElement*>& neighbours);
    float _compareHistograms(std::map<int,int> histogramA, int countA,
                             std::map<int,int> histogramB, int countB);

	// forceSpaceFilling is an option for setting up the k-nearest neighbours in the exemplar
    std::vector<Algorithm::SimilarElement> _nearestSimilarElements(
		FElement* element, Domain& elementDomain,
        std::vector<FElement*> candidates, Domain& candidateDomain,
        const FNeighbourhoodParameters& searchParameters,
		bool forceSpaceFilling = false,
		bool filterCandidatesByType = true
	);
    
    std::vector<Algorithm::SimilarElement> _generation_nearestSimilarElements(FElement* element, Domain& elementDomain,
                                                                              std::vector<FElement*> candidates, Domain& candidateDomain,
                                                                              const FNeighbourhoodParameters& searchParameters);


    void _orderNearest(std::vector<FElement*>& elements, const Eigen::Vector3f referencePoint);
    
    
	struct OptimizationProblem
	{
		std::set<FElement*> activeElements;
		std::set<FElement*> frozenElements;
		std::set<FElement*> elements;
	};
	enum PredictionType : uint8_t 
	{
		FrozenElements,
		ActiveElements
	};

	OptimizationProblem _localOptimization( OptimizationProblem problem, AlgorithmResult& result);

	void _removeNonOptimizingElements( std::set<FElement *>& localElements );

	std::set<FElement*> _filterSurfaceVolumeInteractions( std::set<FElement*>& elements );

	
	std::set<FElement*> _nearbyFrozenElements( const std::set<FElement*>& activeElements );
    
    bool _overlaps(const Eigen::Vector3f& position, const float radius, FNeighbourhoodParameters& parameters, const FElement* elementToIgnore = nullptr);
    bool _inBoundary(const Eigen::Vector3f& position, const float radius);
    
    std::vector<FElement*> _sphereOfInfluence(FElement * e, float r = -1.0);
    
    float _maxRadius = 0.0f;
    float _computeMaxRadius();
    

	// Latent Semantic Analysis Idea
    void _sampleNeighbourhood(FElement * element, Domain& domain, Eigen::VectorXf& vector_out);
    auto _discretize(FElement* element, Domain& domain, const int n) -> Eigen::VectorXf;
    
	// Freespace
    void _rebuildFreespace();
	std::vector<FElement*> _fastFilterGenerativeExemplars( FElement * seed, Freespace& freespace );
    
    void _expandFreeSpace(std::vector<FElement*>& generated);


	void _initBrushIndex();
	void _initVoidIndex();

protected:
	struct ExampleSelection
	{
	public:
		std::unordered_set<FElement*> selection;

		// k-coherence search
		std::unordered_map<FElement*, std::vector<FElement*> > _kCoherentElements;

		std::unordered_map<int16, std::vector<FElement*>> _generativeElementsByType;

		std::vector<FElement*> _generativeElements;

		float weight = 1.0f;

		void clear();

		void init( Domain& exemplar, Algorithm& algorithm );

	protected:
		void _initGenerativeExamples( Domain& exemplar );

		void _initKCoherentNeighbours( 
			Domain& exemplar, 
			Algorithm& algorithm
		);
	};

	friend ExampleSelection;

	std::shared_ptr<Algorithm::ExampleSelection> _defaultSelection = std::make_shared<Algorithm::ExampleSelection>();
	std::vector< std::shared_ptr<Algorithm::ExampleSelection> > _exampleSelections;

	std::shared_ptr<Algorithm::ExampleSelection> _highestWeightExampleSelection();

protected:
	struct SourceExampleMap
	{
	public:
		struct SourceExample
		{
			SourceExample() = delete;

			SourceExample( FElement * element, float weight, std::shared_ptr<ExampleSelection> exampleSelection ) :
				element( element ), weight( weight ), selection( exampleSelection ) {}

			FElement * element = nullptr;
			float weight = 0.0f;

			std::vector<FElement*>& kCoherentNeighbours( ExampleSelection& defaultSelection )
			{
				if(auto ptr = selection.lock())
					return ptr->_kCoherentElements[element];
				else
					return defaultSelection._kCoherentElements[element];
			}

			std::vector<FElement*>& generativeElementsByType( int16 type, ExampleSelection& defaultSelection )
			{
				if(auto ptr = selection.lock())
					return ptr->_generativeElementsByType[type];
				else
					return defaultSelection._generativeElementsByType[type];
			}

			std::weak_ptr<ExampleSelection> selection;
		};

		std::vector<SourceExample>& sourceExamples( FElement * outputElement )
		{
			return _sourceExamples[outputElement];
		}

		void eraseElement( FElement * outputElement )
		{
			auto found = _sourceExamples.find( outputElement );

			if(found != _sourceExamples.end())
				_sourceExamples.erase( found );
		}

		void clear()
		{
			_sourceExamples.clear();
		}

	protected:
		std::unordered_map<FElement*, std::vector<SourceExample> > _sourceExamples; // key = output element, value = the example element that create the output element
	};

protected:
	struct PredictionNeighbourhood
	{
		FElement * element = nullptr;
		SimilarElement similarNeighbourhood;

		float weight = 1.0f;

		std::shared_ptr<Algorithm::ExampleSelection> exampleSelection;
	};

	struct PredictedPosition
	{
		PredictedPosition( size_t elementIndex, Eigen::Vector3f position, float weight ) : elementIndex( elementIndex ), position( position ), weight( weight ) {}

		PredictedPosition( size_t elementIndex, Eigen::Vector3f position ) : elementIndex( elementIndex ), position( position ), weight( 1.0f ) {}

		int32_t elementIndex;
		Eigen::Vector3f position;

		float weight = 1.0f;
	};

	std::vector<PredictionNeighbourhood> _predictionNeighbourhoods( std::set<FElement *>& elements );

	void _buildPredictions(
		const std::vector<PredictionNeighbourhood>& predictionNeighbourhoods,
		const std::set<FElement *>& excludeFromPredictions,
		std::map<FElement *, unsigned int>& problem,
		std::vector< std::vector< PredictedPosition > >& predictions_in_out
	);

protected:
	FElement* _copyExemplarToOutput( FElement* exemplarElement, const Eigen::Vector3f& position, std::shared_ptr<Algorithm::ExampleSelection> selection );


protected:
    bool _didInit = false;
    
    Domain _output;
    
    Domain _exemplar;
    
    std::map<int,int> _exemplarTypeHistogram; // a histogram of elementID counts in the exemplar
    
    Eigen::Affine3f _toMeshTransform = Eigen::Affine3f(Eigen::Affine3f::Identity());
    std::shared_ptr<tcodsMeshInterface> _meshInterface;
    
    std::set<FElement*> _horizon;

	SourceExampleMap _sourceExampleMap;

    // LSA
    Eigen::MatrixXf queryTransformMatrix;
    Eigen::MatrixXf lowDimensionalNeighbourhoodMatrix;
    std::vector<SimilarElement> _lsa_nearestSimilarElements(FElement* element, Domain& elementDomain, const FNeighbourhoodParameters& searchParameters);
    
    

	Freespace _freeSpace;

    
    // Threading
    // If a thread_pool is not provided, it will be created in _initialize(). Calling clear()
    // does not recreate the thread_pool.
    
    std::shared_ptr<ctpl::thread_pool> _threadPool;
    
    std::shared_ptr<OcclusionBase> _occlusionTester;

	// Painting
	PositionFacePairCloud _brushPoints;
	std::unique_ptr<PositionFacePairCloudAdaptor> _brushIndex;

	bool _hasCurrentBrushPoint = false;
	PositionRadiusFace _currentBrushPoint;

	bool _canRemoveFromHorizon( FElement * element );

	// Void regions
	PositionFacePairCloud _voidPoints;
	std::unique_ptr<PositionFacePairCloudAdaptor> _voidIndex;

protected:
	// CINPACT energy
	// --------------
	auto _cinpact( const float u, const float k, const float c ) -> float;

	auto _cinpactSum( const Eigen::Vector3f point, const std::vector<FElement*>& elements, const int16_t elementType ) -> float;
	auto _discreteCinpactSimiliarity( const FElement& a, Domain& aDomain, const FElement& b, Domain& bDomain ) -> float;

	auto _nearestElements_discreteCinpactSimilarity( const FElement& element, Domain& domain, std::vector<FElement*> toConsiderInExemplar, Domain& exemplar )->std::vector<SimilarElement_NoPairings>;
};


