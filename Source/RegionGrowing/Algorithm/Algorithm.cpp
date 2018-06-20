//
//  Algorithm.cpp
//  RegionGrowing
//
//  Created by Timothy Davison on 2015-07-20.
//  Copyright (c) 2015 EpicGames. All rights reserved.
//

#include "RegionGrowing.h"

#include <unordered_set>
#include <limits>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <memory>
#include <ctime>


#include "Algorithm.h"
#include "dbscan.hpp"
#include "SpaceMapping.hpp"

#include "Eigen/Sparse"
#include "Eigen/Geometry"

using namespace Eigen;
using namespace std;

class NoOcclusion : public OcclusionBase
{
public:
	virtual bool isVisible( PositionFace point, float radius )
	{
		return true;
	}
};

// -----------------------------------------------------------------------------
/// Initialization
// -----------------------------------------------------------------------------

void Algorithm::_conditionally_initializeThreadPool()
{
	if(_threadPool == nullptr)
	{
		//unsigned int n = std::thread::hardware_concurrency();

		_threadPool = std::make_shared<ctpl::thread_pool>();
		_threadPool->resize( nThreads );
	}
}

void Algorithm::_initialize()
{
    _maxRadius = _computeMaxRadius();

	_initElementParameters();
    
	_initDefaultSelection();
    _initSelections();
    _initExemplarTypeHistogram();

	_initFreespacePoints();

	_initBrushIndex();
	_initVoidIndex();
    
    if( _occlusionTester == nullptr )
    {
        _occlusionTester = std::make_shared<NoOcclusion>();
    }
    
    _didInit = true;
}

float Algorithm::_computeMaxRadius()
{
    float maxRadius = 0.0f;
    for( auto& e : _exemplar )
    {
        const float radius = e->radius;
        if( radius > maxRadius )
            maxRadius = radius;
    }
    
    return maxRadius;
}

void Algorithm::_initElementParameters()
{
	// use the element parameters as is
	if(perElementParameters)
		return;

	// don't use per-element parameters? then override all of them with the algorithm's parameters
	for(auto& e : _exemplar)
	{
		e->minAssignmentDistance = minAssignmentDistance;
		e->freespaceRadius = freespaceRadius;
		e->generationParameters = generationParameters;
		e->optimizationParameters = optimizationParameters;
		e->generationInnerRadius = generationInnerRadius;
	}
}

void Algorithm::_initDefaultSelection()
{
	_defaultSelection->clear();

	// create the default selection
	for(auto& ptr : _exemplar)
	{
		FElement * element = ptr.get();

		_defaultSelection->selection.insert( element );
	}

	_defaultSelection->init( exemplar(), *this );
}

void Algorithm::_initSelections()
{
	_sourceExampleMap.clear();

	for(auto& ptr : _exampleSelections)
	{
		ptr->init( _exemplar, *this );
	}

}

void Algorithm::_initExemplarTypeHistogram()
{
    _exemplarTypeHistogram.clear();
    
    for( auto& ex : _exemplar )
        _exemplarTypeHistogram[ex->type]++;
}

void Algorithm::_initFreespacePoints()
{
	_freeSpace.clear();



	const auto& exampleElements = _defaultSelection->selection;

	for(FElement * element : exampleElements)
	{
		std::vector<FElement*>& coherentExamples = _defaultSelection->_kCoherentElements[element];

		std::vector<Eigen::Vector3f> points;
		for(FElement * c : coherentExamples)
		{
			auto position = c->position;

			auto neighbours = _exemplar.neighbours( c, c->generationParameters.radius );

			for(auto& c_ : neighbours)
				points.push_back( c_->position - position + element->position );
		}

		std::vector< std::vector<rg::PointAndIndex> > clusters_out;
		std::vector<rg::PointAndIndex> noise_out;

		rg::dbscan( points, freespaceRadius, 2, clusters_out, noise_out );

		std::vector<Eigen::Vector3f>& freePoints = _freeSpace.freespacePoints( element );

		auto setupCluster = [&]( std::vector<rg::PointAndIndex>& dbCluster ) mutable
		{
			// find the midpoint and add use that as the free-space point
			Eigen::Vector3f centre = Eigen::Vector3f::Zero();

			for(auto& c : dbCluster)
				centre += c.point;

			centre /= dbCluster.size();

			freePoints.push_back( centre );
		};

		// add noise as their own little clusters
		for(auto& noise : noise_out)
		{
			std::vector<rg::PointAndIndex> oneNoise;
			oneNoise.push_back( noise );

			setupCluster( oneNoise );
		}

		for(auto & dbCluster : clusters_out)
			setupCluster( dbCluster );
	}

	//for (auto& exampleSelection : _exampleSelections)
	//{
	//	const auto& exampleElements = exampleSelection->selection;

	//	for ( FElement * element : exampleElements)
	//	{
	//		std::vector<FElement*>& coherentExamples = exampleSelection->_kCoherentElements[element];

	//		std::vector<Eigen::Vector3f> points;
	//		for (FElement * c : coherentExamples)
	//		{
	//			auto position = c->position;

	//			auto neighbours = _exemplar.neighbours(c, c->generationParameters.radius);

	//			for (auto& c_ : neighbours)
	//				points.push_back(c_->position - position + element->position);
	//		}

	//		std::vector< std::vector<rg::PointAndIndex> > clusters_out;
	//		std::vector<rg::PointAndIndex> noise_out;

	//		rg::dbscan(points, freespaceRadius, 2, clusters_out, noise_out);

	//		std::vector<Eigen::Vector3f>& freePoints = _freeSpace.freespacePoints(element);

	//		auto setupCluster = [&](std::vector<rg::PointAndIndex>& dbCluster) mutable
	//		{
	//			// find the midpoint and add use that as the free-space point
	//			Eigen::Vector3f centre = Eigen::Vector3f::Zero();

	//			for (auto& c : dbCluster)
	//				centre += c.point;

	//			centre /= dbCluster.size();

	//			freePoints.push_back(centre);
	//		};

	//		// add noise as their own little clusters
	//		for (auto& noise : noise_out)
	//		{
	//			std::vector<rg::PointAndIndex> oneNoise;
	//			oneNoise.push_back(noise);

	//			setupCluster(oneNoise);
	//		}

	//		for (auto & dbCluster : clusters_out)
	//			setupCluster(dbCluster);
	//	}
	//}
}

void Algorithm::loadOutputElements( std::vector<FElement>& toLoad )
{
	clearPainting();
	clearOutput();

	_output.setElements( toLoad, true /* create entities too */ );

	std::vector<FElement*> allExamples = _exemplar.elements();

	// now we need to recalculate the sourceExample for every output domain element


	std::vector< std::future< std::pair<FElement*, FElement* > > > futures;


	for(auto& ptr : _output)
	{
		FElement * element = ptr.get();

		futures.emplace_back( _threadPool->push( [this, element, &allExamples]( int threadID ) mutable {
			auto& found = _defaultSelection->_generativeElementsByType[element->type];

			std::vector<FElement*>& generativeExemplars = found.size() ? found : allExamples;

			auto nearest = _nearestSimilarElements(
				element,
				_output,
				generativeExemplars,
				_exemplar,
				element->generationParameters
			);

			FElement * similarExample = nearest.size() ? nearest[0].element : nullptr;


			return std::make_pair( element, similarExample );
		} ) );
	}

	for(auto& future : futures)
	{
		auto pair = future.get();

		if(pair.second)
		{
			std::vector<SourceExampleMap::SourceExample>& sourceExamples = _sourceExampleMap.sourceExamples( pair.first );

			sourceExamples.emplace_back( pair.second, 1.0f, _defaultSelection );
		}
	}
}

std::shared_ptr<Algorithm::ExampleSelection> Algorithm::_highestWeightExampleSelection()
{
	float maxWeight = 0.0f;
	std::shared_ptr<Algorithm::ExampleSelection> maxSelection;

	for(auto& selection : _exampleSelections)
	{
		if(selection->weight >= maxWeight)
		{
			maxWeight = selection->weight;
			maxSelection = selection;
		}
	}

	if(maxSelection == nullptr)
		maxSelection = _defaultSelection;

	return maxSelection;
}

void Algorithm::_initializeOutput(PositionFace& position, AABB& limits, AlgorithmResult& result)
{
    if( _exemplar.size() == 0 )
        return;
    
    Vector3f samplingPoint = Vector3f::Zero();
    
	// where to sample the exemplar from
    std::set<int32_t> neighbourhoodEntityIndices;

	std::shared_ptr<Algorithm::ExampleSelection> exampleSelection = _highestWeightExampleSelection();
	auto& selectionElements = exampleSelection->selection;

	// sample from the centroid
	std::vector<FElement*> elementsToSample;
	if( selectionElements.size() )
		elementsToSample = std::vector<FElement*>( selectionElements.begin(), selectionElements.end() );
	else
		elementsToSample = _exemplar.elements();

	// find the centre of the exemplar
	Vector3f centre = Vector3f::Zero();

	for(auto& element : elementsToSample)
		centre += element->position;

	centre /= float( elementsToSample.size() );

	samplingPoint = centre;

	// which is the closest element to the sampling point?
	FElement * closestElement = nullptr;
	{
		float minDistanceSqrd = std::numeric_limits<float>::max();
		for(FElement * e : elementsToSample)
		{
			float dist = (e->position - samplingPoint).squaredNorm();

			if(dist < minDistanceSqrd)
			{
				closestElement = e;
				minDistanceSqrd = dist;
			}
		}
	}

	if(!closestElement)
		return;

	auto neighbourhood = _exemplar.nearestInRadius( samplingPoint, closestElement->generationParameters.radius );

	for(auto neighbour : neighbourhood)
	{
		if(selectionElements.size() == 0 || selectionElements.find(neighbour) != selectionElements.end() )
			neighbourhoodEntityIndices.insert( neighbour->entityIndex );
	}
    
    auto mapping = position.face < 0 ? _mapping( position.position, samplingPoint ) : _mapping(position.position, position.face, samplingPoint);
    
    auto& outputEntities = _output.entities();
    
    for( int32_t entityIndex : neighbourhoodEntityIndices )
    {
        Entity& entity = _exemplar.entityAt(entityIndex);
        
        int32_t outputEntityIndex = outputEntities.size();

        outputEntities.emplace_back();
        Entity& outputEntity = outputEntities.back();
        outputEntity.entityID = entity.entityID;
        
        for( auto sampleIndex : entity.elementIndices )
        {
            FElement * element = _exemplar.elementAt(sampleIndex);
            
            auto p = mapping->toSurface(element->position);

            if( !p.hit )
                continue;
            
            if( !_inBoundary(p.position, element->radius) )
                continue;
            
            if( !limits.contains(p.position, element->radius) )
                continue;
            
            Eigen::Vector3f p_ = p.position;
            
			PositionFace facePair( p.position, p.face );

			if(selectionElements.find( element ) == selectionElements.end())
				continue;

            if( !_occlusionTester->isVisible( facePair, element->radius) )
                continue;
            
            if( _overlaps(p_, element->radius, element->generationParameters) )
                continue;
            
            int32_t ouputSampleIndex = _output.size();
            FElement * added = _copyExemplarToOutput(element, p_, exampleSelection );
            
            added->entityIndex = outputEntityIndex;
			added->faceIndex = p.face;

            if( !forceRotation && (generationMode == EGenerationMode::SurfaceProjection || generationMode == EGenerationMode::SurfaceWalking || generationMode == EGenerationMode::SurfacePainting))
			{
				auto rotationAndNormal = _meshInterface->rotationAndNormalAtIndex( p.face );

				FQuat quat = rotationAndNormal.first;
				FQuat rotation = quat * unreal( element->rotation );

				added->rotation = eigen( rotation );
			}
			else if(forceRotation)
			{
				added->rotation = eigen(forcedRotation);
			}
            
            outputEntity.elementIndices.push_back(ouputSampleIndex);
            
            result.generated.push_back(added);
        }
    }

	_output.rebalance();
}

FElement* Algorithm::_copyExemplarToOutput(FElement * exemplarElement, const Vector3f& position, std::shared_ptr<Algorithm::ExampleSelection> selection)
{
    FElement modifiedExemplar(*exemplarElement);  
    modifiedExemplar.position = position;
    
    FElement* element = _output.add(modifiedExemplar);
    _horizon.insert(element); 
    
	auto& sourceExamples = _sourceExampleMap.sourceExamples( element );

	sourceExamples.emplace_back( exemplarElement, 1.0f, selection );
    
    return element;
}

void Algorithm::removeElements( std::vector<FElement*>& elements )
{
    // clear the horizon
    for(auto& e : elements)
    {
		_horizon.erase( e );
    }

    
    // clear the source examples
    for(auto& e : elements)
    {
		_sourceExampleMap.eraseElement( e );
    }

    
    // clear out the freespace
	_freeSpace.remove( elements );

    _rebuildFreespace();


    // clear the output
    _output.erase( elements );
}

void Algorithm::addToHorizon( std::vector<FElement*>& elements )
{
    for(auto& e : elements)
        _horizon.insert( e );
}


// -----------------------------------------------------------------------------
/// LSA
// -----------------------------------------------------------------------------

void Algorithm::_sampleNeighbourhood(FElement * element, Domain& domain, Eigen::VectorXf& vector_out)
{
    auto neighbourhood = domain.neighbours(element, element->generationParameters.radius);
    neighbourhood.push_back(element);
    
    int n = 8;
    float cellSize = element->generationParameters.radius / float(n);
    
    // eigen is column major (column by column)
    vector_out = Eigen::VectorXf(n * n * n);
    typedef Eigen::Triplet<float> Triplet;
    
    for( auto e : neighbourhood )
    {
        Vector3f p = e->position - element->position;
        p /= cellSize;
        
        vector_out(int(p.z()) * n * n + int(p.y()) * n + int(p.x())) += 1.0f;
    }
    
    
    // or
    
    for( int xi = 0; xi < n; ++xi )
    {
        for( int yi = 0; yi < n; ++yi )
        {
            for( int zi = 0; zi < n; ++zi )
            {
                
            }
        }
    }
    
}

auto Algorithm::_discretize(FElement* element, Domain& domain, const int n) -> Eigen::VectorXf
{
    // the bounding box extends from (p_e - r) to (p_e + r).
    const float r = generationParameters.radius;
    const float delta = (r * 2.0f) / float(n - 1);
    const float invDelta = 1.0f / delta;
    
    const Vector3f rvec = Vector3f(r, r, r);
    const Vector3f halfExtents = Vector3f(delta / 2.0f, delta / 2.0f, delta / 2.0f);
    
    const Vector3i toIndex = Vector3i(0, n, n * n);
    
    auto neighbourhood = domain.neighbours(element, generationParameters.radius);
    neighbourhood.push_back(element);
    
    VectorXf volume(n * n * n);
    for( int i = 0; i < n * n * n; ++i )
        volume(i) = 0.0f;
        
        for( auto e : neighbourhood )
        {
            // shift to a bounding box from (0,0,0) to (2r, 2r, 2r)
            Vector3f p = e->position - element->position + rvec;
            
            p = (p + halfExtents) * invDelta;
            
            Eigen::Vector3i ivec = p.cast<int>();
            
            int i = ivec.dot(toIndex);
            
            volume[i] += 1.0f;
        }
    
    return volume;
}

void Algorithm::_initLSA()
{
    const int n = 8;
    
    const int n3 = n * n * n;
    
    const int m = _exemplar.size();
    
    Eigen::MatrixXf A(n3, m);
    
    int j = 0;
    
    for( auto& e : _exemplar )
    {
        auto vec = _discretize(e.get(), _exemplar, n);
        
        A.col(j) = vec;
        
        j++;
    }
    
    Eigen::JacobiSVD<MatrixXf> svd(A, ComputeThinU | ComputeThinV);
    
    const int k = 5;
    
    VectorXf singularValues = svd.singularValues();
    
    MatrixXf U_k = svd.matrixU().leftCols<k>();
    VectorXf singularValues_k = singularValues.head<k>();
    MatrixXf V_k = svd.matrixV().leftCols<k>();
    
    MatrixXf A_k = U_k * singularValues_k.asDiagonal() * V_k.transpose();
    
    VectorXf q = A.col(0);
    VectorXf answer = A.transpose() * q;
    
    VectorXf singularValuesInverse_k(k);
    for( int i = 0; i < k; ++i )
        singularValuesInverse_k(i) = singularValues_k(i) != 0.0f ? 1.0f / singularValues_k(i) : 0.0f;
    
    
    queryTransformMatrix = U_k * singularValuesInverse_k.asDiagonal();
    lowDimensionalNeighbourhoodMatrix = V_k;
    
    VectorXf q_k = q.transpose() * queryTransformMatrix;
    VectorXf answer_k = lowDimensionalNeighbourhoodMatrix * q_k;
    
    
    
    for( int i = 0; i < answer.size(); ++i )
        UE_LOG(LogTemp, Warning, TEXT("%d %f"), i, answer(i));
    
    for( int i = 0; i < answer_k.size(); ++i )
        UE_LOG(LogTemp, Warning, TEXT("%d %f"), i, answer_k(i));
    
    std::stringstream out;
    
    out << "U_k\n" << U_k << "\nV_k---\n" << V_k << "\nS_k---\n" << singularValues_k << "\nA_k---\n" << A_k;
    
    FString fString(out.str().c_str());
    
    UE_LOG(LogTemp, Warning, TEXT("%s"), *fString);
    
    return;
}

void Algorithm::setOcclusionTester( std::shared_ptr<OcclusionBase> const& occlusionTester )
{
	if( occlusionTester == nullptr )
		_occlusionTester = std::make_shared<NoOcclusion>();
	else
		_occlusionTester = occlusionTester;
}

void Algorithm::init( std::shared_ptr<ctpl::thread_pool> threadPool )
{
	if(threadPool)
		setThreadPool( threadPool );
	else
		_conditionally_initializeThreadPool();
}


void Algorithm::loadExemplar()
{

}

void Algorithm::setMeshInterface( std::shared_ptr<tcodsMeshInterface> meshInterface )
{
	_meshInterface = meshInterface;
}

void Algorithm::setToMeshTransform( const Eigen::Affine3f& transform )
{
	_toMeshTransform = transform;
}

// -----------------------------------------------------------------------------
/// Generation
// -----------------------------------------------------------------------------

AlgorithmResult Algorithm::generate( std::vector<PositionFace>& startingPositions, float radius, AABB limits)
{
	return AlgorithmResult();
}

void Algorithm::_expandFreeSpace(std::vector<FElement*>& generated)
{
    // update the freespace index
    for( FElement * element : generated ) 
    {
        auto& position = element->position;
        
		auto& sourceExamples = _sourceExampleMap.sourceExamples( element );
		for(SourceExampleMap::SourceExample& sourceExample : sourceExamples)
		{
			FElement * exemplarElement = sourceExample.element;

			auto mapping = _mapping( *element, *exemplarElement );

			float searchRadiusSqrd = freespaceRadius * freespaceRadius;

			auto points = _freeSpace.freespacePoints( exemplarElement );
			for(Eigen::Vector3f& point : points)
			{
				// we need to project to the plane and offset the positions
				auto onSurface = mapping->toSurface( point );

				if(!onSurface.hit)
					continue;

				Eigen::Vector3f& onSurfacePoint = onSurface.position;

				// check for overlap
				auto nearest = _output.nearest( onSurfacePoint, freespaceRadius );

				if(nearest.element == nullptr)
				{
					_freeSpace.addOutputPoint( onSurfacePoint );
				}
			}

			// remove any free-space points that overlap with the generated element
			auto freespaceIndices = _freeSpace.outputIndicesInRadius( position, freespaceRadius );

			for(auto& indicesPair : freespaceIndices)
				_freeSpace.removeOutputPoint( indicesPair.first );
		}
    }
}

bool Algorithm::_overlaps(const Eigen::Vector3f& position, const float radius, FNeighbourhoodParameters& parameters, const FElement* elementToIgnore /*= nullptr*/)
{
    // we have to use _maxRadius for the query, since our radius and neighbours radius may be different
    //    float maxAllowedRadius = (radius + _maxRadius) / relaxation;
    float searchRadius = parameters.radius;
    
    auto neighbours = _output.nearestInRadius(position, searchRadius);
    
    for( FElement* neighbour : neighbours )
    {
        if( elementToIgnore == neighbour )
            continue;
        
        float allowedRadius = (radius + neighbour->radius) * relaxation;
        
        float distance = (neighbour->position - position).norm();
        if( distance < allowedRadius  )
            return true;
    }
    
    return false;
}

bool Algorithm::_inBoundary(const Vector3f& position, const float radius)
{
    // check boundary conditions
    if( generationMode == EGenerationMode::SpaceFilling )
    {
		if(seedsIgnoreMeshInBoundary)
			return true;

        auto nearest = _meshInterface->nearestPointOnMesh(unreal(position));
        
        float radiusSquared = radius * radius;
        Eigen::Vector3f nearestPoint = eigen(nearest.point);
        if( (nearestPoint - position).squaredNorm() < radiusSquared )
            return false;
        
        // look up the face and see if its normal is pointing away from us
        auto face = _meshInterface->mesh.face(nearest.faceIndex);
        auto normal = eigen( face->normal() );

        Eigen::Vector3f direction = eigen(nearest.point) - position;
        
        direction.normalize();
        
        if( flipSurfaceNormals )
            direction = direction * -1.0f;
        
        if( normal.dot(direction) >= 0.0f )
            return false;
    }
    else if( generationMode == EGenerationMode::SurfacePainting || generationMode == EGenerationMode::SpacePainting )
    {
		if(_brushPoints.pts.size() == 0)
			return false;
       
		size_t foundIndex;
		float dSqrd;
		
		_brushIndex->knnSearch( &position.x(), 1, &foundIndex, &dSqrd );

		auto brushPoint = _brushPoints.pts[foundIndex];
		float brushSizeSqrd = brushPoint.radius * brushPoint.radius;

		if(dSqrd >= brushSizeSqrd)
			return false;
    }

	// check for overlaps with the void index
	if( _voidPoints.pts.size() )
	{
		size_t foundIndex;
		float dSqrd;

		_voidIndex->knnSearch( &position.x(), 1, &foundIndex, &dSqrd );

		float d = std::sqrtf( dSqrd );

		if(d < voidSize + radius)
			return false;
	}

    return true;
}

/**
 Marks elements as frozen if they do not have any right-partial-pairings.
 */
std::set<FElement*> Algorithm::_removeFrozen(const std::set<FElement*>& elements, Domain& domain, std::vector<FElement*>& frozen_out)
{
    std::set<FElement*> result;
    
    for( FElement * e : elements )
    {
        auto eNeighbours = domain.neighbours(e, e->generationParameters.radius, e->generationParameters.kNearest);
        
        unsigned int rightPairingsCount = 0;
        
		auto& sourceExamples = _sourceExampleMap.sourceExamples( e );

		bool foundRightPairings = false;

		for(SourceExampleMap::SourceExample& sourceExample : sourceExamples)
		{
			auto& coherentNeighbours = sourceExample.kCoherentNeighbours( *_defaultSelection );

			auto mapping = _mapping( *e );


			for(FElement * candidate : coherentNeighbours)
			{
				if(e->type != candidate->type)
					continue;

				mapping->setExemplarOrigin( candidate->position );

				auto candidateNeighbours = _exemplar.neighbours( candidate, candidate->generationParameters.radius, candidate->generationParameters.kNearest );

				SimilarElement similar = distance( *e, eNeighbours, *candidate, candidateNeighbours, mapping.get() );

				if(similar.rightPartialPairings.size())
				{
					result.insert( e );
					foundRightPairings = true;
					break;
				}
			}

			if(foundRightPairings)
				break;
		}
		
		if(!foundRightPairings)
			frozen_out.push_back( e );
    }
    
    return result;
}

std::set<FElement*> Algorithm::_removeFrozen_usingFreespacePoints(const std::set<FElement*>& elements, Domain& domain, std::vector<FElement*>& frozen_out)
{
    std::set<FElement*> result;
    
    for( FElement * e : elements )
    {
		auto freespaceIndices = _freeSpace.outputIndicesInRadius( e->position, e->generationParameters.radius );

        if( freespaceIndices.size() == 0 && _canRemoveFromHorizon(e) )
            frozen_out.push_back(e);
        else
            result.insert(e);
    }
    
    return result;
}

std::vector<FElement*> Algorithm::_seeds(const std::set<FElement*>& elements_in)
{
    vector<FElement*> result;
    
    if( elements_in.size() == 0 )
        return result;
    
    std::vector<FElement*> elements;


	std::copy( elements_in.begin(), elements_in.end(), std::back_inserter( elements ) );
    
    // build a local kd-tree for the elements
    EigenPointCloud cloud;

	EigenDynamicPointCloudIndexAdaptor index( 3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams( 10 ) );

    
    for( FElement* e : elements )
    {
        const auto p = e->position;
        cloud.pts.emplace_back(p);
    }

	if(cloud.pts.size() > 0 )
		index.addPoints( 0, cloud.pts.size() - 1 );
    
    // index.buildIndex();
    
    unordered_set<FElement*> available;
    for( FElement * e : elements )
        available.insert(e);
    
    FElement * e = elements[0];
    
	int k = e->generationParameters.kNearest;

	std::vector<size_t> indices( k );
	std::vector<float> distances( k );
    

    
    while( available.size() > 0 )
    {
        result.push_back(e);
        available.erase(e);
        
        Vector3f p = e->position;
 
		k = e->generationParameters.kNearest;

		// the std spec says that capacity will not be reduced, but the size will (we want this behavior for performance avoid too many allocations
		indices.resize( k ); 
		distances.resize( k );
        
        nanoflann::KNNResultSet<float,size_t> resultSet(k);
        resultSet.init(&indices[0], &distances[0]);
        index.findNeighbors(resultSet, &p(0), nanoflann::SearchParams());
        
        FElement * next = nullptr;
        
        // remove all the elements in the interval between this and the seedSeparation'th element
        int neighbourIndex = 1; // skip e
        for( int i = 0; i < seedSeparation && neighbourIndex < resultSet.size(); ++i, ++neighbourIndex )
        {
            assert(neighbourIndex < k);
            
            int indicesIndexOfNeighbour = indices[neighbourIndex];
            
            FElement * neighbour = elements.at(indicesIndexOfNeighbour);//[indices[neighbourIndex]];
            
            if( available.find(neighbour) != available.end() )
            {
                available.erase(neighbour);
                next = neighbour;
            }
        }
        
        if( next != nullptr )
            e = next;
        else if( available.size() > 0 )
            e = *available.begin();
        
        
    }
    

    
    if( result.size() )
    {
        _orderNearest(result, result[0]->position);
        return result;
    }
    else
        return result;
}

void Algorithm::_orderNearest(std::vector<FElement*>& elements, const Vector3f referencePoint)
{
    sort(elements.begin(), elements.end(), [referencePoint](const FElement * a, const FElement * b)
         {
             return (a->position - referencePoint).squaredNorm() < (b->position - referencePoint).squaredNorm();
         });
}

// -----------------------------------------------------------------------------
/// Distance
// -----------------------------------------------------------------------------

Algorithm::SimilarElement Algorithm::distance_densityBased
( 
	FElement& element, std::vector<FElement*>& elementNeighbours,
	FElement& example, std::vector<FElement*>& exampleNeighbours,
	rg::Mapping* mapping 
)
{
	int elementsSize = elementNeighbours.size();
	int examplesSize = exampleNeighbours.size();

	vector<Vector3f> elementPredictions( elementsSize );
	vector<Vector3f> examplePredictions( examplesSize );

	const Vector3f examplePosition = example.position;
	const Vector3f elementPosition = element.position;

	bool workInExemplarSpace = mapping->compareInExemplarSpace();

	if(workInExemplarSpace)
	{
		for(int i = 0; i < elementsSize; ++i)
			elementPredictions[i] = mapping->toExemplarVector( elementNeighbours[i]->position - elementPosition );

		for(int i = 0; i < examplesSize; ++i)
			examplePredictions[i] = exampleNeighbours[i]->position - examplePosition;
	}
	else
	{
		for(int i = 0; i < elementsSize; ++i)
			elementPredictions[i] = elementNeighbours[i]->position;

		for(int i = 0; i < examplesSize; ++i)
		{
			auto surface = mapping->toSurface( exampleNeighbours[i]->position );
			examplePredictions[i] = surface.position;
		}
	}

	vector<pair<int, int>> indexPairs;
	float cost = 0.0f;

	for(int i = 0; i < elementsSize; ++i)
	{
		const Vector3f& p = elementPredictions[i];

		for(int j = 0; j < examplesSize; ++j)
		{
			const Vector3f& p_ = examplePredictions[j];

			// https://en.wikipedia.org/wiki/Multivariate_normal_distribution ?
		}
	}




	SimilarElement result;
	result.element = &example;
	result.cost = cost;

	// convert pairs of indices to element pairings
	// --------------------------------------------
	auto& fullPairings = result.fullPairings;
	auto& leftPartialPairings = result.leftPartialPairings;
	auto& rightPartialPairings = result.rightPartialPairings;


	vector<bool> assignedElements( elementsSize );
	vector<bool> assignedExamples( examplesSize );

	float minDistanceSquared = minAssignmentDistance * minAssignmentDistance;

	for(auto pair : indexPairs)
	{
		if(pair.first < 0 || pair.second < 0)
			break;

		FElement * e = elementNeighbours[pair.first];
		FElement * ex = exampleNeighbours[pair.second];

		Vector3f p_e = elementPredictions[pair.first];
		Vector3f p_ex = examplePredictions[pair.second];

		if((p_e - p_ex).squaredNorm() < minDistanceSquared)
		{
			fullPairings.emplace_back( e, ex );

			assignedElements[pair.first] = true;
			assignedExamples[pair.second] = true;
		}
	}

	for(int i = 0; i < elementsSize; ++i)
	{
		if(assignedElements[i])
			continue;

		leftPartialPairings.emplace_back( elementNeighbours[i], nullptr );
	}

	for(int i = 0; i < examplesSize; ++i)
	{
		if(assignedExamples[i])
			continue;

		rightPartialPairings.emplace_back( nullptr, exampleNeighbours[i] );
	}



	return result;
}

Algorithm::SimilarElement Algorithm::distance_hungarian( FElement& element, std::vector<FElement*>& elementNeighbours, FElement& example, std::vector<FElement*>& exampleNeighbours, rg::Mapping* mapping )
{
	SimilarElement result;

	return result;
}

void Algorithm::beginRound()
{
	RoundRecord& record = roundSummary().emplace();

	record.start = std::clock();
}

void Algorithm::endRound( AlgorithmResult& result )
{
	RoundRecord& record = roundSummary().back();

	record.end = std::clock();

	record.elementsGenerated = result.generated.size();
	record.elementsRemoved = result.removed.size();
	record.elementsModified = result.modified.size();

	record.elementsTotal = output().size();

	if(enableRoundSummaries)
	{
		if(_output.size() && calculate_kCoherenceEnergy)
		{
			record.totalEnergy_kCoherence = totalEnergy_kCoherence( optimizationParameters );
			record.averageEnergy_kCoherence = record.totalEnergy_kCoherence / float( _output.size() );
		}

		if(_output.size() && calculate_bruteForceEnergy)
		{
			record.totalEnergy_bruteForce = totalEnergy_bruteForce( optimizationParameters );
			record.averageEnergy_bruteForce = record.totalEnergy_bruteForce / float( _output.size() );
		}
	}
}

auto Algorithm::totalEnergy_kCoherence( FNeighbourhoodParameters parameters ) -> double
{
	std::vector< std::future<float> > futures;

	for(auto& e : _output)
	{
		FElement * element = e.get();

		futures.emplace_back( _threadPool->push( [this, parameters, element]( int threadID ) mutable {
			float cost = 0.0f;
			
			auto& sourceExamples = _sourceExampleMap.sourceExamples( element );

			for(auto& sourceExample : sourceExamples)
			{
				auto& coherentNeighbours = sourceExample.kCoherentNeighbours( *_defaultSelection );

				std::vector<FElement*> generative;
				for(FElement * e : coherentNeighbours)
				{
					if(e->generative)
						generative.push_back( e );
				}

				// auto nearestVec = _nearestSimilarElements( element, _output, generative, _exemplar, parameters );
				if(useCinpactEnergy)
				{
					auto nearestVec = _nearestElements_discreteCinpactSimilarity( *element, _output, generative, _exemplar );

					if(nearestVec.size())
						cost += nearestVec[0].cost;
					else
						cost += 0.0f;
				}
				else
				{
					auto nearestVec = _nearestSimilarElements( element, _output, generative, _exemplar, parameters );

					if(nearestVec.size())
						cost += nearestVec[0].cost;
					else
						cost += 0.0f;
				}
			}
			
			return cost;
		} ) );
	}

	double energy = 0.0;

	for(auto& f : futures)
		energy += f.get();

	return energy;
}

auto Algorithm::totalEnergy_bruteForce( FNeighbourhoodParameters parameters ) -> double
{
	std::vector< std::future<float> > futures;

	for(auto& e : _output)
	{
		FElement * element = e.get();

		futures.emplace_back( _threadPool->push( [this, parameters, element]( int threadID ) mutable {
			if(useCinpactEnergy)
			{
				auto nearestVec = _nearestElements_discreteCinpactSimilarity( *element, _output, _defaultSelection->_generativeElements, _exemplar );

				if(nearestVec.size())
					return nearestVec[0].cost;
				else
					return 0.0f;
			}
			else
			{
				auto nearestVec  = _nearestSimilarElements( element, _output, _defaultSelection->_generativeElements, _exemplar, parameters );

				if(nearestVec.size())
					return nearestVec[0].cost;
				else
					return 0.0f;
			}
		} ) );
	}

	double energy = 0.0;

	for(auto& f : futures)
		energy += f.get();

	return energy;
}

Algorithm::SimilarElement Algorithm::distance(FElement& element, std::vector<FElement*>& elementNeighbours,
                                              FElement& example, std::vector<FElement*>& exampleNeighbours,
                                              rg::Mapping* mapping)
{
    int elementsSize = elementNeighbours.size();
    int examplesSize = exampleNeighbours.size();
    
    vector<Vector3f> elementPredictions(elementsSize);
    vector<Vector3f> examplePredictions(examplesSize);
    
    const Vector3f examplePosition = example.position;
    const Vector3f elementPosition = element.position;
    
	bool workInExemplarSpace = mapping->compareInExemplarSpace();

	if(workInExemplarSpace)
	{
		for(int i = 0; i < elementsSize; ++i)
			elementPredictions[i] = mapping->toExemplarVector( elementNeighbours[i]->position - elementPosition );

		for(int i = 0; i < examplesSize; ++i)
			examplePredictions[i] = exampleNeighbours[i]->position - examplePosition;
	}
	else
	{
		for(int i = 0; i < elementsSize; ++i)
			elementPredictions[i] = elementNeighbours[i]->position;

		for(int i = 0; i < examplesSize; ++i)
		{
			auto surface = mapping->toSurface( exampleNeighbours[i]->position);
			examplePredictions[i] = surface.position;
		}
	}
    
    vector<pair<int, int>> indexPairs;
    float cost = 0.0f;
    
	if(element.type != example.type)
		cost += typeCost;

    // this is where we would normally do ICP
    cost = _closestPoint(elementPredictions, elementNeighbours, examplePredictions, exampleNeighbours, indexPairs);
    
    SimilarElement result;
    result.element = &example;
    result.cost = cost;
    
    // convert pairs of indices to element pairings
    // --------------------------------------------
    auto& fullPairings = result.fullPairings;
    auto& leftPartialPairings = result.leftPartialPairings;
    auto& rightPartialPairings = result.rightPartialPairings;
    
    
    vector<bool> assignedElements(elementsSize);
    vector<bool> assignedExamples(examplesSize);
    
    float minDistanceSquared = minAssignmentDistance * minAssignmentDistance;
    
    for( auto pair : indexPairs )
    {
        if( pair.first < 0 || pair.second < 0 )
            break;
        
        FElement * e = elementNeighbours[pair.first];
        FElement * ex = exampleNeighbours[pair.second];
        
        Vector3f p_e = elementPredictions[pair.first];
        Vector3f p_ex = examplePredictions[pair.second];
        
        if( (p_e - p_ex).squaredNorm() < minDistanceSquared )
        {
            fullPairings.emplace_back(e, ex);
            
            assignedElements[pair.first] = true;
            assignedExamples[pair.second] = true;
        }
    }
    
    for( int i = 0; i < elementsSize; ++i )
    {
        if( assignedElements[i] )
            continue;
        
        leftPartialPairings.emplace_back(elementNeighbours[i], nullptr);
    }
    
    for( int i = 0; i < examplesSize; ++i )
    {
        if( assignedExamples[i] )
            continue;
        
        rightPartialPairings.emplace_back(nullptr, exampleNeighbours[i]);
    }
    
    
    
    return result;
}

float Algorithm::_closestPoint(const std::vector<Vector3f>& aPositions, const std::vector<FElement*>& aNeighbours,
                               const std::vector<Vector3f>& bPositions, const std::vector<FElement*>& bNeighbours,
                               std::vector<std::pair<int,int>>& pairings)
{
    float cost = 0.0f;
    
    int aSize = aPositions.size();
    int bSize = bPositions.size();
    
    pairings.clear();
    pairings.reserve(aSize);
    
    float thresholdDistanceSquared = minAssignmentDistance * minAssignmentDistance;
    
    struct FastIndexSetIndex
    {
        int index;      // the index at this position
        int runStart;   // the start of a run of indices
    };
    std::vector<FastIndexSetIndex> indices(bSize + 1);
    
    for( int i =0; i < bSize + 1; ++i )
        indices[i] = {i, i};
    
    for( int aIndex = 0; aIndex < aSize; ++aIndex )
    {
        const Vector3f& aPosition = aPositions[aIndex];
        FElement * aElement = aNeighbours[aIndex];
        
        FElement * bNearest = nullptr;
        int bNearestIndex = -1;
        float minDSquared = numeric_limits<float>::max();
        
        
        int bIndex = indices[0].index;
        while( bIndex < bSize )
        {
            FElement * bElement = bNeighbours[bIndex];
            const Vector3f& bPosition = bPositions[bIndex];
            
            float dSquared = (aPosition - bPosition).norm();
            
            // add in the cost of unmatched types
            if( aElement->type != bElement->type )
                dSquared += typeCost * typeCost;
            
            if( dSquared < minDSquared )
            {
                minDSquared = dSquared;
                bNearest = bElement;
                bNearestIndex = bIndex;
            }
            
            bIndex = indices[bIndex + 1].index;
        }
        
        if( bNearest != nullptr )
        {
            // erase an index
            static FastIndexSetIndex nullIndex = {-1, -1};
            
            auto& bPrevious = bNearestIndex - 1 >= 0 ? indices[bNearestIndex - 1] : nullIndex;
            auto& bNext = indices[bNearestIndex + 1];
            auto& bCur = indices[bNearestIndex];
            
            bCur.index = bNext.index;
            
            if( bPrevious.index == bNearestIndex )
            {
                indices[bCur.runStart].index = bNext.index;
                indices[bNext.index].runStart = bCur.runStart;
            }
            else
                indices[bCur.index].runStart = bNearestIndex;
            
            pairings.emplace_back(aIndex, bNearestIndex);
            
            cost += sqrt(minDSquared);
            
            if( aElement->type != bNearest->type )
                cost += typeCost;
        }
        else
            pairings.emplace_back(aIndex, -1);
    }
    
    return cost;
}

std::vector<Algorithm::SimilarElement> Algorithm::_nearestSimilarElements
(
	FElement* element, Domain& elementDomain,
	std::vector<FElement*> candidates, Domain& candidateDomain,
	const FNeighbourhoodParameters& searchParameters,
	bool forceSpaceFilling,
	bool filterCandidateByType
)
{
    vector<Algorithm::SimilarElement> results;
    
    auto elementNeighbours = elementDomain.neighbours(element, searchParameters.radius, searchParameters.kNearest, true, nullptr, enableVolumeSurfaceInteraction );

    auto mapping = _mapping(*element, forceSpaceFilling);
    
    for( FElement * candidate : candidates )
    {
        if( element->type != candidate->type && filterCandidateByType)
            continue; 
        
        mapping->setExemplarOrigin(candidate->position);
        
        auto candidateNeighbours = candidateDomain.neighbours(candidate, searchParameters.radius, searchParameters.kNearest);
        
        SimilarElement similar = distance(*element, elementNeighbours, *candidate, candidateNeighbours, mapping.get());
        
        results.push_back(similar);
    }
    
    sort(results.begin(), results.end(), [](const SimilarElement& a, const SimilarElement& b) -> bool
    {
        return a.cost < b.cost;
    });
    
    return results;
}

std::vector<Algorithm::SimilarElement> Algorithm::_generation_nearestSimilarElements(FElement* element, Domain& elementDomain,
                                                                                     std::vector<FElement*> candidates, Domain& candidateDomain,
                                                                                     const FNeighbourhoodParameters& searchParameters)
{
    vector<Algorithm::SimilarElement> results;
    
    float radius = std::max(searchParameters.radius, sourceHistogramRadius);
    
    auto neighbours = elementDomain.neighbours(element, radius, -1);
    
    auto histogram = _histogram(element, neighbours);
    
    // we only need to run one neighbourhood query, we can extract the element neighbourhood from
    // this query
    vector<FElement*> elementNeighbours;
    elementNeighbours.reserve(searchParameters.kNearest);
    
    float searchRadiusSqrd = searchParameters.radius * searchParameters.radius;
    auto elementPosition = element->position;
    
    for( int i = 0; i < searchParameters.kNearest && i < neighbours.size(); i++ )
    {
        float d = (neighbours[i]->position - elementPosition).squaredNorm();
        
        if( d < searchRadiusSqrd )
            elementNeighbours.push_back(neighbours[i]);
    }
    
    auto mapping = _mapping(*element);
    
    for( FElement * candidate : candidates )
    {
		// ignore element's that don't match on id if we are painting
        if( element->type != candidate->type && generationMode != EGenerationMode::SurfacePainting )
            continue;
        
        mapping->setExemplarOrigin(candidate->position);
        
        auto candidateNeighbours = candidateDomain.neighbours(candidate, searchParameters.radius, searchParameters.kNearest);
        
        SimilarElement similar = distance(*element, elementNeighbours, *candidate, candidateNeighbours, mapping.get());
        
         
        
        // -----------------
        // Sampling distance
        float samplingDistance = 0.0f;
        
        for( FElement * e : candidateNeighbours )
        {
            if( histogram.find(e) == histogram.end() )
                continue;
            
            float dist = (e->position - candidate->position).norm();
            
            if( dist > sourceHistogramRadius )
                continue;
            
            dist += sourceHistogramRadius - dist;
            
            samplingDistance += dist * histogram[e];
        }
        
        // ---------------
        // Local Histogram
        std::map<int,int> localHistogram;
        
        for( FElement * e : elementNeighbours )
            localHistogram[e->type]++;
        
        localHistogram[element->type]++;
        int localHistogramCount = elementNeighbours.size() + 1 + similar.rightPartialPairings.size();
        
        for( auto& pair : similar.rightPartialPairings )
            localHistogram[pair.second->type]++;
        
        float typeHistogramTerm = _compareHistograms(_exemplarTypeHistogram, _exemplar.size(),
                                                     localHistogram, localHistogramCount);
        
        similar.cost += samplingDistance * samplingDistanceWeight + typeHistogramTerm * sourceHistogramWeight;
        
        results.push_back(similar);
    }
    
    sort(results.begin(), results.end(), [](const SimilarElement& a, const SimilarElement& b) -> bool
         {
             return a.cost < b.cost;
         });
    
    return results;
}


std::vector<Algorithm::SimilarElement> Algorithm::_lsa_nearestSimilarElements(FElement* element, Domain& elementDomain, const FNeighbourhoodParameters& searchParameters)
{
    vector<Algorithm::SimilarElement> results;
    
    //    Eigen::VectorXf q = _discretize(element, elementDomain, 8);
    //
    //    Eigen::VectorXf q_k = q.transpose() * queryTransformMatrix;
    //    Eigen::VectorXf answer_k = lowDimensionalNeighbourhoodMatrix * q_k;
    //
    //    std::vector<size_t> indices(answer_k.size());
    //    for( int i = 0; i < indices.size(); ++i )
    //        indices[i] = i;
    //
    //    std::sort(indices.begin(), indices.end(),
    //              [&answer_k](size_t a, size_t b) {
    //                  return answer_k[a] > answer_k[b];
    //              });
    //
    //    for( size_t i : indices )
    //    {
    //        results.emplace_back();
    //
    //        SimilarElement& result = results.back();
    //
    //        result.element = _exemplar.elementAt(i);
    //        result.cost
    //
    //    }
    //
    //    vector<Algorithm::SimilarElement> results;
    
    return results;
}


// -----------------------------------------------------------------------------
/// Freespace
// -----------------------------------------------------------------------------

void Algorithm::_rebuildFreespace()
{
	_freeSpace.rebuild();
}

std::vector<FElement*> Algorithm::_fastFilterGenerativeExemplars(FElement * seed, Freespace& freespace)
{
    std::vector<FElement*> result;

    // find the nearby freespace points
	std::vector< Eigen::Vector3f > freePoints = freespace.outputPointsInRadius( seed->position, seed->generationParameters.radius );
    
    // mapping
    auto mapping = _mapping(*seed);

	if(mapping->compareInExemplarSpace())
	{
		// remap the free points
		for( Eigen::Vector3f& point : freePoints )
			point = mapping->toExemplar( point );

		const float freespaceRadiusSqrd = freespaceRadius * freespaceRadius;

		auto& sourceExamples = _sourceExampleMap.sourceExamples( seed );
		for(SourceExampleMap::SourceExample& sourceExample : sourceExamples)
		{
			auto& candidates = sourceExample.kCoherentNeighbours( *_defaultSelection );

			for(FElement * exemplar : candidates)
			{
				int generativeCount = 0;

				// check for overlaps with freepoints, if there are, then this exemplar will generate elements
				for(auto& freePoint : freePoints)
				{
					auto point = freePoint + exemplar->position;

					auto nearest = _exemplar.nearest( point, seed->generationParameters.radius );

					if(nearest.element)
						generativeCount++;
				}

				if(generativeCount > 0)
					result.push_back( exemplar );
			}
		}
	}
	else
	{
		// get the relevant freespace points in the cache
		const float generationRadiusSqrd = seed->generationParameters.radius * seed->generationParameters.radius;

		auto& sourceExamples = _sourceExampleMap.sourceExamples( seed );
		for(SourceExampleMap::SourceExample& sourceExample : sourceExamples)
		{
			auto& candidates = sourceExample.kCoherentNeighbours( *_defaultSelection );

			// now check if any free space points overlap with an exemplar
			for(FElement * exemplar : candidates)
			{
				mapping->setExemplarOrigin( exemplar->position );

				auto neighbours = _exemplar.neighbours( exemplar, seed->generationParameters.radius, -1, false );

				int generativeCount = 0;

				for(auto ex_ : neighbours)
				{
					auto exemplarPoint = mapping->toSurface( ex_->position );

					if(!exemplarPoint.hit)
						continue;

					// check for overlaps with freepoints, if there are, then this exemplar will generate elements
					for(auto& freePoint : freePoints)
					{
						float d = (freePoint - exemplarPoint.position).squaredNorm();

						if(d < generationRadiusSqrd)
						{
							generativeCount++;
							break;
						}
					}

					if(generativeCount)
						break;
				}

				if(generativeCount > 0)
					result.push_back( exemplar );
			}
		}
	}
    
    return result;
}

// -----------------------------------------------------------------------------
/// Histogram
// -----------------------------------------------------------------------------

float Algorithm::_sourceHistogramCost(const std::vector<FElement*>& outputElements, const std::vector<FElement*>& exemplarElements)
{
    map<FElement*, int> histogram;
     
    for( FElement * e : outputElements )
    {
		auto& sourceExamples = _sourceExampleMap.sourceExamples( e );

		for(auto& sourceExample : sourceExamples)
		{
			FElement * source = sourceExample.element;

			if(histogram.find( source ) == histogram.end())
				histogram[source] = 1;
			else
				histogram[source]++;
		}
    }
    
    float cost = 0.0f;
    
    for( FElement* e_ : exemplarElements )
    {
        if( histogram.find(e_) != histogram.end() )
            cost += histogram[e_];
    }
    
    return cost;
}

std::map<FElement*, int> Algorithm::_histogram(FElement * element, std::vector<FElement*>& neighbours)
{
    std::map<FElement*, int> histogram;
    
    for( FElement * e_ : neighbours )
    {
		auto& sourceExamples = _sourceExampleMap.sourceExamples( e_ );

		for(auto& sourceExample : sourceExamples)
		{
			histogram[sourceExample.element]++;
		}
    }
    
    return histogram;
}

float Algorithm::_compareHistograms(std::map<int,int> histogramA, int countA_in,
                                    std::map<int,int> histogramB, int countB_in)
{
    float distance = 0.0f;
    
    for( auto& pair : histogramA )
    {
        int countA = pair.second;
        int countB = histogramB.find(pair.first) == histogramB.end() ? 0 : histogramB[pair.first];
        
        float ratioA = float(countA) / float(countA_in);
        float ratioB = float(countB) / float(countB_in);
        
        distance += std::abs(ratioA - ratioB);
    }
    
    return distance;
}

// -----------------------------------------------------------------------------
/// Optimization
// -----------------------------------------------------------------------------

AlgorithmResult Algorithm::horizonOptimization()
{
    AlgorithmResult result;
    
	OptimizationProblem problem;
	problem.activeElements = _horizon;

	problem.frozenElements = _nearbyFrozenElements( problem.activeElements );
	
	{
		for(auto& e : _output)
			problem.elements.insert( e.get() );
	}
    _localOptimization( problem, result);
    
    _output.rebalance();
    
    return result;
}

AlgorithmResult Algorithm::globalOptimization()
{
    AlgorithmResult result;
    
	OptimizationProblem problem;

	{
		for(auto& e : _output)
			problem.elements.insert( e.get() );
	}

	problem.activeElements = problem.elements;
	problem.frozenElements = problem.elements;

	_localOptimization( problem, result );
    
    _output.rebalance();
    
    return result;
}

void Algorithm::clear()
{
	clearOutput();
	clearPainting();

	_exemplar.clear();

	_defaultSelection->clear();
	_exampleSelections.clear();

	_didInit = false;
}

void Algorithm::clearOutput()
{
	roundSummary().clear();

	_horizon.clear();
	_output.clear();
	_sourceExampleMap.clear();

	_initFreespacePoints();
}

void Algorithm::clearPainting()
{
	_brushPoints.pts.clear();
	_initBrushIndex();

	_voidPoints.pts.clear();
	_initVoidIndex();
}

template<typename MatrixType>
MatrixType pinv(MatrixType& inMatrix)
{
    using namespace Eigen;
    Eigen::JacobiSVD<MatrixType> svd(inMatrix, ComputeFullU | ComputeFullV);
    
    auto singularValues_inv = svd.singularValues();
    
    double tolerance = 1.e-4;
    
    for( int i = 0; i < inMatrix.cols(); ++i )
        singularValues_inv(i) = singularValues_inv(i) > tolerance ? 1.0 / singularValues_inv(i) : 0.0;
    
    MatrixType returnValue = (svd.matrixV() * singularValues_inv.asDiagonal() * svd.matrixU().transpose());
    
    return returnValue;
}

std::vector<FElement*> _filter( std::vector<FElement*> original, std::unordered_set<FElement*> toRemove )
{
	std::vector<FElement*> result;

	for(FElement * e : original)
	{
		if(toRemove.find( e ) == toRemove.end())
			result.push_back( e );
	}

	return result;
}

std::set<FElement*> Algorithm::_nearbyFrozenElements(const std::set<FElement*>& activeElements)
{
	std::set<FElement*> frozen;

	for(auto e : activeElements)
	{
		auto sphere = _sphereOfInfluence( e, frozenElementRadius );

		for(FElement * inSphere : sphere)
		{
			if(activeElements.find( inSphere ) == activeElements.end())
				frozen.insert( inSphere );
		}
	}

	return frozen;
}


std::vector<Algorithm::PredictionNeighbourhood> Algorithm::_predictionNeighbourhoods( std::set<FElement *>& elements )
{
	// reserve space in the results vector
	size_t n = 0;

	for(FElement * e : elements)
	{
		auto& examples = _sourceExampleMap.sourceExamples( e );

		n += examples.size();
	}

	std::vector<PredictionNeighbourhood> results( n );

	vector< future< void > > futures;

	size_t i = 0;
	for(FElement * e : elements)
	{
		auto& examples = _sourceExampleMap.sourceExamples( e );

		for(SourceExampleMap::SourceExample& sourceExample : examples)
		{
			futures.emplace_back( _threadPool->push( [this, e, &sourceExample, i, &results]( int threadID ) {
				// get the nearest element
				auto& coherentNeighbours = sourceExample.kCoherentNeighbours( *_defaultSelection );

				auto nearestVec = _nearestSimilarElements( 
					e, _output, 
					coherentNeighbours, _exemplar, 
					e->optimizationParameters,
					false, // forceSpaceFilling
					false // filterCandidatesByType
				);

				PredictionNeighbourhood result;

				if(nearestVec.size() == 0)
				{
					result.element = e;

					return;
				}

				result.element = e;
				result.similarNeighbourhood = nearestVec[0];
				result.weight = sourceExample.weight;

				result.exampleSelection = sourceExample.selection.lock();

				results[i] = result;
			} ) );

			++i;
		}
	}

	for(auto& f : futures)
	{
		f.wait();
	}

	return results;
}

void Algorithm::_buildPredictions(
	const std::vector<PredictionNeighbourhood>& predictionNeighbourhoods,
	const std::set<FElement *>& limitPredictionsTo, 
	std::map<FElement *, unsigned int>& problem, 
	std::vector< std::vector< PredictedPosition > >& predictions_in_out
)
{
	for(const PredictionNeighbourhood& f : predictionNeighbourhoods)
	{
		if(f.element == nullptr)
			continue;

		if(f.similarNeighbourhood.element == nullptr)
			continue;

		FElement * element = f.element;
		FElement * example = f.similarNeighbourhood.element;

		const size_t i = problem[element];

		auto& pairings = f.similarNeighbourhood.fullPairings;

		auto mapping = _mapping( *element, *example );

		for(auto& pair : pairings)
		{
			FElement * element_ = pair.first;
			const FElement * example_ = pair.second;

			// only create predictions for the active elements
			if(limitPredictionsTo.find( element_ ) == limitPredictionsTo.end())
				continue;

			if(problem.find( element_ ) == problem.end())
				continue;

			unsigned int j = problem[element_];

			// this is where we have to walk backwards on the integral curve
			// this is going to involve a shortest-path calculation :/
			auto mappedPosition = mapping->toSurface( example_->position );
			if(!mappedPosition.hit)
				continue;

			if(!enableVolumeSurfaceInteraction)
			{
				// if one element is on a surface and the other in a volume, don't predict
				if((element->faceIndex < 0 && element_ >= 0) ||
					(element->faceIndex >= 0 && element_ < 0))
					continue;
			}

			predictions_in_out[j].emplace_back( 
				i, // .elementIndex
				mappedPosition.position, // .position
				f.weight // .weight
			);
		}
	}
}



void Algorithm::_removeNonOptimizingElements( std::set<FElement *>& localElements )
{
	std::vector<FElement*> toRemove;

	for(FElement * f : localElements)
	{
		if(f->optimizationParameters.radius == 0.0f || f->optimizationParameters.kNearest == 0)
			toRemove.push_back( f );
	}

	for(FElement * f : toRemove)
	{
		localElements.erase( f );
	}
}

std::set<FElement*> Algorithm::_filterSurfaceVolumeInteractions( std::set<FElement*>& elements )
{
	if(enableVolumeSurfaceInteraction)
		return elements;

	std::set<FElement*> filtered;

	if(generationMode == EGenerationMode::SurfacePainting)
	{
		for(FElement * e : elements)
		{
			if(e->faceIndex <= 0)
				continue;

			filtered.insert( e );
		}
	}
	else // volume elements
	{
		for(FElement * e : elements)
		{
			if(e->faceIndex > 0 )
				continue;

			filtered.insert( e );
		}
	}

	return filtered;
}

Algorithm::OptimizationProblem Algorithm::_localOptimization( OptimizationProblem problem_in, AlgorithmResult& result )
 {
	using namespace Eigen;
	using namespace std;

	typedef SparseMatrix<float> Matrix;
	typedef Triplet<float> Triplet;

	std::set<FElement*>& activeElements = problem_in.activeElements;
	std::set<FElement*>& elements = problem_in.elements;
	std::set<FElement*>& frozenElements = problem_in.frozenElements;

	// add the elements to the problem
	std::set<FElement*> localElements = activeElements;


	// add the frozen guys to the problem
	for(FElement * f : frozenElements)
		localElements.insert( f );

	// add remove any elements that don't have an optimization radius
	_removeNonOptimizingElements( localElements );

	// filter local elements for volume-surface interactions
	localElements = _filterSurfaceVolumeInteractions( localElements );
    
    // setup the problem map
    // we associate with each element and index in the A matrix (the problem matrix)
    std::map<FElement*, unsigned int> problem;
    std::vector<FElement*> indexToElement(localElements.size());
    
    unsigned int n = 0;
    for(auto e : localElements )
    {
        problem[e] = n;
        indexToElement[n] = e;
        ++n;
    } 

    if( n == 0 )
        return problem_in;
    
    std::vector<std::vector< PredictedPosition > > predictions(n);
    for( unsigned int i = 0; i < n; ++i )
        predictions[i] = std::vector< PredictedPosition >();
    
	for(auto keyValue : problem)
	{
		// this first prediction is a dummy for the covariance calculation
		// for some reason we do this twice, is this a mistake?
		FElement * element = keyValue.first;
		unsigned int i = keyValue.second;

		predictions[i].emplace_back( i, element->position );
		predictions[i].emplace_back( i, element->position );
	}

	// build the cache of nearest elements
	// collect the predicted positions from each frozen neighbourhood
	std::vector<PredictionNeighbourhood> predictionNeighbourhoods = _predictionNeighbourhoods( localElements );

	_buildPredictions( predictionNeighbourhoods, localElements /* limit to */, problem, predictions );


    // allocate the linear system
    // allocate Ax = b
	VectorXf b[3];
	for(int c = 0; c < 3; ++c)
	{
		b[c] = VectorXf( n );
		b[c].setZero();
	}

	std::vector<Triplet> a_t[3];
   
    // add the initial positions
	for(auto keyValue : problem)
	{
		const FElement * e = keyValue.first;
		const unsigned int i = keyValue.second;

		const float weight = 0.5;

		for(int c = 0; c < 3; ++c)
		{
			a_t[c].emplace_back( i, i, weight );
			b[c]( i ) += weight * e->position( c );
		}
	}

    // build a variance for each updated element position
    // Compute the eigen vectors and values for the prediction clusters
    for( unsigned int i = 0; i < n; ++i )
    {
        auto& positions = predictions[i];
        
        int m = positions.size();
        
        Matrix3f cinv;
        Vector3f mean;
        
        if( m <= 1 )
        {
            cinv = Matrix3f::Identity();
            mean = Vector3f::Zero();
        }
        else
        {
            MatrixXf x(m, 3);
            
            for( int j = 0; j < m; ++j )
                x.row(j) = positions[j].position;
            
            // it's a weird eigen type
            auto mean_ = x.colwise().mean();
            MatrixXf a = x.rowwise() - mean_;
            Matrix3f c = a.transpose() * a * (1.0f / float(m - 1));
            
            cinv = pinv<Matrix3f>(c);
            mean = mean_;
        }
        
        // ignore the first item, it's a dummy for the actual position used in the covariance calculation
        float normalizeTerm = 1.0f / (positions.size() - 1);
        for( int index = 1; index < positions.size(); ++index )
        {
            auto& prediction = positions[index];
            
            int j = prediction.elementIndex;
            FElement * element = indexToElement[j];
            
            const Vector3f predictedPosition = prediction.position;
            
            // find how the prediction position varies wrt to the covariance of
            // the other predictions
            Vector3f p = predictedPosition - mean;
            
            float w = std::exp( -.5f * p.transpose() * cinv * p ) / normalizeTerm;

			w *= prediction.weight;
            
            if( w != w || w == std::numeric_limits<float>::infinity() )
                w = 0.0f;
            
            const Vector3f predictedDirection = predictedPosition - element->position;
            
            for( int c = 0; c < 3; ++c )
            {
                a_t[c].emplace_back(i, i, w);
                a_t[c].emplace_back(j, j, w);
                a_t[c].emplace_back(i, j, -w);
                a_t[c].emplace_back(j, i, -w);
                
                b[c](j) -= w * predictedDirection(c);
                b[c](i) += w * predictedDirection(c);
            }
        }
        
        
    }
    
    // finally, we can build A
    Matrix a[3] = {Matrix(n,n), Matrix(n,n), Matrix(n,n)};
    for( int c = 0; c < 3; ++c )
    {
        a[c].setFromTriplets(a_t[c].begin(), a_t[c].end());
        a[c].finalize();
        a[c].makeCompressed();
    }
    
    VectorXf x[3] = {VectorXf(n), VectorXf(n), VectorXf(n)};
    
    for( int c = 0; c < 3; ++c )
    {
        Eigen::SimplicialCholesky<Matrix> cholesky(a[c]);
        x[c] = cholesky.solve(b[c]);
    }
    
    unordered_set<FElement*> modified;
    
	{
		for(auto& prediction : predictionNeighbourhoods)
		{
			if(prediction.element && prediction.similarNeighbourhood.element)
			{
				auto& sourceExamples = _sourceExampleMap.sourceExamples( prediction.element );

				auto it = sourceExamples.begin();

				// can we find a source example already for the selection?
				for(it = sourceExamples.begin(); it != sourceExamples.end(); ++it)
				{
					// found?
					if(auto ptr = it->selection.lock())
					{
						if(ptr == prediction.exampleSelection)
						{
							SourceExampleMap::SourceExample sourceExample( prediction.similarNeighbourhood.element, prediction.weight, prediction.exampleSelection );
							
							*it = sourceExample;
							break;
						}
					}
				}

				// not found, push_back a new SourceExample
				if( it == sourceExamples.end() )
					sourceExamples.emplace_back( prediction.similarNeighbourhood.element, prediction.weight, prediction.exampleSelection );
			}
		}
	}


    // update the positions
    for( auto keyValue : problem )
    {
        unsigned int i = keyValue.second;
        
        Vector3f p;
        for( int c = 0; c < 3; ++c )
            p(c) = x[c](i);
        
        FElement * e = keyValue.first;

		// update the position, p
        if( generationMode == EGenerationMode::SurfaceProjection )
        {
			auto& sourceExamples = _sourceExampleMap.sourceExamples( e );

			if(sourceExamples.size())
			{
				FElement * ex = sourceExamples[0].element;

				auto nearest = _meshInterface->nearestPointOnMesh( unreal( p ) );
				auto rotationAndNormal = _meshInterface->rotationAndNormalAtIndex( nearest.faceIndex );

				p = eigen( nearest.point ) + eigen( rotationAndNormal.second ) * ex->position.z();

				FQuat quat = rotationAndNormal.first;
				FQuat rotation = quat * unreal( ex->rotation );

				e->rotation = eigen( rotation );
			}
        }
		else if((generationMode == EGenerationMode::SurfaceWalking || generationMode == EGenerationMode::SurfacePainting))
		{
			auto& sourceExamples = _sourceExampleMap.sourceExamples( e );

			if(sourceExamples.size())
			{
				FElement * ex = sourceExamples[0].element;

				auto nearest = _meshInterface->nearestPointOnMesh( unreal( p ) );
				auto rotationAndNormal = _meshInterface->rotationAndNormalAtIndex( nearest.faceIndex );

				p = eigen( nearest.point );// +eigen( rotationAndNormal.second ) * ex->position.z();

				FQuat quat = rotationAndNormal.first;
				FQuat rotation = quat * unreal( ex->rotation );

				e->rotation = eigen( rotation );
				e->faceIndex = nearest.faceIndex;
			}
		}

		if(forceRotation)
			e->rotation = eigen( forcedRotation );
        
        e->position = p;
        


        modified.insert(e);
    }
    
    
    // votes and update types
	if(useTypeVoting)
	{
		// perform voting
		unordered_map<FElement*, unordered_map<int, int>> typeVotes;

		for(PredictionNeighbourhood& f : predictionNeighbourhoods)
		{
			FElement * element = f.element;
			
			auto& pairings = f.similarNeighbourhood.fullPairings;

			for(auto pair : pairings)
			{
				FElement * other = pair.first;

				if(!enableVolumeSurfaceInteraction)
				{
					// if one element is on a surface and the other in a volume, don't predict
					if((element->faceIndex < 0 && other >= 0) ||
						(element->faceIndex >= 0 && other < 0))
						continue;
				}


				auto& votes = typeVotes[pair.first];
				votes[pair.second->type]++;
			}
		}

		// analyze votes
		for(auto& elementVotes : typeVotes)
		{
			FElement * element = elementVotes.first;
			auto& votes = elementVotes.second;

			int maxType = 0;
			int maxCount = 0;
			int originalCount = 0;

			// determine max vote
			for(auto pair : votes)
			{
				if(pair.second > maxCount)
				{
					maxType = pair.first;
					maxCount = pair.second;
				}

				if(pair.first == element->type)
					originalCount++;
			}

			// reassign
			if(element->type != maxType && maxCount > originalCount)
			{
				element->type = maxType;
				
				modified.insert( element );

				// we changed the type, so we'll have to find a new source exemplar
				auto& sourceExamples = _sourceExampleMap.sourceExamples( element );

				for(SourceExampleMap::SourceExample& sourceExample : sourceExamples)
				{
					auto& generativeByType = sourceExample.generativeElementsByType( maxType, *_defaultSelection );

					auto nearest = _nearestSimilarElements( element, _output, generativeByType, _exemplar, element->generationParameters );

					if(!nearest.size())
						continue;


					FElement * sourceElement = nearest[0].element;

					// assign graph objects
					element->graphObjects = sourceElement->graphObjects;

					sourceExample.element = sourceElement;
				}
			}
		}
	}

	// rebalance the output kd-tree
	{
		std::clock_t start = std::clock();

		_output.rebalance();

		double deltaTime = (std::clock() - start) / (double)CLOCKS_PER_SEC;
		UE_LOG( LogTemp, Warning, TEXT( "Rebalanced in %f" ), (float)(deltaTime) );
	}

	// remove overlaps
	if(removeOverlaps)
	{
		std::unordered_set<FElement*> toUpdateFreespace;
		std::unordered_set<FElement*> toRemove;

		for(auto keyValue : problem)
		{
			FElement * element = keyValue.first;

			// element has been removed, don't remove his neighbours or we will have also removed the element that removed him (creating a void)
			if(toRemove.find( element ) != toRemove.end())
				continue;

			auto neighbours = _output.nearestInRadius( element->position, element->generationParameters.radius );

			auto overlaps = _output.nearestInRadius( element->position, element->radius * relaxation );

			// remove the other guys
			bool didRemove = false;

			for(FElement * e : neighbours)
			{
				if(e == element)
					continue;

				float allowedRadius = (element->radius + e->radius) * relaxation;

				float distance = (e->position - element->position).norm();
				if(distance < allowedRadius)
				{
					toRemove.insert( e );
					didRemove = true;
				}
			}

			if( didRemove )
				toUpdateFreespace.insert( element );
		}

		std::vector<FElement*> toRemoveVec( toRemove.begin(), toRemove.end() );
		removeElements( toRemoveVec );

		for(FElement * r : toRemoveVec)
		{
			modified.erase( r );
			activeElements.erase( r );
			elements.erase( r );
			frozenElements.erase( r );
		}

		// wake up the guys that weren't removed (but caused a removal)
		{
			std::vector<FElement*> asVector( toUpdateFreespace.begin(), toUpdateFreespace.end() );

			_expandFreeSpace( asVector );
		}

		// check if result contains any of the removed elements
		result.modified = _filter( result.modified, toRemove );
		result.generated = _filter( result.generated, toRemove );
		result.frozen = _filter( result.frozen, toRemove );
		result.removed = toRemoveVec;

		_output.rebalance();
	}
    
    std::copy(modified.begin(), modified.end(), std::back_inserter(result.modified));

	return problem_in;
}





std::vector<FElement*> Algorithm::_sphereOfInfluence(FElement * e, float r)
{
	if(r < 0)
		r = generationInnerRadius;

    return _output.neighbours(e, r, -1, false);
}

void Algorithm::_initBrushIndex()
{
	_brushPoints.pts.clear();
    
    _brushIndex.reset( new PositionFacePairCloudAdaptor( 3, _brushPoints, nanoflann::KDTreeSingleIndexAdaptorParams( 10 /* max leaf */ ) ) );
}



void Algorithm::addBrushPoint( PositionRadiusFace& point )
{
	_removeVoidPoint( point );

	size_t foundIndex;
	float dSqrd; 

	const float separationDistSqrd = std::pow( point.radius * 0.5f, 2.0f );

	if(_brushPoints.kdtree_get_point_count())
	{
		_brushIndex->knnSearch( &point.position.x(), 1, &foundIndex, &dSqrd );

		// don't add a brush point if it's very near another one
		if(dSqrd < separationDistSqrd)
			return;
	}


	_brushPoints.pts.push_back( point );
	size_t index = _brushPoints.kdtree_get_point_count() - 1;
	_brushIndex->addPoints( index, index );

	// wake up elements near the point
	std::vector<FElement*> nearest = _output.nearestInRadius( point.position, point.radius * 1.5f );

	_expandFreeSpace( nearest );
	
	// add them to the horizon
	for(auto e : nearest)
		_horizon.insert( e );
}


void Algorithm::setCurrentBrushPoint( PositionRadiusFace& point )
{
	_currentBrushPoint = point;
	_hasCurrentBrushPoint = true;
}

void Algorithm::clearCurrentBrushPoint()
{
	_hasCurrentBrushPoint = false;
}


void Algorithm::clearBrushPoints()
{
	_initBrushIndex();
}

std::vector<FElement*> Algorithm::eraseAt( PositionRadiusFace brushPoint )
{
	_removeBrushPoint( brushPoint );

	std::vector<FElement*> inRadius = _output.nearestInRadius( brushPoint.position, brushPoint.radius );

	if( inRadius.size() )
		this->removeElements( inRadius );

	return inRadius;
}

Algorithm::ExampleSelectionPtr Algorithm::addExampleSelection( std::vector<FElement*> selection, float weight /*= 1.0f */ )
{
	_exampleSelections.emplace_back( new ExampleSelection() );

	ExampleSelection * exampleSelection = _exampleSelections.back().get();

	exampleSelection->selection = std::unordered_set<FElement*>(selection.begin(),selection.end());
	exampleSelection->weight = weight;

	exampleSelection->init( _exemplar, *this );

	return (ExampleSelectionPtr)exampleSelection;
}

void Algorithm::removeExampleSelection( ExampleSelectionPtr exampleSelection )
{
	for(auto itr = _exampleSelections.begin(); itr < _exampleSelections.end(); itr++)
	{
		if((*itr).get() == (ExampleSelection*)exampleSelection)
		{
			_exampleSelections.erase( itr );
			return;
		}
	}
}

void Algorithm::updateExampleSelection( ExampleSelectionPtr ptr, std::vector<FElement*> selection, float weight /*= 1.0f */ )
{
	// double check that the ptr is valid
	for(auto itr = _exampleSelections.begin(); itr < _exampleSelections.end(); itr++)
	{
		if((*itr).get() == (ExampleSelection*)ptr)
			break;
	}

	assert( itr != _exampleSelections.end() );

	ExampleSelection * exampleSelection = (ExampleSelection*)ptr;

	exampleSelection->selection = std::unordered_set<FElement*>( selection.begin(), selection.end() );
	exampleSelection->weight = weight;

	exampleSelection->init( _exemplar, *this );
}

void Algorithm::_initVoidIndex()
{
	_voidIndex.reset( new PositionFacePairCloudAdaptor( 3, _voidPoints, nanoflann::KDTreeSingleIndexAdaptorParams( 10 /* max leaf */ ) ) );
}

void Algorithm::addVoidPoint( PositionRadiusFace& point )
{
	size_t foundIndex;
	float dSqrd;

	if(_voidPoints.kdtree_get_point_count())
	{
		_voidIndex->knnSearch( &point.position.x(), 1, &foundIndex, &dSqrd );

		// don't add a brush point if it's very near another one
		if(dSqrd < voidSize * voidSize * (1.0f / 8.0f) )
			return;
	}

	_voidPoints.pts.push_back( point );
	size_t newIndex = _voidPoints.kdtree_get_point_count() - 1;
	_voidIndex->addPoints( newIndex, newIndex );
}



void Algorithm::_removeBrushPoint( PositionRadiusFace& point )
{
	if(_brushPoints.kdtree_get_point_count() == 0)
		return;

	std::vector<std::pair<size_t, float>> points;
	nanoflann::SearchParams searchParams;

	_brushIndex->radiusSearch( &point.position.x(), point.radius, points, searchParams );

	if(points.size() == 0)
		return;

	std::unordered_set<size_t> toRemove;

	for(auto& pair : points)
		toRemove.insert( pair.first );

	// hack, we'll just rebuild the index
	auto pts = _brushPoints.pts;

	_brushPoints.pts.clear();
	_brushIndex.reset( new PositionFacePairCloudAdaptor( 3, _brushPoints, nanoflann::KDTreeSingleIndexAdaptorParams( 10 /* max leaf */ ) ) );

	for(size_t i = 0; i < pts.size(); ++i )
	{
		if(toRemove.find( i ) == toRemove.end())
			_brushPoints.pts.push_back( pts[i] );
	}

	if(_brushPoints.pts.size() > 0)
		_brushIndex->addPoints( 0, _brushPoints.pts.size() - 1 );
}

void Algorithm::_removeVoidPoint( PositionRadiusFace& point )
{
	if(_voidPoints.kdtree_get_point_count() == 0)
		return;

	size_t foundIndex;
	float dSqrd;

	_voidIndex->knnSearch( &point.position.x(), 1, &foundIndex, &dSqrd );

	// does the point overlap?
	if(dSqrd >= voidSize * voidSize )
		return;

	// it does, kill it, by rebuilding the index
	_voidPoints.pts.erase( _voidPoints.pts.begin() + foundIndex );
	_voidIndex->removePoint( foundIndex );
}






bool Algorithm::_canRemoveFromHorizon( FElement * element )
{
	if(!_hasCurrentBrushPoint)
		return true;

	FVector p = unreal( element->position );

	float brushSqrd = _currentBrushPoint.radius * _currentBrushPoint.radius;

	float distSqrd = (element->position - _currentBrushPoint.position).squaredNorm();

	return distSqrd > brushSqrd;
}

auto Algorithm::_cinpact( const float u, const float k, const float c ) -> float
{
	if( u >= c || u <= -c )
		return 0.0f;

	auto result = std::exp(
		(-k * (u * u)) /
		((c * c) - (u * u)) 
	);

	return result;
}

auto Algorithm::_cinpactSum( const Eigen::Vector3f point, const std::vector<FElement*>& elements, const int16_t elementType ) -> float
{
	float theSum = 0.0f;

	const float k = 3.0f;

	for(FElement * element : elements)
	{
		if(elementType != element->type)
			continue;

		const float c = cellExtents;// element->radius * 2.0f;

		const float u = (element->position - point).norm();

		theSum += _cinpact( u, k, c );
	}

	return theSum;
}

auto Algorithm::_discreteCinpactSimiliarity( const FElement& a, Domain& aDomain, const FElement& b, Domain& bDomain ) -> float
{
	using namespace Eigen;

	int numIndices_half = std::floor( generationParameters.radius / cellExtents );

	const Vector3f start = -Vector3f( cellExtents * numIndices_half, cellExtents * numIndices_half, cellExtents * numIndices_half );
	const Vector3f end = -start;

	Vector3f offset = start;

	float cost = 0.0f;

	// pull the types out of the generative-exemplar-by-type map
	std::vector<int16_t> types;
	for(auto& pair : _defaultSelection->_generativeElementsByType)
	{
		types.push_back( pair.first );
	}

	// calculate the cost as the cinpact difference between elements of the same type
	for(offset.x() = start.x(); offset.x() <= end.x(); offset.x() += cellExtents)
	{
		for(offset.y() = start.y(); offset.y() <= end.y(); offset.y() += cellExtents)
		{
			for(offset.z() = start.z(); offset.z() <= end.z(); offset.z() += cellExtents)
			{
				Vector3f p_a = a.position + offset;
				Vector3f p_b = b.position + offset;

				auto aElements = aDomain.nearestInRadius( p_a, cellExtents );
				auto bElements = bDomain.nearestInRadius( p_b, cellExtents );

				for(auto t : types)
				{
					cost += std::abs( _cinpactSum( p_a, aElements, t ) - _cinpactSum( p_b, bElements, t ) );
				}
			}
		}
	}

	return cost;
}

auto Algorithm::_nearestElements_discreteCinpactSimilarity( const FElement& element, Domain& domain, std::vector<FElement*> toConsiderInExemplar, Domain& exemplar ) 
-> std::vector<SimilarElement_NoPairings>
{
	std::vector<SimilarElement_NoPairings> result;

	for(FElement * exemplarElement : toConsiderInExemplar)
	{
		float cost = _discreteCinpactSimiliarity( element, domain, *exemplarElement, exemplar );

		SimilarElement_NoPairings similar;
		similar.element = exemplarElement;
		similar.cost = cost;

		result.push_back( similar );
	}

	sort( result.begin(), result.end(), []( const SimilarElement_NoPairings& a, const SimilarElement_NoPairings& b ) -> bool
	{
		return a.cost < b.cost;
	} );

	return result;
}

void Algorithm::ExampleSelection::clear()
{
	selection.clear();

	_kCoherentElements.clear();

	_generativeElements.clear();
	_generativeElementsByType.clear();
}

void Algorithm::ExampleSelection::init( Domain& exemplar, Algorithm& algorithm )
{
	_initGenerativeExamples( exemplar );
	_initKCoherentNeighbours( exemplar, algorithm );
}

void Algorithm::ExampleSelection::_initGenerativeExamples( Domain& exemplar )
{
	_generativeElements.clear();
	_generativeElementsByType.clear();

	for(FElement * element : selection)
	{
		if(!element->generative)
			continue;

		_generativeElements.push_back( element );

		auto& byTypeVector = _generativeElementsByType[element->type];
		byTypeVector.push_back( element );
	}
}

void Algorithm::ExampleSelection::_initKCoherentNeighbours( Domain& exemplar, Algorithm& algorithm )
{
	_kCoherentElements.clear();

	const float clusteringRadiusSqrd = algorithm.kCoherenceClusteringPenaltyRadius * algorithm.kCoherenceClusteringPenaltyRadius;

	std::vector< std::future< std::pair<FElement*, vector<FElement*> > > > futures;

	for(auto& ex_ptr : exemplar)
	{
		FElement * ex = &(*ex_ptr);

		futures.emplace_back( algorithm._threadPool->push( [this, ex, &exemplar, &algorithm, clusteringRadiusSqrd]( int threadID ) mutable {
			vector<SimilarElement> nearestExamples = _nearestSimilarElements( 
				ex, exemplar, 
				_generativeElements, exemplar,
				ex->generationParameters, 
				true, // forceSpaceFilling
				false // filterCandidatesByType
			);

			int k = min<float>( nearestExamples.size(), algorithm.kCoherence );

			vector<FElement*> kNeighbours( k );

			for(int i = 0; i < k; ++i)
				kNeighbours[i] = nearestExamples[i].element;

			//vector<FElement*> kNeighbours;

			//unordered_set<SimilarElement*> similarElements;
			//for(SimilarElement& similar : nearestExamples)
			//	similarElements.insert( &similar );

			//for(int i = 0; i < k; ++i)
			//{
			//	float minCost = numeric_limits<float>::max();
			//	SimilarElement * nearest = &nearestExamples[0];

			//	// find the nearest element
			//	for(SimilarElement * similar : similarElements)
			//	{
			//		float cost = similar->cost;

			//		// we don't want to choose k-coherent neighbours that are
			//		// all close to eachother, so spread them out
			//		for(FElement * kNeighbour : kNeighbours)
			//		{
			//			if((kNeighbour->position - similar->element->position).squaredNorm() < clusteringRadiusSqrd)
			//				cost += algorithm.kCoherenceClusteringPenalty;
			//		}

			//		if(cost < minCost)
			//		{
			//			minCost = cost;
			//			nearest = similar;
			//		}
			//	}

			//	similarElements.erase( nearest );
			//	kNeighbours.push_back( nearest->element );
			//}

			// the coherence neighbours should include the example itself!
			// kNeighbours.push_back( ex );

			return std::make_pair( ex, kNeighbours );
		} ) );
	}

	for(auto& f : futures)
	{
		auto pair = f.get();

		_kCoherentElements[pair.first] = pair.second;
	}
}
