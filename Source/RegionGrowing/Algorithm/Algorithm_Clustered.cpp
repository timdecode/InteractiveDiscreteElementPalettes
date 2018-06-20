//
//  Algorithm_Clustered.cpp
//  RegionGrowing
//
//  Created by Timothy Davison on 2015-07-20.
//  Copyright (c) 2015 EpicGames. All rights reserved.
//

#include "RegionGrowing.h"


#include "Algorithm.h"
#include "Algorithm/FreeSpace.h"
#include "Algorithm/dbscan.hpp"

#include "Eigen/Sparse"
#include "Eigen/Geometry"

#include "Algorithm_Clustered.h"

using namespace Eigen;
using namespace std;

AlgorithmResult Algorithm_Clustered::generate( std::vector<PositionFace>& startingPositions, float radius, AABB limits )
{
	AlgorithmResult result;

	if(!_didInit)
	{
		_initialize();

		for(auto start : startingPositions)
			_primeOutput( start, limits, result );

		_calculateFreeSpace( result.generated );

		return result;
	} 

	// Randomly pick our first problem
	// Iterate on it
	std::vector< Observation > observations;

	for(OutputFreeSpaceCluster& freespace : _freeSpaceHorizon)
	{
		FreeSpaceCluster& cluster = *(freespace.exampleCluster);

		if(cluster.offsetsAndElements.size() == 0)
			continue;

		int32 chosen = _randomStream.RandRange( 0, cluster.offsetsAndElements.size() - 1 );

		observations.emplace_back( chosen, &freespace );
	}

	for(Observation& observation : observations)
	{
		OutputFreeSpaceCluster * outputCluster = observation.cluster;

		FreeSpaceCluster * exampleCluster = outputCluster->exampleCluster;

		auto mapping = _mapping( outputCluster->position, outputCluster->faceIndex, Vector3f::Zero() );

		auto elementAndOffset = exampleCluster->offsetsAndElements[observation.exampleIndex];  

		auto mappingResult = mapping->toSurface( elementAndOffset.first );

		if( _canPlace( elementAndOffset.second, mappingResult, limits) )
			_instantiateObservation( observation, mappingResult );
	}

	// now, re-pick our instantiated observations and pick the best match
	// We do this in two steps:
	// 1. find the best new observations, without updating the output domain.
	// 2. instantiate the observations to the output
	// This is a double buffering approach and does not require sequential execution.
	// 3. update the horizon with new free-space points

	// **Step 1**
	// Find the best new observations, without updating the output domain.
	for(unsigned int round = 0; round < numRounds; round++)
	{
		for(Observation& observation : observations)
		{
			FElement * output = observation.output;

			OutputFreeSpaceCluster * outputCluster = observation.cluster;

			FreeSpaceCluster * exampleCluster = outputCluster->exampleCluster;

			// find the best example in the cluster to reassign the observation to
			int32 bestExample = -1;
			float minimumDistance = std::numeric_limits<float>::max();

			for(int i = 0; i < exampleCluster->offsetsAndElements.size(); ++i)
			{
				auto& pair = exampleCluster->offsetsAndElements[i];

				Vector3f elementOffset = pair.first;
				FElement * example = pair.second;

				FElement tempOutputElement;

				Observation tempObservation( i, outputCluster );
				tempObservation.output = &tempOutputElement;

				{
					auto mapping = _mapping( outputCluster->position, outputCluster->faceIndex, Vector3f::Zero() );

					auto mappingResult = mapping->toSurface( elementOffset );

					// instantiating a temporary observation will not affect the output or neighbouring elements
					if(!_canPlace( example, mappingResult, limits ))
						continue;

					_instantiateObservation( tempObservation, mappingResult );
				}

				{
					std::vector<FElement*> outputNeighbours = _output.nearestInRadius( tempObservation.output->position, generationParameters.radius, generationParameters.kNearest );

					for(auto it = outputNeighbours.begin(); it != outputNeighbours.end(); it++)
					{
						if(*it == output)
						{
							outputNeighbours.erase( it );
							break;
						}
					}

					std::vector<FElement*> exampleNeighbours = _exemplar.neighbours( example, generationParameters.radius, generationParameters.kNearest );

					auto mapping = _mapping( tempObservation.output->position, tempObservation.output->faceIndex, example->position );

					auto pairing = distance( tempOutputElement, outputNeighbours, *example, exampleNeighbours, mapping.get() );

					if(pairing.cost < minimumDistance)
					{
						minimumDistance = pairing.cost;
						bestExample = i;
					}
				}

			}

			// reassign the observation to the found best element
			observation.exampleIndex = bestExample;

			if(observation.isValid())
				setSourceExample( observation.output, observation.cluster->exampleCluster->offsetsAndElements[bestExample].second );
		}
	}

	// **Step 2** 
	// Instantiate the observations to the output.
	for(Observation& observation : observations)
	{
		if(!observation.isValid())
			continue;

		OutputFreeSpaceCluster * outputCluster = observation.cluster;

		FreeSpaceCluster * exampleCluster = outputCluster->exampleCluster;

		auto mapping = _mapping( outputCluster->position, outputCluster->faceIndex, Vector3f::Zero() );

		auto mappingResult = mapping->toSurface( observation.example()->position );

		if(_canPlace( observation.example(), mappingResult, limits ))
			_instantiateObservation( observation, mappingResult );
	}

	_output.rebalance();

	// **Step 3**
	// Update the horizon with new free-space points.
	std::vector<FElement*>& generated = result.generated;

	generated.reserve( observations.size() );

	for(Observation& observation : observations)
	{
		if( observation.isValid() && observation.output )
			generated.push_back( observation.output );
	}

	_calculateFreeSpace( generated );

	return result;
}

bool Algorithm_Clustered::_canPlace( FElement * example, rg::Mapping::Result& mappingResult, AABB& limits )
{
	if(!mappingResult.hit)
		return false;

	Eigen::Vector3f p = mappingResult.position;

	if(!_inBoundary( p, example->radius ))
		return false;

	if(!limits.contains( p ))
		return false;

	PositionFace facePair( p, mappingResult.face );

	if(!_occlusionTester->isVisible( facePair, example->radius ))
		return false;

	if(_overlaps( p, example->radius, example->generationParameters ))
		return false;

	return true;
}

void Algorithm_Clustered::_primeOutput( PositionFace& position, AABB& limits, AlgorithmResult& result )
{
	if(_exemplar.size() == 0)
		return;

	Vector3f samplingPoint = Vector3f::Zero();

	// where to sample the exemplar from
	std::set<int32_t> neighbourhoodEntityIndices;

	// sample from the centroid

	// find the centre of the exemplar
	Vector3f centre = Vector3f::Zero();

	for(auto& element : _exemplar)
		centre += element->position;

	centre /= float( _exemplar.size() );

	samplingPoint = centre;

	auto neighbourhood = _exemplar.nearestInRadius( samplingPoint, generationParameters.radius );

	for(auto neighbour : neighbourhood)
		neighbourhoodEntityIndices.insert( neighbour->entityIndex );

	auto mapping = position.face < 0 ? _mapping( position.position, samplingPoint ) : _mapping( position.position, position.face, samplingPoint );

	auto& outputEntities = _output.entities();

	for(int32_t entityIndex : neighbourhoodEntityIndices)
	{
		Entity& entity = _exemplar.entityAt( entityIndex );

		int32_t outputEntityIndex = outputEntities.size();

		outputEntities.emplace_back();
		Entity& outputEntity = outputEntities.back();
		outputEntity.entityID = entity.entityID;

		for(auto sampleIndex : entity.elementIndices)
		{
			FElement * element = _exemplar.elementAt( sampleIndex );

			auto mappingResult = mapping->toSurface( element->position );

			if(!_canPlace( element, mappingResult, limits ))
				continue;

			int32_t ouputSampleIndex = _output.size();
			FElement * added = _copyExemplarToOutput( element, mappingResult.position, _defaultSelection );
			setSourceExample( added, element );

			added->entityIndex = outputEntityIndex;
			added->faceIndex = mappingResult.face;

			if(!forceRotation && (generationMode == EGenerationMode::SurfaceProjection || generationMode == EGenerationMode::SurfaceWalking || generationMode == EGenerationMode::SurfacePainting))
			{
				auto rotationAndNormal = _meshInterface->rotationAndNormalAtIndex( mappingResult.face );

				FQuat quat = rotationAndNormal.first;
				FQuat rotation = quat * unreal( element->rotation );

				added->rotation = eigen( rotation );
			}
			else if(forceRotation)
			{
				added->rotation = eigen( forcedRotation );
			}

			outputEntity.elementIndices.push_back( ouputSampleIndex );

			result.generated.push_back( added );
		}
	}
}

void Algorithm_Clustered::_instantiateObservation( Observation& observation, rg::Mapping::Result& mappingResult )
{
	if(!observation.isValid())
		return;

	OutputFreeSpaceCluster& outputCluster = *(observation.cluster);
	FreeSpaceCluster& inputCluster = *(outputCluster.exampleCluster);

	FElement * outputElement = nullptr;

	Vector3f outputPosition = mappingResult.position;

	// do we just need to update the output element?
	if(observation.output)
	{
		outputElement = observation.output;

		// copy the example's properties (POD) to the output element
		*outputElement = *(observation.example());	

		outputElement->position = outputPosition;

		setSourceExample( outputElement, observation.example() );
	}
	// or do we need to create an output element?
	else
	{
		observation.output = _copyExemplarToOutput( observation.example(), outputPosition, _defaultSelection );

		outputElement = observation.output;
	}

	outputElement->faceIndex = mappingResult.face;

	auto rotationAndNormal = _meshInterface->rotationAndNormalAtIndex( mappingResult.face );

	FQuat quat = rotationAndNormal.first;
	FQuat rotation = quat * unreal( observation.example()->rotation );

	outputElement->rotation = eigen(rotation);
}

void Algorithm_Clustered::_rebuildFreespaceIndex()
{
	 
}

void Algorithm_Clustered::_calculateFreeSpace( std::vector<FElement*>& newElements )
{
	_freeSpaceHorizon.clear();
	 
	_freeSpace._freespaceCloud.pts.clear();

	if(_freeSpace._freespaceIndex)
		delete _freeSpace._freespaceIndex;

	_freeSpace._freespaceIndex = new EigenDynamicPointCloudIndexAdaptor( 3, _freeSpace._freespaceCloud, nanoflann::KDTreeSingleIndexAdaptorParams( 10 /* max leaf */ ) );
	//_freespaceIndex->buildIndex();

	for(FElement * outputElement : newElements)
	{
		FElement * sourceElement = sourceExample(outputElement);

		std::vector<FreeSpaceCluster>& exampleClusters = _exemplarClusters[sourceElement];

		auto mapping = _mapping( outputElement->position, outputElement->faceIndex, sourceElement->position );

		// see if there is room for each cluster in the output and if there is, add it to the horizon
		for(FreeSpaceCluster& exampleCluster : exampleClusters)
		{
			auto mappingResult = mapping->toSurface( exampleCluster.centroid );

			// ignore any free-space points that overlap with the generated element
			bool freeSpaceOverlap = false;

			if(_freeSpace._freespaceCloud.pts.size())
			{
				size_t indices;
				float distancesSqrd;

				//dynoflann::PredicateKNNResultSet<DistanceType, size_t, size_t, > resultSet( num_closest );
				//resultSet.init( out_indices, out_distances_sq );
				//this->findNeighbors( resultSet, query_point, dynoflann::SearchParams() );

				_freeSpace._freespaceIndex->knnSearch( &mappingResult.position( 0 ), 1, &indices, &distancesSqrd );

				OutputFreeSpaceCluster& nearestCluster = _freeSpaceHorizon[indices];

				float distance = FMath::Sqrt( distancesSqrd );

				if(distance - (exampleCluster.radius + nearestCluster.exampleCluster->radius) < 0.0f)
					freeSpaceOverlap = true;
			}

			// check for element overlap
			if(!freeSpaceOverlap && _output.size())
			{
				auto nearest = _output.nearest( mappingResult.position );

				FElement * element = nearest.element;

				float distance = FMath::Sqrt( nearest.distanceSquared );

				if(distance - (exampleCluster.radius + element->radius) < 0.0f)
					freeSpaceOverlap = true;
			}

			if(freeSpaceOverlap)
				continue;

			// no overlap, add the example cluster
			_freeSpaceHorizon.emplace_back();

			OutputFreeSpaceCluster& newOutputCluster = _freeSpaceHorizon.back();

			newOutputCluster.exampleCluster = &exampleCluster;
			newOutputCluster.position = mappingResult.position;
			newOutputCluster.faceIndex = mappingResult.face;

			_freeSpace._freespaceCloud.pts.emplace_back( newOutputCluster.position );
			size_t newIndex = _freeSpaceHorizon.size() - 1;
			_freeSpace._freespaceIndex->addPoints( newIndex, newIndex );
		}
	}
}

void Algorithm_Clustered::_initialize()
{
	Algorithm::_initialize();

	_randomStream.Initialize( 42 );

	_initGenerativeExamples();
	_initExemplarClusters();
}

void Algorithm_Clustered::_initGenerativeExamples()
{
	for(auto& e : _exemplar)
	{
		FElement * element = e.get();

		if(e->generative)
			_nearestGenerativeExample[element] = element;
		// we're going to have to find the most similar example element that is generative
		else
		{
			vector<SimilarElement> nearestExamples = _nearestSimilarElements( element, _exemplar, _defaultSelection->_generativeElements, _exemplar, generationParameters, true );

			if(nearestExamples.size())
				_nearestGenerativeExample[element] = nearestExamples[0].element;
		}
	}
}

void Algorithm_Clustered::_initExemplarClusters()
{
	_exemplarClusters.clear();
	_freeSpace._freespaceCloud.pts.clear();

	if(_freeSpace._freespaceIndex)
		delete _freeSpace._freespaceIndex;

	_freeSpace._freespaceIndex = new EigenDynamicPointCloudIndexAdaptor( 3, _freeSpace._freespaceCloud, nanoflann::KDTreeSingleIndexAdaptorParams( 10 /* max leaf */ ) );
	//_freespaceIndex->buildIndex();

	for(auto& element : _exemplar)
	{
		if(!element->generative)
			continue;

		std::vector<FElement*>& coherentExamples = _defaultSelection->_kCoherentElements[element.get()];

		std::vector<FElement*> elements;
		std::vector<Eigen::Vector3f> points;

		for(FElement * c : coherentExamples)
		{
			Vector3f position = c->position;

			auto neighbours = _exemplar.neighbours( c, generationParameters.radius );

			for(auto c_ : neighbours)
			{
				points.push_back( c_->position - position + element->position );
				elements.push_back( c_ );
			}
		}

		std::vector< std::vector<rg::PointAndIndex> > clusters_out;
		std::vector<rg::PointAndIndex> noise_out;

		rg::dbscan( points, freespaceRadius, 2, clusters_out, noise_out );

		auto& clusters = _exemplarClusters[element.get()];


		auto setupCluster = [&]( std::vector<rg::PointAndIndex>& dbCluster ) mutable
		{
			// find the midpoint and add use that as the free-space point
			Eigen::Vector3f centre = Eigen::Vector3f::Zero();
			Eigen::AlignedBox<float,3> bbox;

			for(auto& c : dbCluster)
			{
				centre += c.point;

				float radius = elements[c.index]->radius;
				Vector3f extents( radius, radius, radius );

				Eigen::AlignedBox<float, 3> point_bbox( c.point - extents, c.point + extents );

				bbox.extend( c.point );
			}

			centre /= dbCluster.size();

			// fill the cluster data structure
			clusters.emplace_back();
			FreeSpaceCluster& cluster = clusters.back();

			cluster.centroid = centre;

			cluster.radius = freespaceRadius;// bbox.sizes().maxCoeff();

			for(auto& c : dbCluster)
			{
				FElement * element_c = elements[c.index];
				Vector3f offset = element_c->position - centre;

				cluster.offsetsAndElements.emplace_back( offset, elements[c.index] );
			}
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
}

void Algorithm_Clustered::setSourceExample( FElement * output, FElement * source )
{
	FElement * generative = _nearestGenerativeExample[source];

	auto& sourceExamples = _sourceExampleMap.sourceExamples( output );
	sourceExamples.clear();
	sourceExamples.emplace_back( source, 1.0f, _defaultSelection );
}

FElement* Algorithm_Clustered::sourceExample( FElement * output )
{
	auto& sourceExamples = _sourceExampleMap.sourceExamples( output );

	if(sourceExamples.size())
		return sourceExamples[0].element;
	else
		return nullptr;
}