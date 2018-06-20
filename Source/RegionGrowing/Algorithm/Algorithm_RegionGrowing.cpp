//
//  Algorithm_RegionGrowing.cpp
//  RegionGrowing
//
//  Created by Timothy Davison on 2018-01-04.
//  Copyright (c) Timothy Davison. All rights reserved.
//

#include "RegionGrowing.h"

#include "Algorithm/Algorithm_RegionGrowing.h"

#include "dbscan.hpp"
#include "SpaceMapping.hpp"

#include "Eigen/Sparse"
#include "Eigen/Geometry"

AlgorithmResult Algorithm_RegionGrowing::_reassignSourceExamples()
{
	AlgorithmResult result;
	std::set<FElement*> elements;

	for(auto& brushPoint : _brushPoints.pts)
	{
		auto generationArea = _output.nearestInRadius( brushPoint.position, brushPoint.radius );

		for(FElement * e : generationArea)
		{
			if(!enableVolumeSurfaceInteraction && e->faceIndex <= 0 && generationMode == EGenerationMode::SurfacePainting)
				continue;

			elements.insert( e );
		}
	}

	// reassign source examples to elements from the example selection
	for(FElement * element : elements)
	{
		auto& sourceExamples = _sourceExampleMap.sourceExamples( element );

		bool modified = false;

		// add missing source examples
		for(auto& exampleSelectionPtr : _exampleSelections)
		{
			SourceExampleMap::SourceExample * found = nullptr;

			for(auto& sourceExample : sourceExamples)
			{
				if(sourceExample.selection.lock() == exampleSelectionPtr)
				{
					found = &sourceExample;
					break;
				}
			}

			// update the weight
			if(found)
			{
				if(found->weight != exampleSelectionPtr->weight)
				{
					found->weight = exampleSelectionPtr->weight;
					modified = true;
				}

				continue;
			}
			// add a new source example if we didn't find it
			else
			{
				auto nearest = _nearestSimilarElements(
					element,
					_output,
					exampleSelectionPtr->_generativeElements,
					_exemplar,
					element->generationParameters,
					false, // force space filling
					false // don't filter the candidate by type (include it)
				);

				if(!nearest.size())
					continue;

				FElement * nearestElement = nullptr;

				// filter the nearest set to only contain elements in the selection
				for(auto& near : nearest)
				{
					if(exampleSelectionPtr->selection.find( near.element ) != exampleSelectionPtr->selection.end())
					{
						nearestElement = near.element;
						break;
					}
				}

				if(!nearestElement)
					continue;

				sourceExamples.emplace_back(
					nearestElement,
					exampleSelectionPtr->weight,
					exampleSelectionPtr
				);

				// hack reassign graph objects
				element->graphObjects = nearestElement->graphObjects;

				modified = true;
			}
		}

		if(modified)
			result.modified.push_back( element );

		//// remove a source example, if it's not in our selections
		//for(auto it = sourceExamples.begin(); it != sourceExamples.end(); )
		//{
		//	FElement * sourceElement = it->element;

		//	bool found = false;
		//	for(auto& selection : _exampleSelections)
		//	{
		//		if(selection->selection.find( sourceElement ) != selection->selection.end())
		//		{
		//			found = true;
		//			break;
		//		}
		//	}

		//	if(found)
		//		++it;
		//	else // it's not in our selections, nuke it
		//		it = sourceExamples.erase( it );
		//}

		//// add new source examples for the element
		//for(auto& selection : _exampleSelections)
		//{
		//	bool found = false;
		//	for(auto& sourceExample : sourceExamples)
		//	{
		//		if(selection->selection.find( sourceExample.element ) != selection->selection.end())
		//		{
		//			found = true;
		//			break;
		//		}
		//	}

		//	if(found)
		//		continue;

		//	// the sourceExamples does not contain any elements in the current selection
		//	// so, find one

		//	auto nearest = _nearestSimilarElements(
		//		element,
		//		_output,
		//		selection->_generativeElementsByType[element->type],
		//		_exemplar,
		//		element->generationParameters
		//	);

		//	if(!nearest.size())
		//		continue;

		//	FElement * nearestElement = nearest[0].element;

		//	sourceExamples.emplace_back(
		//		nearestElement,
		//		selection->weight, 
		//		selection );

		//	// hack reassign graph objects
		//	element->graphObjects = nearestElement->graphObjects;

		//	result.modified.push_back( element );
		//}
	}



	return result;
}

AlgorithmResult Algorithm_RegionGrowing::_generate( std::vector<PositionFace>& startingPositions, float radius /*= -1.0f*/, AABB limits /*= AABB() */ )
{
	AlgorithmResult result;

	_rebuildFreespace();

	result = _reassignSourceExamples();

	// we only care about those elements around the origin
	// The local horizon are all those elements within the synthesize radius of a starting point or within
	// the neighbourhood of a brush point.
	std::set<FElement*> localHorizon;
	if(radius > 0.0f)
	{
		auto buildLocalHorizon = [&]( PositionRadiusFace origin )
		{
			PositionFace positionFace( origin.position, origin.face );

			auto local = _output.nearestInRadius( origin.position, radius, -1, false );
			auto generationArea = _output.nearestInRadius( origin.position, origin.radius );

			for(auto e : local)
			{
				if(_horizon.find( e ) == _horizon.end())
					continue;

				// don't add elements to the problem if we are generating on a surface and the element isn't on a surface
				if(!enableVolumeSurfaceInteraction && origin.face > 0 && generationMode == EGenerationMode::SurfacePainting)
					continue;

				localHorizon.insert( e );
			}

			// do we need to add any seed points?
			// yes, if there are no elements nearby or if there are no points in our generation radius
			if(!(local.size() == 0 || generationArea.size() == 0))
				return;

			if(!_occlusionTester->isVisible( positionFace, 1.0f ))
				return;


			_initializeOutput( positionFace, limits, result );

			_expandFreeSpace( result.generated );
		};

		for(auto origin : startingPositions)
		{
			PositionRadiusFace positionFace( origin.position, origin.face, generationParameters.radius );
			buildLocalHorizon( positionFace );
		}

		for(auto brushPoint : _brushPoints.pts)
		{
			buildLocalHorizon( brushPoint );
		}

		localHorizon = _removeFrozen_usingFreespacePoints( localHorizon, _output, result.frozen );

		// update the global horizon
		for(auto e : result.frozen)
		{
			_horizon.erase( e );
		}
	}
	else
	{
		_horizon = _removeFrozen_usingFreespacePoints( _horizon, _output, result.frozen );

		localHorizon = _horizon;
	}

	EigenPointCloud cloud;
	EigenDynamicPointCloudIndexAdaptor startingPositions_kdTree( 3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams( 10 ) );

	{
		std::vector<Eigen::Vector3f> asPoints;
		for(auto& pair : startingPositions)
			asPoints.push_back( pair.position );

		cloud.pts = asPoints;

		if(cloud.pts.size() > 0)
			startingPositions_kdTree.addPoints( 0, cloud.pts.size() - 1 );
	}

	// seeds!!
	auto seeds = _seeds( localHorizon );

	if(seeds.size() > 0)
	{
		// prune the seeds
		if(radius > 0)
		{
			seeds = _constrainSeedsToBounds( radius, seeds, startingPositions_kdTree );
		}

		// precompute our nearest elements
		// build the cache of nearest elements
		std::vector<std::vector<SimilarElement>> similarElementsCache;

		std::vector<std::future<std::vector<SimilarElement>>> futures;

		for(int i = 0; i < seeds.size(); ++i)
		{
			// emplace back, not push_back, so that we call the future move constructor
			futures.emplace_back( _threadPool->push( [&, i]( int threadID ) mutable {
				FElement * seed = seeds[i];

				std::vector<Algorithm::SimilarElement> nearest;


				auto filteredGenerativeExemplars = _fastFilterGenerativeExemplars( seed, _freeSpace );

				nearest = _generation_nearestSimilarElements( seed, _output, filteredGenerativeExemplars, _exemplar, seed->generationParameters );



				//// nearest will have the exemplar samples ordered first
				//// followed by the unbiased nearest exemplars
				//for(auto& s : unbiasedNearest)
				//{
				//	nearest.push_back( s );
				//}

				//// resort
				//sort( nearestSimilarElements.begin(), nearestSimilarElements.end(), []( const SimilarElement& a, const SimilarElement& b ) -> bool
				//{
				//	return a.cost < b.cost;
				//} );

				return nearest;
			} ) );
		}

		for(auto& f : futures)
			similarElementsCache.push_back( f.get() );

		std::shared_ptr<ExampleSelection> exampleSelection = _highestWeightExampleSelection();
		auto& selectionElements = exampleSelection->selection;

		for(int i = 0; i < seeds.size(); ++i)
		{
			FElement * seed = seeds[i];


			// Warning, this is very hacky, because it doesn't consider newly synthesized neighbours
			vector<SimilarElement>& similarElements = similarElementsCache[i];

			vector<FElement*> generatedElements;

			float lastCost = 0.0f;
			if(similarElements.size())
				lastCost = similarElements[0].cost;

			int assigned = 0;


			for(SimilarElement& similar : similarElements)
			{
				// second pass at the seed
				if(ignoreBadSuggestions)
				{
					// check if the suggestion is terrible
					if(similar.cost > similar.fullPairings.size() * ignoreBadSuggestionsDistanceFactor * typeCost)
					{
						continue;
					}
				}

				lastCost = similar.cost;

				FElement * example = similar.element;

				for(auto& pair : similar.rightPartialPairings)
				{
					FElement* ex_ = pair.second;

					Vector3f prediction = ex_->position - example->position;
					float predictionNorm = prediction.norm();
					if(predictionNorm > seed->generationParameters.radius || predictionNorm > seed->generationInnerRadius)
						continue;

					// now add all the siblings of the entity
					std::vector<FElement*> toAdd;

					Entity& ex_parent = _exemplar.entityAt( ex_->entityIndex );

					for(int ex_i : ex_parent.elementIndices)
					{
						FElement * ex_sibling = _exemplar.elementAt( ex_i );

						toAdd.push_back( ex_sibling );
					}

					std::vector<Eigen::Vector3f> newPositions;
					std::vector<int> newFaces;

					auto mapping = _mapping( *seed, *example );

					for(FElement * ex_sibling : toAdd)
					{
						auto toSurface = mapping->toSurface( ex_sibling->position );
						if(!toSurface.hit)
							continue;

						Vector3f& newPosition = toSurface.position;

						if(selectionElements.find( ex_sibling ) == selectionElements.end())
							continue;

						if(_overlaps( newPosition, ex_sibling->radius, ex_sibling->generationParameters ))
							continue;

						if(!_inBoundary( newPosition, ex_sibling->radius ))
							continue;

						if(!limits.contains( newPosition, ex_sibling->radius ))
							continue;

						PositionFace facePair( newPosition, toSurface.face );
						if(!_occlusionTester->isVisible( facePair, ex_sibling->radius ))
							continue;

						newPositions.push_back( newPosition );
						newFaces.push_back( toSurface.face );

						// remap for the next element
						mapping = _mapping( newPosition, toSurface.face, ex_sibling->position );
					}

					if(newPositions.size() != toAdd.size())
						continue;

					assigned++;

					auto entityIndex = _output.entitySize();
					Entity& entity = _output.emplaceEntityBack();
					entity.entityID = ex_parent.entityID;
					auto& elementIndices = entity.elementIndices;

					for(int j = 0; j < newPositions.size(); ++j)
					{
						auto& newPosition = newPositions[j];
						FElement * ex_sibling = toAdd[j];

						int elementIndex = _output.size();

						FElement* newElement = _copyExemplarToOutput( ex_sibling, newPosition, exampleSelection );

						newElement->faceIndex = newFaces[j];
						newElement->entityIndex = entityIndex;
						elementIndices.push_back( elementIndex );

						if(generationMode == EGenerationMode::SurfaceProjection)
						{
							auto rotationAndNormal = _meshInterface->rotationAndNormalAtSurfacePoint( unreal( newPosition ) );

							FQuat quat = rotationAndNormal.first;
							FQuat rotation = quat * unreal( ex_sibling->rotation );

							newElement->rotation = eigen( rotation );
						}
						else if(!forceRotation && (generationMode == EGenerationMode::SurfaceWalking || generationMode == EGenerationMode::SurfacePainting))
						{
							auto rotationAndNormal = _meshInterface->rotationAndNormalAtIndex( newElement->faceIndex );

							FQuat quat = rotationAndNormal.first;
							FQuat rotation = quat * unreal( ex_sibling->rotation );

							newElement->rotation = eigen( rotation );
						}
						else if(forceRotation)
							newElement->rotation = eigen( forcedRotation );

						result.generated.push_back( newElement );
						localHorizon.insert( newElement );
					}
				}

				if(assigned > 0)
					break;
			}

			// place the free-space points in low priority
			if(assigned == 0 && _canRemoveFromHorizon( seed ))
			{
				_horizon.erase( seed );
				localHorizon.erase( seed );

				result.frozen.push_back( seed );

				// remove all the free-space points around the removed element
				auto freespaceIndices = _freeSpace.outputIndicesInRadius( seed->position, seed->generationParameters.radius );

				for(auto& pair : freespaceIndices)
					_freeSpace.removeOutputPoint( pair.first );
			}
		}
	}




	if(!disableOptimization)
	{
		AlgorithmResult optimizationResult;

		OptimizationProblem optimizationProblem;

		if(useGlobalOptimization)
		{
			for(auto& e : _output)
			{
				optimizationProblem.activeElements.insert( (e.get()) );
				optimizationProblem.elements.insert( (e.get()) );
				optimizationProblem.frozenElements.insert( e.get() );
			}
		}
		else
		{

			optimizationProblem.activeElements = localHorizon;

			for(auto& e : _output)
				optimizationProblem.elements.insert( e.get() );

			optimizationProblem.frozenElements = _nearbyFrozenElements( optimizationProblem.activeElements );
		}

		for(int i = 0; i < optimizationRounds; ++i)
		{
			AlgorithmResult optimizationResult;

			optimizationProblem = _localOptimization( optimizationProblem, optimizationResult );

			result.append( optimizationResult );
		}
	}

	_expandFreeSpace( result.generated );

	return result;
}

std::vector<FElement *> Algorithm_RegionGrowing::_constrainSeedsToBounds( float radius, std::vector<FElement *>& seeds, EigenDynamicPointCloudIndexAdaptor &startingPositions_kdTree )
{
	std::vector<FElement*> toKeep;

	float radius_sq = radius * radius;

	for(FElement * seed : seeds)
	{
		Eigen::Vector3f p = seed->position;


		PositionFace facePair( p, seed->faceIndex );

		if(!_occlusionTester->isVisible( facePair, seed->radius ))
			continue;

		// keep the point if we near a starting point
		{
			size_t index = 0;
			float distance = 0.0f;

			startingPositions_kdTree.knnSearch( &p( 0 ), 1, &index, &distance );

			bool nearStarting = distance < radius_sq;

			if(nearStarting)
			{
				toKeep.push_back( seed );
				continue;
			}
		}

		// keep the point if we are near a brush point
		{
			size_t index = 0;
			float distance = 0.0f;

			_brushIndex->knnSearch( &p( 0 ), 1, &index, &distance );

			bool nearBrush = distance < radius_sq;

			if(nearBrush)
				toKeep.push_back( seed );
		}
	}

	return toKeep;
}

AlgorithmResult Algorithm_RegionGrowing::generate( std::vector<PositionFace>& startingPositions, float radius /*= -1.0f*/, AABB limits /*= AABB() */ )
{
	// initialize (don't count towards round time)
	if(!_didInit)
	{
		AlgorithmResult result;

		_initialize();

		beginRound();

		for(auto start : startingPositions)
			_initializeOutput( start, limits, result );

		_expandFreeSpace( result.generated );

		endRound( result );

		return result;
	}
	else
	{
		beginRound();

		AlgorithmResult r = _generate( startingPositions, radius, limits );

		endRound( r );

		return r;
	}
}

void Algorithm_RegionGrowing::_initialize()
{
	Algorithm::_initialize();

	_initFreespacePoints();
}