//
//  Algorith_Roveri.cpp
//  RegionGrowing
//
//  Created by Timothy Davison on 2018-05-24.
//  Copyright (c) 2018 Timothy Davison. All rights reserved.


#include "RegionGrowing.h"

#include "Algorithm/Algorithm_Roveri.h"

#define _USE_MATH_DEFINES // for C++  
#include <cmath>  

using namespace Eigen;

void Algorithm_Roveri::_initialize()
{
	Algorithm::_initialize();

	simulation.clear();

	InputCloud& inputCloud = simulation.inputCloud;
	{
		for(auto& ePtr : _exemplar)
		{
			std::vector<Vector3f> attributes;

			attributes.emplace_back( Vector3f( 0.0f, 0.0f, 1.0f ) ); // normal
			attributes.emplace_back( Vector3f( 1.0f, 0.0f, 0.0f ) );			 // type attributes

			Vector3f position = ePtr->position;

			inputCloud.cloud.samples.emplace_back( position, attributes );
		}

		inputCloud.initCloud();
	}

	simulation.init();

	BackgroundGrid& backgroundGrid = simulation.backgroundGrid;
	{
		backgroundGrid.generateBackgroundPoints( simulation.output_canvas_size, simulation.neighSize, simulation.is_2d, simulation.neighSize );
		for(int i = 0; i < backgroundGrid.points.size(); i++) {
			int rand_input_sample_index = rand() % inputCloud.cloud.samples.size();

			backgroundGrid.points[i].matching_position = inputCloud.cloud.samples[rand_input_sample_index].position;
		}
	}
}

AlgorithmResult Algorithm_Roveri::generate( std::vector<PositionFace>& positions, float radius /*= -1.0f*/, AABB limits /*= AABB() */ )
{
	AlgorithmResult result;

	if(!_didInit)
		_initialize();

	simulation.run();

	Cloud& cloud = simulation.outputCloud.cloud;

	int sizeDifference = cloud.samples.size() - _output.size();

	AlgorithmResult generationResult;
	// add elements
	if(sizeDifference > 0)
	{
		size_t numToAdd = sizeDifference;
		size_t cloudStart = cloud.samples.size() - numToAdd;

		for(size_t i = 0; i < numToAdd; ++i)
		{
			FElement templateElement;
			templateElement.position = cloud.samples[cloudStart + i].position;

			size_t elementIndex = _output.size();
			size_t entityIndex = _output.entitySize();

			FElement * element = _output.add( templateElement );
			element->entityIndex = entityIndex;

			Entity& entity = _output.emplaceEntityBack();			
			entity.elementIndices.push_back( elementIndex );

			generationResult.generated.push_back( element );
		}
	}
	// remove elements
	else if( sizeDifference < 0 )
	{
		size_t numToRemove = -sizeDifference;

		std::vector<FElement*> toRemove;

		size_t endIndex = _output.size() - 1;

		for(size_t i = 0; i < numToRemove; ++i)
		{
			FElement * element = _output.elementAt( endIndex - i );
			toRemove.push_back( element );

			generationResult.removed.push_back( element );
		}

		_output.erase( toRemove );
	}
	 
	// copy data to elements
	{
		size_t i = 0;
		for(auto& sample : cloud.samples)
		{
			FElement * element = _output.elementAt( i );

			element->position = sample.position;

			++i;
		}
	}


	// everything is modified
	for(auto& e : _output)
	{
		result.modified.push_back( e.get() );
	}

	result.append( generationResult );

	return result;
}


void Algorithm_Roveri::clear()
{
	Algorithm::clear();

	simulation.clear();
}

void Algorithm_Roveri::loadExemplar()
{

}

#undef _USE_MATH_DEFINES


