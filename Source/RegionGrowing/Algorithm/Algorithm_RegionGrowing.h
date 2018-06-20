//
//  Algorith_RegionGrowing.h
//  RegionGrowing
//
//  Created by Timothy Davison on 2018-01-04.
//  Copyright (c) 2018 Timothy Davison. All rights reserved.
//

#pragma once

#include "Algorithm/Algorithm.h"

class Algorithm_RegionGrowing : public Algorithm
{
public:
	virtual ~Algorithm_RegionGrowing() {}

	virtual AlgorithmResult generate( std::vector<PositionFace>& positions, float radius = -1.0f, AABB limits = AABB() );

protected:
	virtual void _initialize();
	AlgorithmResult _reassignSourceExamples();
	AlgorithmResult _generate( std::vector<PositionFace>& startingPositions, float radius /*= -1.0f*/, AABB limits /*= AABB() */ );

	std::vector<FElement *> _constrainSeedsToBounds( float radius, std::vector<FElement *>& seeds, EigenDynamicPointCloudIndexAdaptor &startingPositions_kdTree );

};

struct QuadraturePoint
{
	Eigen::Vector3f outputPosition;

	Eigen::Vector3f examplePosition;
};