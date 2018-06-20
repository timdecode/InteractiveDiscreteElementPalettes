//
//  Algorithm.h
//  RegionGrowing
//
//  Created by Timothy Davison on 2015-07-20.
//  Copyright (c) 2016 Timothy Davison. All rights reserved.
//

#pragma once

#include "Eigen/Dense"

#include "Algorithm/FreeSpace.h"

struct Observation
{
	Observation( int32 exampleIndex_in, OutputFreeSpaceCluster * cluster_in ) : exampleIndex( exampleIndex_in ), cluster( cluster_in ) {}

	int32 exampleIndex = -1; // a sample index into the cluster
	FElement * output = nullptr;

	OutputFreeSpaceCluster * cluster = nullptr;

	FElement* example() { return cluster->exampleCluster->offsetsAndElements[exampleIndex].second; }
	Eigen::Vector3f offset() { return cluster->exampleCluster->offsetsAndElements[exampleIndex].first; }

	bool isValid() { return exampleIndex >= 0;  }
};

class Algorithm_Clustered : public Algorithm
{
public:
	uint32 numRounds = 5;

protected:
	std::unordered_map<FElement*, std::vector<FreeSpaceCluster>> _exemplarClusters;

	std::vector<OutputFreeSpaceCluster> _freeSpaceHorizon;

	std::unordered_map<FElement*, FElement*> _nearestGenerativeExample;

	FRandomStream _randomStream;

protected:


public:
	virtual ~Algorithm_Clustered() {}
	virtual AlgorithmResult generate( std::vector<PositionFace>& startingPositions, float radius, AABB limits ) override;

protected:
	virtual void _initialize();

	void _initGenerativeExamples();
	void _primeOutput( PositionFace& position, AABB& limits, AlgorithmResult& result );

	void _initExemplarClusters();

	void _instantiateObservation( Observation& observation, rg::Mapping::Result& mappingResult );

	void _rebuildFreespaceIndex();
	void _calculateFreeSpace( std::vector<FElement*>& newElements );

	bool _canPlace( FElement * example, rg::Mapping::Result& mappingResult, AABB& limits );

	void setSourceExample( FElement * output, FElement * source );
	FElement* sourceExample( FElement * output );
};