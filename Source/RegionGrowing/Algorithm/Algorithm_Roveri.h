//
//  Algorith_Roveri.h
//  RegionGrowing
//
//  Created by Timothy Davison on 2018-05-24.
//  Copyright (c) 2018 Timothy Davison. All rights reserved.
//
//  This is a wrapper around Roveri's SimulationManager from his 2015 Eurographics paper, Example-Based Repetitive Structure Synthesis

#pragma once

#include "Algorithm/Algorithm.h"
#include "Algorithm/SimpleUniformGrid.h"

#include "Algorithm/Roveri/SimulationManager.h"

#include <random>
#include <iostream>

class Algorithm_Roveri : public Algorithm
{
public:
	virtual ~Algorithm_Roveri() {}


	virtual AlgorithmResult generate( std::vector<PositionFace>& positions, float radius = -1.0f, AABB limits = AABB() );

	virtual void clear() override;

	virtual void loadExemplar() override;

public:
	// Exposed publicly to set parameters on the simulation. However, don't mess with the inputCloud or backgroundGrid directly, that's
	// what this wrapper does internally.
	SimulationManager simulation;

protected:
	virtual void _initialize() override;
};