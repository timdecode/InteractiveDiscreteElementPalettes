// Copyright (c) 2018 Timothy Davison. All rights reserved.

#include "RegionGrowing.h"
#include "MyRoveriComponent.h"

#include "Algorithm/Algorithm_MyRoveri.h"


Algorithm* UMyRoveriComponent::getOrCreateAlgorithm()
{
	return &_algorithmRoveri;
}

void UMyRoveriComponent::loadParameters()
{
	Super::loadParameters();

	_algorithmRoveri.sampleSpacing = sampleSpacing;
	_algorithmRoveri.sigma = sigma;
	_algorithmRoveri.learningRate = learningRate;
	_algorithmRoveri.nRandomStarts = nRandomStarts;
	_algorithmRoveri.use2DSynthesis = use2DSynthesis;
	_algorithmRoveri.samplingControlRadius = samplingControlRadius;
	_algorithmRoveri.debug_outputTotalSimilarity = debug_outputTotalSimilarity;
	_algorithmRoveri.debug_OptimizationMLogging = debug_OptimizationMLogging;

	_algorithmRoveri.gradientDescentMaxIterations = gradientDescentMaxIterations;
	_algorithmRoveri.gradientDescentPrecision = gradientDescentPrecision;
}

void UMyRoveriComponent::initAlgorithms()
{
	Super::initAlgorithms();

	_algorithmRoveri.init( _threadPool );
}
