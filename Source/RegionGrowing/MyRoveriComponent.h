// Copyright (c) 2018 Timothy Davison. All rights reserved.

#pragma once

#include "Components/ActorComponent.h"

#include "RegionGrowingComponent.h"

#include "Algorithm/Algorithm_MyRoveri.h"

#include "MyRoveriComponent.generated.h"

class URuntimeMeshComponent;
class AElementActor;

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class REGIONGROWING_API UMyRoveriComponent : public URegionGrowingComponent
{
	GENERATED_BODY()

public:
	UPROPERTY( EditAnywhere ) float sampleSpacing = 1.0f;
	UPROPERTY( EditAnywhere ) float sigma = 0.5f;
	UPROPERTY( EditAnywhere ) float learningRate = 0.1f;
	UPROPERTY( EditAnywhere ) int32 nRandomStarts = 5;
	UPROPERTY( EditAnywhere ) bool use2DSynthesis = true;
	UPROPERTY( EditAnywhere ) float samplingControlRadius = 0.5f;

	UPROPERTY( EditAnywhere ) bool debug_outputTotalSimilarity = false;
	UPROPERTY( EditAnywhere ) bool debug_OptimizationMLogging = false;

	UPROPERTY( EditAnywhere ) int gradientDescentMaxIterations = 50;
	UPROPERTY( EditAnywhere ) float gradientDescentPrecision = 0.001f;

public:
	virtual Algorithm* getOrCreateAlgorithm() override;
	virtual void loadParameters() override;
	virtual void initAlgorithms() override;

protected:
	Algorithm_MyRoveri _algorithmRoveri;
};

