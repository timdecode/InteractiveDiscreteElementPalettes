// Copyright (c) 2018 Timothy Davison. All rights reserved.

#pragma once

#include "Components/ActorComponent.h"

#include "RegionGrowingComponent.h"

#include "Algorithm/Algorithm_Roveri.h"

#include "RoveriComponent.generated.h"

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class REGIONGROWING_API URoveriComponent : public URegionGrowingComponent
{
	GENERATED_BODY()

public:
	virtual Algorithm* getOrCreateAlgorithm() override;
	virtual void loadParameters() override;
	virtual void initAlgorithms() override;

protected:
	Algorithm_Roveri _algorithmRoveri;
};

