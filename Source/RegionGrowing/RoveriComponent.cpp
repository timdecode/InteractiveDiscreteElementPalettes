// Fill out your copyright notice in the Description page of Project Settings.

#include "RegionGrowing.h"
#include "RoveriComponent.h"

#include "Algorithm/Algorithm_MyRoveri.h"


Algorithm* URoveriComponent::getOrCreateAlgorithm()
{
	return &_algorithmRoveri;
}

void URoveriComponent::loadParameters()
{
	Super::loadParameters();


}

void URoveriComponent::initAlgorithms()
{
	Super::initAlgorithms();

	_algorithmRoveri.init( _threadPool );
}
