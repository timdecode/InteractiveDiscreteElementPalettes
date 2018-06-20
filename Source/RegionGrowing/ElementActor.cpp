// Copyright 2016, Timothy Davison. All rights reserved.

#include "RegionGrowing.h"
#include "ElementActor.h"


// Sets default values
AElementActor::AElementActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

    
}

// Called when the game starts or when spawned
void AElementActor::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AElementActor::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );

}

void AElementActor::writeToElement( FElement& element )
{
	element.radius = overrideRadius;

	USceneComponent * sceneComponent = this->GetRootComponent();

	if(scaleOverrideRadius)
		element.radius *= sceneComponent->GetComponentTransform().GetMaximumAxisScale();

	element.generative = generative;

	element.generationParameters = generationParameters;
	element.optimizationParameters = optimizationParameters;

	element.minAssignmentDistance = minAssignmentDistance;
	element.freespaceRadius = freespaceRadius;

	element.generationInnerRadius = generationInnerRadius;

	element.hackScale = sceneComponent->GetComponentScale().X;

	element.graphObjects = graphObjects;
}

void AElementActor::readFromElement( FElement& element )
{
	overrideRadius = element.radius;
	scaleOverrideRadius = true;
	generative = element.generative;

	generationParameters = element.generationParameters;
	optimizationParameters = element.optimizationParameters;

	minAssignmentDistance = element.minAssignmentDistance;
	freespaceRadius = element.freespaceRadius;

	generationInnerRadius = element.generationInnerRadius;

	USceneComponent * sceneComponent = GetRootComponent();
	sceneComponent->SetWorldScale3D( FVector( element.hackScale ) );

	graphObjects = element.graphObjects;
}

void AElementActor::readFromActor( AElementActor* elementActor )
{
	overrideRadius = elementActor->overrideRadius;
	scaleOverrideRadius = elementActor->scaleOverrideRadius;
	gradient = elementActor->gradient;
	generative = elementActor->generative;

	generationParameters = elementActor->generationParameters;
	optimizationParameters = elementActor->optimizationParameters;

	minAssignmentDistance = elementActor->minAssignmentDistance;
	freespaceRadius = elementActor->freespaceRadius;
	generationInnerRadius = elementActor->generationInnerRadius;

	GetRootComponent()->SetWorldScale3D( elementActor->GetRootComponent()->GetComponentScale() );

	graphObjects = elementActor->graphObjects;
}

