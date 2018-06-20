// Copyright 2016, Timothy Davison. All rights reserved.

#pragma once

#include "GameFramework/Actor.h"

#include "TimStructBox.h"
#include "Algorithm/Element.h"

#include "NeighbourhoodParameters.h"

#include "ElementActor.generated.h"



UCLASS()
class REGIONGROWING_API AElementActor : public AStaticMeshActor
{
	GENERATED_BODY()
	
public:
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "Agent Library" )
	TArray<FTimStructBox> graphObjects;

    // overrides the inferred radius of an element
    UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "RG" )
    float overrideRadius = 1.0f;

    // whether the override radius is affected by the actor's scale
    UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "RG" )
    bool scaleOverrideRadius = true;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "RG" )
	float gradient = 0.0f;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "RG" )
	bool generative = true;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "RG" )
	FNeighbourhoodParameters generationParameters;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "RG" )
	FNeighbourhoodParameters optimizationParameters;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "RG" )
	float minAssignmentDistance = 1.0f;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "RG" )
	float freespaceRadius = 1.0f;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "RG" )
	float generationInnerRadius = 1.0f;

public:	
	// Sets default values for this actor's properties
	AElementActor();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	// Called every frame
	virtual void Tick( float DeltaSeconds ) override;

	void writeToElement( FElement& element );
	void readFromElement( FElement& element );
	void readFromActor( AElementActor* elementActor );
	
};
