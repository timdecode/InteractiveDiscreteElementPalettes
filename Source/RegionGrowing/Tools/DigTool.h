// Copyright 2017 Timothy Davison, all rights reserved.

#pragma once

#include "VRTool.h"

#include "VolumeComponent.h"

#include "DigTool.generated.h"

UENUM( BlueprintType )
enum class EDigMode : uint8
{
	Adding UMETA( DisplayName = "Adding" ),
	Removing UMETA( DisplayName = "Removing" ),
	Smoothing UMETA( DisplayName = "Smoothing" )
};

/**
 * 
 */
UCLASS( Blueprintable )
class REGIONGROWING_API UDigTool : public UTool
{
	GENERATED_BODY()

public:

	void init(
		UPrimitiveComponent * leftSelectionPoint_in,
		UPrimitiveComponent * rightSelectionPoint_in,
		USceneComponent * targetComponent_in,
		URegionGrowingComponent * regionGrowingComponent_in 
	);

	virtual void oneHandStart( UPrimitiveComponent * hand ) override;
	virtual void oneHandEnd( UPrimitiveComponent * hand ) override;

	virtual void tickOneHand( float dt, UPrimitiveComponent * hand, FTransform lastTransform ) override;

	virtual void focused() override;
	virtual void loseFocus() override;

	virtual void faceDown_released() override;
	virtual void faceUp_released( USceneComponent * interactionPoint = nullptr ) override;

	void setDigMode( EDigMode mode );
	EDigMode getDigMode() { return _digMode; }

public:
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	float radius = 5.0f; // cell units

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	UStaticMesh * selectionMesh;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	UMaterialInterface * selectionMeshMaterial;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	float selectionMeshScaleFactor = 2.0f;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	TSubclassOf<class UUserWidget> widgetClass;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	TSubclassOf<class UUserWidget> selectionPointWidgetClass;

	virtual TSubclassOf<class UUserWidget> getWidgetClass() { return widgetClass; }

	virtual TSubclassOf<class UUserWidget> getSelectionWidgetClass() { return selectionPointWidgetClass; }

	UPROPERTY()
	URegionGrowingComponent * regionGrowingComponent = nullptr;

protected:
	void _dig( class UVolumeComponent * volume, FVector volumePosition, float radius );
	void _smooth( UVolumeComponent * volume, FVector volumePosition, float volumeRadius );

	void _toggleAddRemove();

	void _createSelectionMeshComponent( UPrimitiveComponent * selectionPoint );

	void _destroySelectionMeshComponent();

	float _brushRadius();

	void _convolve( UniformGrid<float>& grid, FIntVector& index );

protected:
	UPROPERTY()
	EDigMode _digMode = EDigMode::Adding;

	UPROPERTY()
	UStaticMeshComponent * _selectionMeshComponent;
};
