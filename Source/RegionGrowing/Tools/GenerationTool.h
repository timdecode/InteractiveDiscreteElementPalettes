// Copyright 2018, Timothy Davison. All rights reserved.

#pragma once

#include "VRTool.h"

#include "GenerationTool.generated.h"

UENUM( BlueprintType )
enum class EGenerativeTickMode : uint8
{
	Generating,
	Erasing,
};

UENUM( BlueprintType )
enum class EGenerativeDrawMode : uint8
{
	Volumetric,
	Surface
};

UINTERFACE( BlueprintType )
class UGenerativeBrushToolDelegate : public UInterface
{
	GENERATED_BODY()
};

class IGenerativeBrushToolDelegate
{
	GENERATED_BODY()

public:
	UFUNCTION( BlueprintCallable, BlueprintImplementableEvent, Category = "LifeBrush" )
	void setGenerativeBrush( class UGenerativeBrushTool * brush );

	UFUNCTION( BlueprintCallable, BlueprintImplementableEvent, Category = "LifeBrush" )
	void didChangeDrawMode( EGenerativeDrawMode newMode, EGenerativeDrawMode oldMode );

	UFUNCTION( BlueprintCallable, BlueprintImplementableEvent, Category = "LifeBrush" )
	void didChangeTickMode( EGenerativeTickMode newMode, EGenerativeTickMode oldMode );
};

UCLASS( Blueprintable )
class UGenerativeBrushTool : public UTool
{
	GENERATED_BODY()

public:
	virtual ~UGenerativeBrushTool();

	URegionGrowingComponent * regionGrowingComponent = nullptr;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	UStaticMesh * brushMesh;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	UMaterialInterface * brushMeshMaterial;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor", meta = (MustImplement="GenerativeBrushToolDelegate") )
	TSubclassOf<class UUserWidget> widgetClass;

	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor", meta = (MustImplement = "GenerativeBrushToolDelegate") )
	TSubclassOf<class UUserWidget> selectionPointWidgetClass;

	virtual TSubclassOf<class UUserWidget> getWidgetClass() { return widgetClass; }

	virtual TSubclassOf<class UUserWidget> getSelectionWidgetClass() { return selectionPointWidgetClass; }

	// The brushMesh will be scaled by the trigger value and the scale factor. The size of the mesh component should
	// match the brush radius.
	UPROPERTY( EditAnywhere, BlueprintReadWrite, Category = "ShipEditor" )
	float brushMeshScaleFactor = 0.01f;

public:
	UFUNCTION( BlueprintCallable, Category = Generation )
	void setDrawMode( EGenerativeDrawMode mode );

	UFUNCTION( BlueprintCallable, Category = Generation )
	EGenerativeDrawMode getDrawMode() { return _drawMode; }

	UFUNCTION( BlueprintCallable, Category = Generation )
	void setTickMode( EGenerativeTickMode mode );

	UFUNCTION( BlueprintCallable, Category = Generation )
	EGenerativeTickMode getTickMode() { return _tickMode; }

public:
	void init(
		UPrimitiveComponent * leftSelectionPoint_in,
		UPrimitiveComponent * rightSelectionPoint_in,
		USceneComponent * targetComponent_in,
		URegionGrowingComponent * regionGrowingComponent_in )
	{
		UTool::init( leftSelectionPoint_in, rightSelectionPoint_in, targetComponent_in );

		regionGrowingComponent = regionGrowingComponent_in;
	}

	virtual void focused() override;
	virtual void loseFocus() override;

	virtual void oneHandStart( UPrimitiveComponent * hand ) override;
	virtual void oneHandEnd( UPrimitiveComponent * hand ) override;

	virtual void twoHandStart( UPrimitiveComponent * handA, UPrimitiveComponent * handB ) override {}
	virtual void twoHandEnd( UPrimitiveComponent * handA, UPrimitiveComponent * handB ) override {}

	virtual void tickOneHand( float dt, UPrimitiveComponent * hand, FTransform lastTransform ) override;

	virtual void tickTwoHand
	(
		float dt,
		UPrimitiveComponent * handA,
		UPrimitiveComponent * handB,
		FTransform transformA,
		FTransform transformB
	) override {}

	virtual void faceDown_released() override;
	virtual void faceUp_released(USceneComponent * interactionPoint = nullptr) override;







protected:
	bool _isVolumetric();

	float _brushRadius();

	void _tickOneHand_generate( float dt, UPrimitiveComponent * hand, FTransform lastTransform );
	void _tickOneHand_erase( float dt, UPrimitiveComponent * hand, FTransform lastTransform );

	void _createBrushMeshComponent( UPrimitiveComponent * selectionPoint );
	void _destroyBrushMeshComponent();

	UPROPERTY()
	UStaticMeshComponent * _brushMeshComponent;

	EGenerativeDrawMode _drawMode = EGenerativeDrawMode::Volumetric;
	EGenerativeTickMode _tickMode = EGenerativeTickMode::Generating;
};
