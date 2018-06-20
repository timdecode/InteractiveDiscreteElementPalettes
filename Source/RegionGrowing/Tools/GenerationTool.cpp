// Copyright 2018, Timothy Davison. All rights reserved.

#include "RegionGrowing.h"

#include "WidgetComponent.h"

#include "GenerationTool.h"

UGenerativeBrushTool::~UGenerativeBrushTool()
{
	_destroyBrushMeshComponent();
}

void UGenerativeBrushTool::focused()
{
	{
		widgetComponent->SetWidgetClass( widgetClass );

		UUserWidget * widget = widgetComponent->GetUserWidgetObject();

		if(!widget->Implements<UGenerativeBrushToolDelegate>())
			return;

		IGenerativeBrushToolDelegate::Execute_didChangeTickMode( widget, _tickMode, _tickMode );
		IGenerativeBrushToolDelegate::Execute_didChangeDrawMode( widget, _drawMode, _drawMode );
	}

	{
		selectionPointWidgetComponent->SetWidgetClass( selectionPointWidgetClass );

		UUserWidget * widget = selectionPointWidgetComponent->GetUserWidgetObject();

		if(!widget->Implements<UGenerativeBrushToolDelegate>())
			return;

		IGenerativeBrushToolDelegate::Execute_didChangeTickMode( widget, _tickMode, _tickMode );
		IGenerativeBrushToolDelegate::Execute_didChangeDrawMode( widget, _drawMode, _drawMode );
	}
}

void UGenerativeBrushTool::loseFocus()
{
	if( _brushMeshComponent )
		_brushMeshComponent->SetRelativeScale3D( FVector::ZeroVector );
}

void UGenerativeBrushTool::oneHandStart( UPrimitiveComponent * hand )
{
	regionGrowingComponent->startPaint();

	_createBrushMeshComponent( hand );
}

void UGenerativeBrushTool::oneHandEnd( UPrimitiveComponent * hand )
{
	regionGrowingComponent->endPaint();
}

bool UGenerativeBrushTool::_isVolumetric()
{
	if(!regionGrowingComponent)
		return false;

	return regionGrowingComponent->generationMode == EGenerationMode::SpacePainting;
}

float UGenerativeBrushTool::_brushRadius()
{
	return 2.0f + 6.0 * selectionATriggerValue();
}

void UGenerativeBrushTool::tickOneHand( float dt, UPrimitiveComponent * hand, FTransform lastTransform )
{
	if(!regionGrowingComponent || !_brushMeshComponent)
		return;

	float radius = _brushRadius();

	_brushMeshComponent->SetRelativeScale3D( FVector( radius * brushMeshScaleFactor ) );

	regionGrowingComponent->brushSize = radius;

	if(_tickMode == EGenerativeTickMode::Generating)
		_tickOneHand_generate( dt, hand, lastTransform );
	else
		_tickOneHand_erase( dt, hand, lastTransform );
}

void UGenerativeBrushTool::faceDown_released()
{
	setTickMode( _tickMode == EGenerativeTickMode::Generating ? EGenerativeTickMode::Erasing : EGenerativeTickMode::Generating );
}

void UGenerativeBrushTool::faceUp_released(USceneComponent * interactionPoint /*= nullptr*/)
{
	setDrawMode( _drawMode == EGenerativeDrawMode::Surface ? EGenerativeDrawMode::Volumetric : EGenerativeDrawMode::Surface );
}

void UGenerativeBrushTool::setDrawMode( EGenerativeDrawMode mode )
{
	auto oldMode = _drawMode;
	
	_drawMode = mode;

	UUserWidget * widget = widgetComponent->GetUserWidgetObject();

	if(!widget || !widget->Implements<UGenerativeBrushToolDelegate>())
		return;

	IGenerativeBrushToolDelegate::Execute_didChangeDrawMode( widget, mode, oldMode );
}

void UGenerativeBrushTool::setTickMode( EGenerativeTickMode mode )
{
	auto oldMode = _tickMode;

	// toggle the tick mode
	_tickMode = mode;

	{
		UUserWidget * widget = widgetComponent->GetUserWidgetObject();

		if(!widget || !widget->Implements<UGenerativeBrushToolDelegate>())
			return;

		IGenerativeBrushToolDelegate::Execute_didChangeTickMode( widget, mode, oldMode );
	}

	{
		UUserWidget * widget = selectionPointWidgetComponent->GetUserWidgetObject();

		if(!widget || !widget->Implements<UGenerativeBrushToolDelegate>())
			return;

		IGenerativeBrushToolDelegate::Execute_didChangeTickMode( widget, mode, oldMode );
	}
}

void UGenerativeBrushTool::_tickOneHand_generate( float dt, UPrimitiveComponent * hand, FTransform lastTransform )
{
	auto meshInterface = regionGrowingComponent->meshInterface();

	FVector location = hand->GetComponentLocation();

	if(_drawMode == EGenerativeDrawMode::Volumetric )
	{
		regionGrowingComponent->generationMode = EGenerationMode::SpacePainting;
		regionGrowingComponent->addBrushPoint( location );
	}
	else if( _drawMode == EGenerativeDrawMode::Surface )
	{
		auto nearest = meshInterface->nearestPointOnMesh( location );

		regionGrowingComponent->generationMode = EGenerationMode::SurfacePainting;
		regionGrowingComponent->addBrushPoint( nearest.point, nearest.faceIndex );
	}
}

void UGenerativeBrushTool::_tickOneHand_erase( float dt, UPrimitiveComponent * hand, FTransform lastTransform )
{
	auto meshInterface = regionGrowingComponent->meshInterface();

	FVector location = hand->GetComponentLocation();

	float radius = _brushRadius();

	if(_isVolumetric())
	{
		regionGrowingComponent->eraseInRadiusAt( location, radius );
	}
	else
	{
		auto nearest = meshInterface->nearestPointOnMesh( location );

		regionGrowingComponent->eraseInRadiusAt( nearest.point, radius, nearest.faceIndex );
	}
}


void UGenerativeBrushTool::_createBrushMeshComponent( UPrimitiveComponent * selectionPoint )
{
	if(_brushMeshComponent)
		return;

	AActor * actor = selectionPoint->GetOwner();

	_brushMeshComponent = NewObject<UStaticMeshComponent>( actor );
	
	_brushMeshComponent->AttachToComponent( selectionPoint, FAttachmentTransformRules::KeepRelativeTransform );
	_brushMeshComponent->SetStaticMesh( brushMesh );
	_brushMeshComponent->SetMaterial( 0, brushMeshMaterial );
	_brushMeshComponent->SetVisibility( true );
	_brushMeshComponent->SetRelativeScale3D( FVector( _brushRadius() * brushMeshScaleFactor ) );

	_brushMeshComponent->RegisterComponent();
}

void UGenerativeBrushTool::_destroyBrushMeshComponent()
{
	if(_brushMeshComponent == nullptr || !_brushMeshComponent->IsValidLowLevel())
		return;

	_brushMeshComponent->DestroyComponent();

	_brushMeshComponent = nullptr;
}

