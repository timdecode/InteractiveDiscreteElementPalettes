// Copyright 2017 Timothy Davison, all rights reserved.

#include "RegionGrowing.h"

#include "DigTool.h"
#include "VolumeComponent.h"

void UDigTool::init( UPrimitiveComponent * leftSelectionPoint_in, UPrimitiveComponent * rightSelectionPoint_in, USceneComponent * targetComponent_in, URegionGrowingComponent * regionGrowingComponent_in )
{
	UTool::init( leftSelectionPoint_in, rightSelectionPoint_in, targetComponent_in );

	regionGrowingComponent = regionGrowingComponent_in;
}

void UDigTool::oneHandStart( UPrimitiveComponent * hand )
{
	_createSelectionMeshComponent( hand );
}

void UDigTool::oneHandEnd( UPrimitiveComponent * hand )
{
	_destroySelectionMeshComponent();
}

float UDigTool::_brushRadius()
{
	return (radius * selectionATriggerValue() * 0.9f) + 0.1f;
}

void UDigTool::tickOneHand( float dt, UPrimitiveComponent * hand, FTransform lastTransform )
{
	if(_selectionMeshComponent)
	{
		_selectionMeshComponent->SetRelativeScale3D( FVector( _brushRadius() * selectionMeshScaleFactor ) );
	}

	UWorld * world = hand->GetWorld();

	TArray<FOverlapResult> overlaps;

	FCollisionShape collisionShape = FCollisionShape::MakeSphere( 20.0f );

	FCollisionQueryParams params;
	params.bTraceComplex = false; // we want convex collision, not per triangle hits

	world->OverlapMultiByChannel( overlaps,
		hand->GetComponentLocation(),
		hand->GetComponentRotation().Quaternion(),
		ECC_WorldStatic,
		collisionShape,
		params );

	for(FOverlapResult& result : overlaps)
	{
		AActor * actor = result.GetActor();

		UVolumeComponent * volume = actor->FindComponentByClass<UVolumeComponent>();

		if(!volume)
			continue;

		FTransform transform = actor->GetRootComponent()->GetComponentToWorld();

		FVector volumePosition = transform.InverseTransformPosition( hand->GetComponentLocation() );

		float transformScale = transform.GetMaximumAxisScale();
		transformScale = FMath::IsNearlyZero( transformScale ) ? 1.0f : transformScale;

		float volumeRadius = _brushRadius() / transformScale;

		if(_digMode == EDigMode::Adding || _digMode == EDigMode::Removing)
			_dig( volume, volumePosition, volumeRadius );
		else if(_digMode == EDigMode::Smoothing)
			_smooth( volume, volumePosition, volumeRadius );
	}
}

void UDigTool::focused()
{
	
}

void UDigTool::loseFocus()
{
}

void UDigTool::_toggleAddRemove()
{
	EDigMode newMode = _digMode == EDigMode::Adding ? EDigMode::Removing : EDigMode::Adding;

	setDigMode( newMode );
}

void UDigTool::faceDown_released()
{
	_toggleAddRemove();
}

void UDigTool::faceUp_released( USceneComponent * interactionPoint /* = nullptr */ )
{
	setDigMode( EDigMode::Smoothing );
}

void UDigTool::setDigMode( EDigMode mode )
{
	_digMode = mode;
}

void UDigTool::_createSelectionMeshComponent( UPrimitiveComponent * selectionPoint )
{
	if(_selectionMeshComponent)
		return;

	_selectionMeshComponent = NewObject<UStaticMeshComponent>( selectionPoint );
	_selectionMeshComponent->AttachToComponent( selectionPoint, FAttachmentTransformRules::KeepRelativeTransform );
	_selectionMeshComponent->SetStaticMesh( selectionMesh );
	_selectionMeshComponent->SetMaterial( 0, selectionMeshMaterial );
	_selectionMeshComponent->SetRelativeScale3D( FVector( _brushRadius() * selectionMeshScaleFactor ) );


	_selectionMeshComponent->RegisterComponent();
}

void UDigTool::_destroySelectionMeshComponent()
{
	if(_selectionMeshComponent == nullptr || !_selectionMeshComponent->IsValidLowLevel())
		return;

	_selectionMeshComponent->DestroyComponent();

	_selectionMeshComponent = nullptr;
}

void UDigTool::_smooth( UVolumeComponent * volume, FVector volumePosition, float volumeRadius )
{
	UniformGrid<float>& grid = volume->grid();

	FVector cellSize = grid.cellSize();

	FVector extents = cellSize * volumeRadius;

	FIntVector start = volume->componentToIndex( volumePosition - extents );
	FIntVector end = volume->componentToIndex( volumePosition + extents );

	start = grid.clampedIndex( start );
	end = grid.clampedIndex( end );

	FIntVector index = start;

	const float rSqrd = volumeRadius * volumeRadius;

	// visit the cells
	for(index.Z = start.Z; index.Z <= end.Z; index.Z++)
	{
		for(index.Y = start.Y; index.Y <= end.Y; index.Y++)
		{
			for(index.X = start.X; index.X <= end.X; index.X++)
			{
				FVector p = grid.samplePoint( index );

				float d = FVector::DistSquared( p, volumePosition );

				if(d > rSqrd)
					continue;

				_convolve( grid, index );
			}
		}
	}

	FVector startP = volume->indexToComponent( start );
	FVector endP = volume->indexToComponent( end );

	volume->markDirty( startP - 2, endP + 2 );
}

void UDigTool::_convolve( UniformGrid<float>& grid, FIntVector& index )
{
	FIntVector offset = FIntVector::ZeroValue;

	float sum = 0.0f;
	for(offset.Z = -1; offset.Z <= 1; offset.Z++)
	{
		for(offset.Y = -1; offset.Y <= 1; offset.Y++)
		{
			for(offset.X = -1; offset.X <= 1; offset.X++)
			{
				FIntVector i = index + offset;

				sum += grid( i );
			}
		}
	}

	sum /= 9.0f;

	grid( index ) = sum;
}

void UDigTool::_dig( UVolumeComponent * volume, FVector volumePosition, float volumeRadius )
{
	UniformGrid<float>& grid = volume->grid();

	FVector cellSize = grid.cellSize();

	FVector extents = cellSize * volumeRadius;

	FIntVector start = volume->componentToIndex( volumePosition - extents );
	FIntVector end = volume->componentToIndex( volumePosition + extents );

	start = grid.clampedIndex( start );
	end = grid.clampedIndex( end );

	FIntVector index = start;

	const float rSqrd = volumeRadius * volumeRadius;

	float newGridValue = _digMode == EDigMode::Adding ? 100.0f : 0.0f;

	for( index.Z = start.Z; index.Z <= end.Z; index.Z++)
	{
		for( index.Y = start.Y; index.Y <= end.Y; index.Y++)
		{
			for( index.X = start.X; index.X <= end.X; index.X++)
			{
				FVector p = grid.samplePoint( index );

				float d = FVector::DistSquared( p, volumePosition );

				if(d > rSqrd)
					continue; 

				grid( index.X, index.Y, index.Z ) = newGridValue;
			}
		}
	}

	FVector startP = volume->indexToComponent( start );
	FVector endP = volume->indexToComponent( end );

	volume->markDirty( startP - 1.0f, endP + 1.0f );
}
