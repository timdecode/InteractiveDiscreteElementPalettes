// Copyright 2018, Timothy Davison. All rights reserved.

#include "RegionGrowing.h"

#include "WidgetComponent.h"

#include "MultiSelectVRPopup.h"

#include "SelectionTool.h"


USelectionTool::~USelectionTool()
{
	_destroySelectionMeshComponent();
}

void USelectionTool::init( UPrimitiveComponent * leftSelectionPoint_in, UPrimitiveComponent * rightSelectionPoint_in, USceneComponent * targetComponent_in, URegionGrowingComponent * regionGrowingComponent_in )
{
	UTool::init( leftSelectionPoint_in, rightSelectionPoint_in, targetComponent_in );

	regionGrowingComponent = regionGrowingComponent_in;

	makeSelectionAActive();
}

void USelectionTool::postInit()
{
	if(regionGrowingComponent)
	{
		selectionPtrA = regionGrowingComponent->addExampleSelection( std::vector<AElementActor*>(), weightA );
		selectionPtrB = regionGrowingComponent->addExampleSelection( std::vector<AElementActor*>(), weightB );
	}
}

void USelectionTool::oneHandStart( UPrimitiveComponent * hand )
{
	_createSelectionMeshComponent( hand );
}

void USelectionTool::oneHandEnd( UPrimitiveComponent * hand )
{
	_destroySelectionMeshComponent();
}

float USelectionTool::_brushRadius()
{
	return 10.0f * selectionATriggerValue();
}

void USelectionTool::tickOneHand( float dt, UPrimitiveComponent * hand, FTransform lastTransform )
{
	if(!regionGrowingComponent)
		return;

	AActor * exemplar = regionGrowingComponent->exemplar;

	if(!exemplar)
		return;


	if(_selectionMeshComponent)
	{
		_selectionMeshComponent->SetRelativeScale3D( FVector( _brushRadius() * selectionMeshScaleFactor ) );
	}

	// perform an overlap test at the hand position to find actors that overlap
	TArray<FOverlapResult> overlaps;

	FCollisionShape collisionShape = FCollisionShape::MakeSphere( _brushRadius() );

	FCollisionQueryParams params;

	params.AddIgnoredActor( regionGrowingComponent->GetOwner() );
	params.AddIgnoredActor( hand->GetOwner() );

	bool overlap = regionGrowingComponent->GetWorld()->OverlapMultiByChannel(
		overlaps,
		hand->GetComponentLocation(),
		hand->GetComponentRotation().Quaternion(),
		ECollisionChannel::ECC_WorldDynamic,
		collisionShape,
		params
	);

	// update the selection
	for(FOverlapResult overlap : overlaps)
	{
		AElementActor * elementActor = Cast<AElementActor>( overlap.Actor.Get() );

		if(!elementActor)
			continue;

		if(!elementActor->GetRootComponent()->IsVisible())
			continue;

		if(selectionMode == ESelectionToolMode::Adding)
		{
			bool inSet = false;
			_selection->Add( elementActor, &inSet );

			UStaticMeshComponent * mesh = elementActor->FindComponentByClass<UStaticMeshComponent>();

			// we visualize selection through a post-process effect on the custom depth
			if(mesh && !inSet)
			{
				mesh->SetRenderCustomDepth( true );
			}
		}
		else if(selectionMode == ESelectionToolMode::Removing)
		{
			bool wasInSet = _selection->Contains( elementActor );

			_selection->Remove( elementActor );

			UStaticMeshComponent * mesh = elementActor->FindComponentByClass<UStaticMeshComponent>();

			if(mesh && wasInSet)
			{
				mesh->SetRenderCustomDepth( false );
			}
		}
	}
}

void USelectionTool::faceDown_released()
{
	if(!regionGrowingComponent)
		return;

	auto mode = selectionMode == ESelectionToolMode::Adding ? ESelectionToolMode::Removing : ESelectionToolMode::Adding;

	setSelectionMode( mode );
}

void USelectionTool::faceDown_touchStart()
{

}

void USelectionTool::faceDown_touchEnd()
{

}

void USelectionTool::faceUp_released(USceneComponent * interactionPoint /*= nullptr*/)
{
	if(!regionGrowingComponent || !interactionPoint)
		return;

	UWorld * world = regionGrowingComponent->GetWorld();


	FVector location = interactionPoint->GetComponentLocation();
	FQuat rotation = FQuat::Identity;

	FTransform transform( rotation, location, FVector( 1.0f ) );


	AMultiSelectVRPopup * multiSelectActor = world->SpawnActor<AMultiSelectVRPopup>( multiSelectActorClass, transform );
	multiSelectActor->setSelectionTool( this );

}

void USelectionTool::setSelectionMode( ESelectionToolMode mode )
{
	auto oldMode = selectionMode;

	selectionMode = mode;

	{
		UUserWidget * widget = widgetComponent->GetUserWidgetObject();

		if(!widget || !widget->Implements<USelectionToolDelegate>())
			return;

		ISelectionToolDelegate::Execute_didChangeSelectionMode( widget, mode, oldMode );
	}

	{
		UUserWidget * widget = selectionPointWidgetComponent->GetUserWidgetObject();

		if(!widget || !widget->Implements<USelectionToolDelegate>())
			return;

		ISelectionToolDelegate::Execute_didChangeSelectionMode( widget, mode, oldMode );
	}
}

void USelectionTool::focused()
{
	{
		widgetComponent->SetWidgetClass( widgetClass );

		UUserWidget * widget = widgetComponent->GetUserWidgetObject();

		if(!widget->Implements<USelectionToolDelegate>())
			return;

		ISelectionToolDelegate::Execute_didChangeSelectionMode( widget, selectionMode, selectionMode );
	}

	{
		selectionPointWidgetComponent->SetWidgetClass( selectionPointWidgetClass );

		UUserWidget * widget = selectionPointWidgetComponent->GetUserWidgetObject();

		if(!widget->Implements<USelectionToolDelegate>())
			return;

		ISelectionToolDelegate::Execute_didChangeSelectionMode( widget, selectionMode, selectionMode );
	}
}

std::vector<AElementActor*> USelectionTool::_toVector( TSet<AElementActor*> aSet )
{
	std::vector<AElementActor*> result;

	for(AElementActor * actor : aSet)
		result.push_back( actor );

	return result;
}

void USelectionTool::loseFocus()
{
	if(!regionGrowingComponent)
		return;

	std::vector<AElementActor*> vecA = _toVector( selectionA );
	std::vector<AElementActor*> vecB = _toVector( selectionB );

	regionGrowingComponent->updateExampleSelection( selectionPtrA, vecA, weightA );
	regionGrowingComponent->updateExampleSelection( selectionPtrB, vecB, weightB );

	// hide the brush mesh component
	if(_selectionMeshComponent)
		_selectionMeshComponent->SetRelativeScale3D( FVector::ZeroVector );
}

void USelectionTool::_createSelectionMeshComponent( UPrimitiveComponent * selectionPoint )
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

void USelectionTool::_destroySelectionMeshComponent()
{
	if(_selectionMeshComponent == nullptr || !_selectionMeshComponent->IsValidLowLevel())
		return;

	_selectionMeshComponent->DestroyComponent();

	_selectionMeshComponent = nullptr;
}

void USelectionTool::makeSelectionAActive()
{
	_hideSelection( selectionB );

	_selection = &selectionA;

	_showSelection( selectionA );
}

void USelectionTool::makeSelectionBActive()
{
	_hideSelection( selectionA );

	_selection = &selectionB;

	_showSelection( selectionB );
}

void USelectionTool::_hideSelection( TSet<AElementActor*>& selected )
{
	for(AElementActor * elementActor : selected)
	{
		UStaticMeshComponent * mesh = elementActor->FindComponentByClass<UStaticMeshComponent>();

		if( mesh )
		{
			mesh->SetRenderCustomDepth( false );
		}
	}
}

void USelectionTool::_showSelection( TSet<AElementActor*>& selected )
{
	for(AElementActor * elementActor : selected)
	{
		UStaticMeshComponent * mesh = elementActor->FindComponentByClass<UStaticMeshComponent>();

		if( mesh )
		{
			mesh->SetRenderCustomDepth( true );
		}
	}
}

void USelectionTool::clearSelection()
{
	_hideSelection( *_selection );

	_selection->Empty();
}