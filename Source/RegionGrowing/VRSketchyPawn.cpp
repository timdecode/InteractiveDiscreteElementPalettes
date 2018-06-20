// Copyright 2016, Timothy Davison. All rights reserved.

#include "RegionGrowing.h"

#include "MotionControllerComponent.h"
#include "RegionGrowingComponent.h"
#include "WidgetComponent.h"

#include "FlexElements.h"

#include "Tools/SelectionTool.h"
#include "Tools/GenerationTool.h"
#include "Tools/DigTool.h"

#include "VRSketchyPawn.h"



// Sets default values
AVRSketchyPawn::AVRSketchyPawn()
{
 	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	RootComponent = CreateDefaultSubobject<USceneComponent>( TEXT( "Root" ) );
	RootComponent->SetMobility( EComponentMobility::Movable );

	// copied from ShipEditor 2016-11-16 Copyright Timothy Davison of Code Monkey Castle, all rights reserved.
	vrScene = CreateDefaultSubobject<USceneComponent>( TEXT( "VRScene" ) );
	vrScene->SetMobility( EComponentMobility::Movable );
	vrScene->AttachToComponent( RootComponent, FAttachmentTransformRules::KeepRelativeTransform );

	leftController = CreateDefaultSubobject<UMotionControllerComponent>( TEXT( "LeftController" ) );
	leftController->Hand = EControllerHand::Left;
	leftController->SetCollisionProfileName( TEXT( "BlockAll" ) );
	leftController->AttachToComponent( vrScene, FAttachmentTransformRules::KeepRelativeTransform );

	leftInteractionPoint = CreateDefaultSubobject<UStaticMeshComponent>( TEXT( "LeftSelectionPoint" ) );
	leftInteractionPoint->AttachToComponent( leftController, FAttachmentTransformRules::KeepRelativeTransform );

	rightController = CreateDefaultSubobject<UMotionControllerComponent>( TEXT( "RightController" ) );
	rightController->Hand = EControllerHand::Right;
	rightController->SetCollisionProfileName( TEXT( "BlockAll" ) );
	rightController->AttachToComponent( vrScene, FAttachmentTransformRules::KeepRelativeTransform );

	rightInteractionPoint = CreateDefaultSubobject<UStaticMeshComponent>( TEXT( "RightSelectionPoint" ) );
	rightInteractionPoint->AttachToComponent( rightController, FAttachmentTransformRules::KeepRelativeTransform );

	rightPadWidget = CreateDefaultSubobject<UWidgetComponent>( TEXT( "RightWidgetComponent" ) );
	rightPadWidget->AttachToComponent( rightController, FAttachmentTransformRules::KeepRelativeTransform );

	rightSelectionPointWidget = CreateDefaultSubobject<UWidgetComponent>( TEXT( "RightSelectionPointWidgetComponent" ) );
	rightSelectionPointWidget->AttachToComponent( rightInteractionPoint, FAttachmentTransformRules::KeepRelativeTransform );

	camera = CreateDefaultSubobject<UCameraComponent>( TEXT( "Camera" ) );
	camera->AttachToComponent( vrScene, FAttachmentTransformRules::KeepRelativeTransform );

	generativeBrushTool = CreateDefaultSubobject<UGenerativeBrushTool>( TEXT( "GenerativeBrushTool" ) );
	selectionTool = CreateDefaultSubobject<USelectionTool>( TEXT( "SelectionTool" ) );
	digTool = CreateDefaultSubobject<UDigTool>( TEXT( "DigTool" ) );

	// necessary for VR 
	BaseEyeHeight = 0.0f;
}

// Called when the game starts or when spawned
void AVRSketchyPawn::BeginPlay()
{
	Super::BeginPlay();
	
	if(!regionGrowingActor)
		return;

	regionGrowingComponent = regionGrowingActor->FindComponentByClass<URegionGrowingComponent>();

	if(!flexSimulationActor)
		return;

	flexComponent = flexSimulationActor->FindComponentByClass<UFlexElements>();

	// tick after the RGC
	AddTickPrerequisiteComponent( regionGrowingComponent );

	_initTools();
}

// Called every frame
void AVRSketchyPawn::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );

	if(!_didInit)
	{
		selectionTool->postInit();

		_didInit = true;
	}

	if(_wantsLeftSpiderMan)
		spiderManLeftUpdate();

	if(_wantsRightSpiderMan)
		spiderManRightUpdate();

	if( _rightTouchActive )
		rightController_touchUpdated();

	if( _leftTouchActive ) 
		leftController_touchUpdated();

	if(_currentTool)
	{
		_currentTool->tick( DeltaTime );
	}

	if(_simulating && flexComponent)
	{
		flexComponent->updateSphereWorldSpace( rightInteractionPoint->GetComponentLocation() );
	}
}

// Called to bind functionality to input
void AVRSketchyPawn::SetupPlayerInputComponent(class UInputComponent* InputComponent)
{
	Super::SetupPlayerInputComponent(InputComponent);

	InputComponent->BindKey( EKeys::MotionController_Left_FaceButton1, IE_Released, this, &AVRSketchyPawn::leftController_upFace );
	InputComponent->BindKey( EKeys::MotionController_Left_FaceButton3, IE_Released, this, &AVRSketchyPawn::leftController_downFace );
	InputComponent->BindKey( EKeys::MotionController_Left_FaceButton4, IE_Released, this, &AVRSketchyPawn::leftController_leftFace );
	InputComponent->BindKey( EKeys::MotionController_Left_FaceButton2, IE_Released, this, &AVRSketchyPawn::leftController_rightFace );

	InputComponent->BindKey( EKeys::MotionController_Right_FaceButton1, IE_Released, this, &AVRSketchyPawn::rightController_upFace );
	InputComponent->BindKey( EKeys::MotionController_Right_FaceButton3, IE_Released, this, &AVRSketchyPawn::rightController_downFace );
	InputComponent->BindKey( EKeys::MotionController_Right_FaceButton4, IE_Released, this, &AVRSketchyPawn::rightController_leftFace );
	InputComponent->BindKey( EKeys::MotionController_Right_FaceButton2, IE_Released, this, &AVRSketchyPawn::rightController_rightFace );

	InputComponent->BindKey( EKeys::Steam_Touch_0, IE_Pressed, this, &AVRSketchyPawn::leftController_touchStart );
	InputComponent->BindKey( EKeys::Steam_Touch_0, IE_Released, this, &AVRSketchyPawn::leftController_touchEnd );

	InputComponent->BindKey( EKeys::Steam_Touch_1, IE_Pressed, this, &AVRSketchyPawn::rightController_touchStart );
	InputComponent->BindKey( EKeys::Steam_Touch_1, IE_Released, this, &AVRSketchyPawn::rightController_touchEnd );

	InputComponent->BindAxisKey( EKeys::MotionController_Left_Thumbstick_X );
	InputComponent->BindAxisKey( EKeys::MotionController_Left_Thumbstick_Y );

	InputComponent->BindAxisKey( EKeys::MotionController_Right_Thumbstick_X );
	InputComponent->BindAxisKey( EKeys::MotionController_Right_Thumbstick_Y );

	InputComponent->BindAxisKey( EKeys::MotionController_Left_TriggerAxis, this, &AVRSketchyPawn::leftTrigger );
	InputComponent->BindAxisKey( EKeys::MotionController_Right_TriggerAxis, this, &AVRSketchyPawn::rightTrigger );

	InputComponent->BindKey( EKeys::MotionController_Left_Grip1, IE_Pressed, this, &AVRSketchyPawn::spiderManLeftStart );
	InputComponent->BindKey( EKeys::MotionController_Left_Grip1, IE_Released, this, &AVRSketchyPawn::spiderManLeftEnd );

	InputComponent->BindKey( EKeys::MotionController_Right_Grip1, IE_Pressed, this, &AVRSketchyPawn::spiderManRightStart );
	InputComponent->BindKey( EKeys::MotionController_Right_Grip1, IE_Released, this, &AVRSketchyPawn::spiderManRightEnd );

	InputComponent->BindKey( EKeys::MotionController_Left_Shoulder, IE_Released, this, &AVRSketchyPawn::takeSnapshotAndHighReshShot );
	InputComponent->BindKey( EKeys::MotionController_Right_Shoulder, IE_Released, this, &AVRSketchyPawn::takeSnapshotAndHighReshShot );
}



void AVRSketchyPawn::_initTools()
{
	auto regionGrowingRoot = regionGrowingActor->GetRootComponent();

	generativeBrushTool->widgetComponent = rightPadWidget;
	generativeBrushTool->selectionPointWidgetComponent = rightSelectionPointWidget;

	selectionTool->widgetComponent = rightPadWidget;
	selectionTool->selectionPointWidgetComponent = rightSelectionPointWidget;

	generativeBrushTool->init( leftInteractionPoint, rightInteractionPoint, regionGrowingRoot, regionGrowingComponent );
	
	selectionTool->init( leftInteractionPoint, rightInteractionPoint, regionGrowingRoot, regionGrowingComponent );

	digTool->init( leftInteractionPoint, rightInteractionPoint, regionGrowingRoot, regionGrowingComponent );

	_currentTool = generativeBrushTool;

	_currentTool->gainControl();
}

void AVRSketchyPawn::takeSnapshotAndHighReshShot()
{
	takeHighReshShot();
	takeSimulationSnapshot();
}

void AVRSketchyPawn::takeHighReshShot()
{
	APlayerController * player = UGameplayStatics::GetPlayerController( GetWorld(), 0 );

	if(player)
	{
		player->ConsoleCommand( TEXT( "HighResShot 2" ), true );
	}
}

void AVRSketchyPawn::takeSimulationSnapshot()
{
	ASimulationSnapshotActor * ismcSnapshot = regionGrowingComponent->createSimulationSnapshot();

	FString baseName = regionGrowingActor->GetActorLabel();

	FString name = FString::Printf( TEXT( "%s_snapshot_%d" ), *baseName, _simulationSnapShotCount );
	
	ismcSnapshot->SetActorLabel( name );

	ismcSnapshot->GetRootComponent()->SetVisibility( false, true );
	ismcSnapshot->SetActorEnableCollision( false );

	_simulationSnapShotCount++;
}

// ------------------------------------------------------------
// Left touch
// ------------------------------------------------------------

FVector2D AVRSketchyPawn::_getRightTouchPoint()
{
	FVector2D p;

	p.X = InputComponent->GetAxisKeyValue( EKeys::MotionController_Right_Thumbstick_X );
	p.Y = InputComponent->GetAxisKeyValue( EKeys::MotionController_Right_Thumbstick_Y );

	return p;
}

void AVRSketchyPawn::rightController_touchStart()
{
	_rightTouchActive = true;

	FVector2D p = _getRightTouchPoint();

	_currentTool->rightTouchStart( p );
}

void AVRSketchyPawn::rightController_touchUpdated()
{
	if(!_rightTouchActive)
		return;
	
	FVector2D p = _getRightTouchPoint();

	_currentTool->rightTouchUpdated( p );
}

void AVRSketchyPawn::rightController_touchEnd()
{
	_rightTouchActive = false;

	FVector2D p = _getRightTouchPoint();

	_currentTool->rightTouchEnd();
}

// ------------------------------------------------------------
// Right touch
// ------------------------------------------------------------

FVector2D AVRSketchyPawn::_getLeftTouchPoint()
{
	FVector2D p;

	p.X = InputComponent->GetAxisKeyValue( EKeys::MotionController_Left_Thumbstick_X );
	p.Y = InputComponent->GetAxisKeyValue( EKeys::MotionController_Left_Thumbstick_Y );

	return p;
}

void AVRSketchyPawn::leftController_touchStart()
{
	_leftTouchActive = true;
}

void AVRSketchyPawn::leftController_touchUpdated()
{

}

void AVRSketchyPawn::leftController_touchEnd()
{
	_leftTouchActive = false;
}

// ------------------------------------------------------------

void AVRSketchyPawn::leftController_upFace()
{

}

void AVRSketchyPawn::leftController_downFace()
{
	if(!_currentTool)
		return;

	_currentTool->faceDown_released();
}

void AVRSketchyPawn::leftController_leftFace()
{
	toggleSimulation();
}

void AVRSketchyPawn::leftController_rightFace()
{
	if(!_currentTool)
		return;

	_currentTool->faceRight_released();
}


void AVRSketchyPawn::rightController_upFace()
{
	if(!_currentTool)
		return;

	_currentTool->faceUp_released(rightInteractionPoint);
}

void AVRSketchyPawn::rightController_downFace()
{
	if(!_currentTool)
		return;

	_currentTool->faceDown_released();
}

void AVRSketchyPawn::rightController_leftFace()
{
	toggleSimulation();
}

void AVRSketchyPawn::rightController_rightFace()
{
	if(!_currentTool)
		return;

	_currentTool->faceRight_released();
}

void AVRSketchyPawn::toggleSimulation()
{
	if(_simulating)
	{
		_endSimulation();
		_simulating = false;
	}
	else
	{
		_startSimulation();
		_simulating = true;
	}
}

void AVRSketchyPawn::_startSimulation()
{
	if(!flexComponent || !regionGrowingComponent)
		return;

	regionGrowingComponent->pauseSynthesis = true;
	regionGrowingComponent->SetActive( false ); 
	regionGrowingComponent->setElementVisibility( false );

	FTransform transform = flexSimulationActor->GetRootComponent()->GetComponentTransform();

	FVector min = regionGrowingComponent->generationLimitsMin;
	FVector max = regionGrowingComponent->generationLimitsMax;

	// bounds box in local space of the region-growing actor
	FBox bounds = FBox( transform.InverseTransformPosition(min), transform.InverseTransformPosition(max) );

	flexComponent->setInstanceManagerBounds( bounds );
	auto exportedDomain = regionGrowingComponent->exportOutputDomain();
	flexComponent->loadExportedDomainInfo( exportedDomain );

	flexComponent->play();
}

void AVRSketchyPawn::_endSimulation()
{
	if(!flexComponent || !regionGrowingComponent)
		return;

	auto elementDomain = flexComponent->exportElementDomain();
	regionGrowingComponent->loadElementDomain(elementDomain);

	regionGrowingComponent->pauseSynthesis = false;
	regionGrowingComponent->SetActive( true );
	regionGrowingComponent->setElementVisibility( true );

	flexComponent->pause();
	flexComponent->clear();
}

void AVRSketchyPawn::leftTrigger( float value )
{
	if(!regionGrowingComponent || !_currentTool)
		return;

	_currentTool->setLeftTriggerValue( value );
	_lastLeftTriggerValue = value;

	if(_leftTriggerDown && value <= 0.1f)
	{
		_leftTriggerDown = false;
		_currentTool->leftEnd();
	}

	if(!_leftTriggerDown && value > 0.1f)
	{
		_leftTriggerDown = true;
		_currentTool->leftStart();
	}
}

void AVRSketchyPawn::rightTrigger( float value )
{
	if(!regionGrowingComponent || !_currentTool)
		return;

	_currentTool->setRightTriggerValue( value );
	_lastRightTriggerValue = value;

	if(_rightTriggerDown && value <= 0.1f)
	{
		_rightTriggerDown = false;
		_currentTool->rightEnd();
	}

	if(!_rightTriggerDown && value > 0.1f)
	{
		_rightTriggerDown = true;
		_currentTool->rightStart();
	}
}

void AVRSketchyPawn::_startCurrentToolLeftTrigger()
{
	_currentTool->setLeftTriggerValue( _lastLeftTriggerValue );

	if(_leftTriggerDown && _lastLeftTriggerValue > 0.1f)
	{
		_currentTool->leftStart();
	}
}

void AVRSketchyPawn::_startCurrentToolRightTrigger()
{
	_currentTool->setRightTriggerValue( _lastRightTriggerValue );

	if(_rightTriggerDown && _lastRightTriggerValue > 0.1f)
	{
		_currentTool->rightStart();
	}
}

void AVRSketchyPawn::spiderManLeftStart()
{
	_wantsLeftSpiderMan = true;

	_lastLeft = leftController->GetComponentLocation();
}

void AVRSketchyPawn::spiderManLeftEnd()
{
	_wantsLeftSpiderMan = false;
}

void AVRSketchyPawn::spiderManLeftUpdate()
{
	FVector left = leftController->GetComponentLocation();

	FVector delta = (left - _lastLeft);

	this->AddActorWorldOffset( -delta, false, nullptr, ETeleportType::TeleportPhysics );

	_lastLeft = leftController->GetComponentLocation();
}

void AVRSketchyPawn::spiderManRightStart()
{
	_wantsRightSpiderMan = true;

	_lastRight = rightController->GetComponentLocation();
}

void AVRSketchyPawn::spiderManRightEnd()
{
	_wantsRightSpiderMan = false;
}

void AVRSketchyPawn::spiderManRightUpdate()
{
	FVector right = rightController->GetComponentLocation();

	FVector delta = (right - _lastRight);

	this->AddActorWorldOffset( -delta, false, nullptr, ETeleportType::TeleportPhysics );

	_lastRight = rightController->GetComponentLocation();
}

void AVRSketchyPawn::setCurrentTool( UTool * newCurrentTool )
{
	if(_currentTool == newCurrentTool)
		return;

	if(_currentTool)
		_currentTool->releaseControl();

	_currentTool = newCurrentTool;

	if(_currentTool)
	{
		_currentTool->gainControl();

		// fire off previous trigger values
		_startCurrentToolLeftTrigger();
		_startCurrentToolRightTrigger();

		if(_rightTouchActive)
			rightController_touchStart();
	}
}

void AVRSketchyPawn::setSimulating( bool simulating )
{
	if(simulating == _simulating)
		return;

	toggleSimulation();
}
