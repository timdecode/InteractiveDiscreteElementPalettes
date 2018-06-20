// Copyright 2016, Timothy Davison. All rights reserved.

#pragma once

#include "GameFramework/Pawn.h"

#include "VRTool.h"


#include "VRSketchyPawn.generated.h"


class UMotionControllerComponent;
class URegionGrowingComponent;

class USelectionTool;
class UGenerativeBrushTool;
class UDigTool;

UCLASS( DefaultToInstanced )
class REGIONGROWING_API AVRSketchyPawn : public APawn
{
	GENERATED_BODY()

public:
	UPROPERTY( EditAnywhere, BlueprintReadOnly, Category = "ShipEditor" )
	USceneComponent * vrScene;

	UPROPERTY( EditAnywhere, BlueprintReadOnly, Category = "ShipEditor" )
	UCameraComponent * camera;

	UPROPERTY( EditAnywhere, BlueprintReadOnly, Category = "ShipEditor" )
	UMotionControllerComponent * leftController;

	UPROPERTY( EditAnywhere, BlueprintReadOnly, Category = "ShipEditor" )
	UMotionControllerComponent * rightController;

	UPROPERTY( EditAnywhere, BlueprintReadOnly, Category = "ShipEditor" )
	UStaticMeshComponent * rightInteractionPoint;

	UPROPERTY( EditAnywhere, BlueprintReadOnly, Category = "ShipEditor" )
	UStaticMeshComponent * leftInteractionPoint;

	UPROPERTY( EditAnywhere, BlueprintReadOnly, Category = "ShipEditor" )
	UWidgetComponent * rightPadWidget;

	UPROPERTY( EditAnywhere, BlueprintReadOnly, Category = "ShipEditor" )
	UWidgetComponent * rightSelectionPointWidget;

	UPROPERTY( EditInstanceOnly, BlueprintReadWrite, Category = "ShipEditor" )
	AActor * regionGrowingActor = nullptr;

	UPROPERTY( EditInstanceOnly, BlueprintReadWrite, Category = "ShipEditor" )
	AActor * flexSimulationActor = nullptr;

	UPROPERTY( EditAnywhere, BlueprintReadOnly, Instanced, Category = "ShipEditor" )
	UGenerativeBrushTool * generativeBrushTool;

	UPROPERTY( EditAnywhere, BlueprintReadOnly, Instanced, Category = "ShipEditor" )
	USelectionTool * selectionTool;

	UPROPERTY( EditAnywhere, BlueprintReadOnly, Instanced, Category = "ShipEditor" )
	UDigTool * digTool;


public:

	UFUNCTION( BlueprintCallable, Category = Generation )
	bool isSimulating() { return _simulating;  }

	UFUNCTION( BlueprintCallable, Category = Generation )
	void setSimulating( bool simulating );

	UFUNCTION( BlueprintCallable, Category = Generation )
	void leftTrigger( float value );

	UFUNCTION( BlueprintCallable, Category = Generation )
	void rightTrigger( float value );

	UFUNCTION( BlueprintCallable, Category = Generation )
	void toggleSimulation();

	UFUNCTION( BlueprintCallable, Category = Generation )
	void setCurrentTool( UTool * newCurrentTool );

	UFUNCTION( BlueprintCallable, Category = Generation )
	UTool * getCurrentTool() { return _currentTool; }

protected:
	URegionGrowingComponent * regionGrowingComponent;
	UFlexElements * flexComponent;

	bool _didInit = false;

	bool _simulating = false;

	UTool * _currentTool;

	bool _leftTriggerDown = false;
	bool _rightTriggerDown = false;

	float _lastLeftTriggerValue = 0.0f;
	float _lastRightTriggerValue = 0.0f;

	bool _wantsLeftSpiderMan = false;
	bool _wantsRightSpiderMan = false;


	FVector _lastLeft;
	FVector _lastRight;


	int _simulationSnapShotCount = 0;

	bool _leftTouchActive = false;
	bool _rightTouchActive = false;

public:
	// Sets default values for this pawn's properties
	AVRSketchyPawn();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	// Called every frame
	virtual void Tick( float DeltaSeconds ) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* InputComponent) override;

	void spiderManLeftStart();
	void spiderManLeftEnd();
	void spiderManLeftUpdate();

	void spiderManRightStart();
	void spiderManRightEnd();
	void spiderManRightUpdate();

protected:
	void _initTools();

	void takeSnapshotAndHighReshShot();
	void takeHighReshShot();
	void takeSimulationSnapshot();

	void leftController_touchStart();
	void leftController_touchUpdated();
	void leftController_touchEnd();
	FVector2D _getLeftTouchPoint();

	void leftController_upFace();
	void leftController_downFace();
	void leftController_leftFace();
	void leftController_rightFace();

	void rightController_touchStart();
	void rightController_touchUpdated();
	void rightController_touchEnd();
	FVector2D _getRightTouchPoint();

	void rightController_upFace();
	void rightController_downFace();
	void rightController_leftFace();
	void rightController_rightFace();

	void _startSimulation();
	void _endSimulation();

	void _startCurrentToolLeftTrigger();
	void _startCurrentToolRightTrigger();
};
