//
//  VRTool.cpp
//  RegionGrowing
//
//  Created by Timothy Davison on 2018-03-06, based on my code for Ship Editor.
//  Copyright (c) Timothy Davison. All rights reserved.
//

#pragma once

#include "GameFramework/Pawn.h"
#include "VRTool.generated.h"

class UMotionControllerComponent;
class URegionGrowingComponent;
class UWidgetComponent;

/**
*
*/
UCLASS( BlueprintType )
class UTool : public UObject
{
	GENERATED_BODY()

public:
	void init
	( 
		UPrimitiveComponent * leftSelectionPoint,
		UPrimitiveComponent * rightSelectionPoint,
		USceneComponent * targetComponent_in 
	)
	{
		targetComponent = targetComponent_in;

		_leftSelectionPoint = leftSelectionPoint;
		_rightSelectionPoint = rightSelectionPoint;

		_selectionA = leftSelectionPoint;
		_selectionB = rightSelectionPoint;

		_handMode = HandMode::None;

		_updateLast();
	}

	virtual void oneHandStart( UPrimitiveComponent * hand ) {}
	virtual void oneHandEnd( UPrimitiveComponent * hand ) {}

	virtual void twoHandStart( UPrimitiveComponent * handA, UPrimitiveComponent * handB ) {}
	virtual void twoHandEnd( UPrimitiveComponent * handA, UPrimitiveComponent * handB ) {}

	// Called when the tool is released. Any action should be aborted.
	virtual void loseFocus() {}

	// Called when the tool gains control.
	virtual void focused() {}

	virtual void tickOneHand( float dt, UPrimitiveComponent * hand, FTransform lastTransform ) {}

	virtual void tickTwoHand
	(
		float dt,
		UPrimitiveComponent * handA,
		UPrimitiveComponent * handB,
		FTransform transformA,
		FTransform transformB
	) {}

	// Face Down
	// -----------------------------
	virtual void faceDown_released()
	{

	}

	virtual void faceDown_touchStart()
	{

	}

	virtual void faceDown_touchEnd()
	{

	}

	virtual void faceDown_pressed()
	{

	}

	// Face Up
	// -----------------------------
	virtual void faceUp_released(USceneComponent * interactionPoint = nullptr)
	{

	}

	virtual void faceUp_touchStart()
	{

	}

	virtual void faceUp_touchEnd()
	{

	}

	virtual void faceUp_pressed()
	{

	}

	// Face Right
	// -----------------------------
	virtual void faceRight_released()
	{

	}



	void tick( float dt )
	{
		if(_handMode == HandMode::OneHand)
			tickOneHand( dt, _selectionA, _lastA );
		else if(_handMode == HandMode::TwoHand)
			tickTwoHand( dt, _selectionA, _selectionB, _lastA, _lastB );

		_updateLast();
	}

	void rightStart()
	{
		if(_handMode == HandMode::None)
		{
			_selectionA = _rightSelectionPoint;
			_selectionB = _leftSelectionPoint;

			_updateLast();

			_handMode = HandMode::OneHand;

			oneHandStart( _selectionA );
		}
		else if(_handMode == HandMode::OneHand)
			_twoHandStart();
	}

	void leftStart()
	{
		if(_handMode == HandMode::None)
		{

			_selectionA = _leftSelectionPoint;
			_selectionB = _rightSelectionPoint;

			_updateLast();

			_handMode = HandMode::OneHand;

			oneHandStart( _selectionA );
		}
		else if(_handMode == HandMode::OneHand)
			_twoHandStart();
	}

	void rightEnd()
	{
		if(_handMode == HandMode::OneHand)
		{
			_handMode = HandMode::None;

			oneHandEnd( _selectionA );
		}
		else if(_handMode == HandMode::TwoHand)
		{
			_handMode = HandMode::OneHand;

			_selectionA = _leftSelectionPoint;
			_selectionB = _rightSelectionPoint;

			twoHandEnd( _selectionA, _selectionB );
		}

		_updateLast();
	}

	void leftEnd()
	{
		if(_handMode == HandMode::OneHand)
		{
			_handMode = HandMode::None;

			oneHandEnd( _selectionA );
		}
		else if(_handMode == HandMode::TwoHand)
		{
			_handMode = HandMode::OneHand;

			_selectionA = _rightSelectionPoint;
			_selectionB = _leftSelectionPoint;

			twoHandEnd( _selectionA, _selectionB );
		}

		_updateLast();
	}

	void setLeftTriggerValue( float value )
	{
		if(_selectionA == _leftSelectionPoint)
			_triggerA = value;
		else if(_selectionB == _leftSelectionPoint)
			_triggerB = value;

		if(_handMode == HandMode::OneHand)
			_triggerB = 0.0f;
		else if(_handMode == HandMode::None)
		{
			_triggerA = 0.0f;
			_triggerB = 0.0f;
		}
	}

	void setRightTriggerValue( float value )
	{
		if(_selectionA == _rightSelectionPoint)
			_triggerA = value;
		else if(_selectionB == _rightSelectionPoint)
			_triggerB = value;

		if(_handMode == HandMode::OneHand)
			_triggerB = 0.0f;
		else if(_handMode == HandMode::None)
		{
			_triggerA = 0.0f;
			_triggerB = 0.0f;
		}
	}

	void rightTouchStart( FVector2D p )
	{
		_rightTouchActive = true;

		if( p.Y >= 0.0f )
		{
			_rightTouchStartDirection = TouchDirection::Down;
			faceDown_touchStart();
		}
		else
		{
			_rightTouchStartDirection = TouchDirection::Up;
			faceUp_touchStart();
		}
	}

	void rightTouchUpdated( FVector2D p )
	{

	}

	void rightTouchEnd()
	{
		if( _rightTouchStartDirection == TouchDirection::Down )
			faceDown_touchEnd();
		else if( _rightTouchStartDirection == TouchDirection::Up )
			faceUp_touchEnd();

		_rightTouchActive = false;
	}

	float selectionATriggerValue()
	{
		return _triggerA;
	}

	float selectionBTriggerValue()
	{
		return _triggerB;
	}

	void gainControl()
	{
		_loadWidgets();
		
		focused();
	}

	void releaseControl()
	{
		loseFocus();

		_hideWidgets();

		_handMode = HandMode::None;

		_selectionA = _leftSelectionPoint;
		_selectionB = _rightSelectionPoint;

		_lastA = FTransform::Identity;
		_lastB = FTransform::Identity;

		_rightTouchActive = false;
	}

	// Call this if both triggers should be considered released. It will trigger a oneHandEnd, or a twoHandEnd followed by a oneHandEnd.
	void endAction()
	{
		if(_handMode == HandMode::OneHand)
		{
			_handMode = HandMode::None;

			oneHandEnd( _selectionA );
		}
		else if(_handMode == HandMode::TwoHand)
		{
			_handMode = HandMode::None;

			twoHandEnd( _selectionA, _selectionB );
			oneHandEnd( _selectionA );

			_selectionA = _leftSelectionPoint;
			_selectionB = _rightSelectionPoint;
		}
	}

protected:
	void _twoHandStart()
	{
		_updateLast();

		_handMode = HandMode::TwoHand;

		twoHandStart( _selectionA, _selectionB );
	}

	void _updateLast()
	{
		_lastA = graphLocalTransform( _selectionA );
		_lastB = graphLocalTransform( _selectionB );
	}

	FTransform graphLocalTransform( USceneComponent * component )
	{
		FTransform inverseGraph = targetComponent->GetOwner()->GetRootComponent()->GetComponentTransform().Inverse();

		return component->GetComponentTransform() * inverseGraph;
	}

	void _hideWidgets();

	void _loadWidgets();

public:
	USceneComponent * targetComponent = nullptr;

	UWidgetComponent * widgetComponent = nullptr;
	UWidgetComponent * selectionPointWidgetComponent = nullptr;

	virtual TSubclassOf<class UUserWidget> getWidgetClass() { return nullptr; }
	virtual TSubclassOf<class UUserWidget> getSelectionWidgetClass() { return nullptr; }

	enum class HandMode
	{
		None,
		OneHand,
		TwoHand
	};

protected:

	HandMode _handMode = HandMode::None;

	UPrimitiveComponent * _leftSelectionPoint;
	UPrimitiveComponent * _rightSelectionPoint;

	UPrimitiveComponent * _selectionA;
	UPrimitiveComponent * _selectionB;

	FTransform _lastA;
	FTransform _lastB;

	float _triggerA = 0.0f;
	float _triggerB = 0.0f;

	bool _rightTouchActive = false;
	enum class TouchDirection
	{
		Up,
		Down
	};

	TouchDirection _rightTouchStartDirection;
};


/**
*
*/
UCLASS( BlueprintType )
class UWidgetTool : public UObject
{
	GENERATED_BODY()

public:

};