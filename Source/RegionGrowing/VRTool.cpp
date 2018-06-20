//
//  Created by Timothy Davison on 2018-03-06, based on my code for Ship Editor.
//  Copyright (c) Timothy Davison. All rights reserved.
//

#include "RegionGrowing.h"

#include "MotionControllerComponent.h"
#include "WidgetComponent.h"

#include "RegionGrowingComponent.h"

void UTool::_hideWidgets()
{
	if(widgetComponent)
	{
		widgetComponent->SetWidgetClass( nullptr );
		widgetComponent->SetVisibility( false );
	}

	if(selectionPointWidgetComponent)
	{
		selectionPointWidgetComponent->SetWidgetClass( nullptr );
		selectionPointWidgetComponent->SetVisibility( false );
	}
}

void UTool::_loadWidgets()
{
	if(widgetComponent)
	{
		widgetComponent->SetWidgetClass( getWidgetClass() );
		widgetComponent->SetVisibility( true );
	}

	if(selectionPointWidgetComponent)
	{
		selectionPointWidgetComponent->SetWidgetClass( getSelectionWidgetClass() );
		selectionPointWidgetComponent->SetVisibility( true );
	}
}
