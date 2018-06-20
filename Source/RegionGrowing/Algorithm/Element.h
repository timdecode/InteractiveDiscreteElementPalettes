//
//  Element.h
//  RegionGrowing
//
//  Created by Timothy Davison on 2015-06-25.
//  Copyright (c) 2015 Timothy Davison. All rights reserved.
//

#pragma once

#include "Eigen/Dense"
#include "TimStructBox.h"
#include "NeighbourhoodParameters.h"
#include <stdint.h>

#include "Element.generated.h"





USTRUCT(BlueprintType)
struct FElement
{
	GENERATED_BODY()
public:
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
    
	UPROPERTY( EditAnywhere ) float radius = 70.0f;
	UPROPERTY( EditAnywhere ) int16 entityType = 0; // entity type, cached from the parent entity for fast lookups
	UPROPERTY( EditAnywhere ) int16 type = 0;// element type
	UPROPERTY( EditAnywhere ) bool generative = true;
	UPROPERTY( EditAnywhere ) int32 entityIndex = 0; // index of the parent entity
	UPROPERTY( EditAnywhere ) int32 faceIndex = 0; // if surface walking, the index of the face that the element is on

	UPROPERTY( EditAnywhere ) FNeighbourhoodParameters generationParameters;
	UPROPERTY( EditAnywhere ) FNeighbourhoodParameters optimizationParameters;

	UPROPERTY( EditAnywhere ) float minAssignmentDistance = 1.0f;
	UPROPERTY( EditAnywhere ) float freespaceRadius = 1.0f;

	UPROPERTY( EditAnywhere ) float generationInnerRadius = 1.0f;

	UPROPERTY( EditAnywhere ) float hackScale = 1.0f;

	UPROPERTY( EditAnywhere ) TArray<FTimStructBox> graphObjects;
    
    //bool operator==(const Element& other) const
    //{
    //    return position == other.position &&
    //           rotation == other.rotation &&
    //           radius == other.radius &&
    //           entityType == other.entityType &&
    //           type == other.type &&
    //           generative == other.generative &&
    //           entityIndex == other.entityIndex &&
    //           faceIndex == other.faceIndex &&
    //           gradient == other.gradient &&
    //           dotProduct == other.dotProduct;
    //}
};
