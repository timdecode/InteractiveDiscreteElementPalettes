// Fill out your copyright notice in the Description page of Project Settings.

using UnrealBuildTool;
using System.Collections.Generic;

public class RegionGrowingTarget : TargetRules
{
	public RegionGrowingTarget(TargetInfo Target)
	{
		Type = TargetType.Game;
        
        ExtraModuleNames.Add("RegionGrowing");
	}
}
