// Fill out your copyright notice in the Description page of Project Settings.

using UnrealBuildTool;
using System.Collections.Generic;

public class RegionGrowingEditorTarget : TargetRules
{
	public RegionGrowingEditorTarget(TargetInfo Target)
	{
		Type = TargetType.Editor;
        
        ExtraModuleNames.Add("RegionGrowing");
	}
}
