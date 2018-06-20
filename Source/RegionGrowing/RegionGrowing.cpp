// Fill out your copyright notice in the Description page of Project Settings.

#include "RegionGrowing.h"


#ifdef __APPLE__
#include <dlfcn.h>
#include <cstring>
#include <string>
#endif

#if WITH_EDITOR
#include "EditorModuleInlined/RegionGrowingComponentEditor.h" 
#include "EditorModuleInlined/TCODSEditorMode.h"
#include "EditorModuleInlined/FVolumeComponentDetailsCustomization.h"
#include "EditorModuleInlined/TimStructBoxDetails.h"
#endif

IMPLEMENT_PRIMARY_GAME_MODULE( FRegionGrowing, RegionGrowing, "RegionGrowing" );

static void anAdressToFindTheDll() {}

void FRegionGrowing::StartupModule()
{
#ifdef __APPLE__
    Dl_info info;
    if( dladdr((void*)&anAdressToFindTheDll, &info) )
    {
        
        const size_t cSize = strlen(info.dli_fname);
        std::wstring ws(cSize, L'#');
        mbstowcs(&ws[0], info.dli_fname, cSize);
        FString dylibName(cSize, &ws[0]);
        
        UE_LOG(LogTemp, Warning, TEXT("FRegionGrowing::StartModule() loaded from %s"), *dylibName);
    }
#endif

#if WITH_EDITOR
    registerCustomClassLayout("RegionGrowingComponent", FOnGetDetailCustomizationInstance::CreateStatic(&URegionGrowingComponentEditor::MakeInstance));
	registerCustomClassLayout( "VolumeComponent", FOnGetDetailCustomizationInstance::CreateStatic( &FVolumeComponentDetailsCustomization::MakeInstance ) );


    // Unreal doesn't want to Hot Reload classes correctly. We often seem to end up with two different versions of a library loaded in memory.
    FPropertyEditorModule& propertyModule = FModuleManager::GetModuleChecked<FPropertyEditorModule>("PropertyEditor");

	structBoxName = FTimStructBox::StaticStruct()->GetFName();

	propertyModule.RegisterCustomPropertyTypeLayout( structBoxName, FOnGetPropertyTypeCustomizationInstance::CreateStatic( &FTimStructBoxCustomization::MakeInstance ) );

	propertyModule.NotifyCustomizationModuleChanged();
    
	

	FEditorModeRegistry::Get().RegisterMode<FTCODSEditorMode>( FTCODSEditorMode::EM_Geometry,
		NSLOCTEXT( "EditorModes", "TrivialConnectionsMode", "Trivial Connections Mesh Editing" ),
		FSlateIcon( FEditorStyle::GetStyleSetName(), "LevelEditor.BspMode", "LevelEditor.BspMode.Small" ),
		true, 500
                                                              );



#endif
}

void FRegionGrowing::ShutdownModule()
{
#if WITH_EDITOR
    FPropertyEditorModule& propertyModule = FModuleManager::GetModuleChecked<FPropertyEditorModule>("PropertyEditor");
    
    for( auto name : registerClassNames )
        propertyModule.UnregisterCustomClassLayout(name);
    
	propertyModule.UnregisterCustomPropertyTypeLayout( structBoxName );

    propertyModule.NotifyCustomizationModuleChanged();
    
    FEditorModeRegistry::Get().UnregisterMode(FTCODSEditorMode::EM_Geometry);
#endif
}

#if WITH_EDITOR
void FRegionGrowing::registerCustomClassLayout(FName className, FOnGetDetailCustomizationInstance detailLayoutDelegate)
{
    registerClassNames.Add( className );
    
    static FName PropertyEditor("PropertyEditor");
    FPropertyEditorModule& propertyModule = FModuleManager::GetModuleChecked<FPropertyEditorModule>(PropertyEditor);
    propertyModule.RegisterCustomClassLayout( className, detailLayoutDelegate );
}
#endif
