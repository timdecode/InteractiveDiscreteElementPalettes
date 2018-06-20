// Copyright 2018, Timothy Davison. All Rights Reserved.

#include "RegionGrowing.h"

#include "TimStructBox.h"

void FTimStructBox::Destroy( UScriptStruct* ActualStruct )
{
	if(ActualStruct && structMemory)
	{
		ActualStruct->DestroyStruct( structMemory );
	}

	if(structMemory)
	{
		FMemory::Free( structMemory );
		structMemory = nullptr;
	}
}

void FTimStructBox::initStruct()
{
	if(!scriptStruct)
		return;

	Create( scriptStruct );
}

FTimStructBox::~FTimStructBox()
{
	ensure( scriptStruct || !structMemory );
	Destroy( scriptStruct );
}

FTimStructBox& FTimStructBox::operator=( const FTimStructBox& Other )
{
	if(this != &Other)
	{
		Destroy( scriptStruct );

		scriptStruct = Other.scriptStruct;

		if(Other.IsValid())
		{
			Create( scriptStruct );
			scriptStruct->CopyScriptStruct( structMemory, Other.structMemory );
		}
	}

	return *this;
}

void FTimStructBox::Create( UScriptStruct* ActualStruct )
{
	check( nullptr == structMemory );
	structMemory = (uint8*)FMemory::Malloc( ActualStruct->GetStructureSize() );
	ActualStruct->InitializeStruct( structMemory );
}

FTimStructBox::FTimStructBox( const FTimStructBox& Other )
{
	structMemory = nullptr;
	scriptStruct = Other.scriptStruct;

	if(Other.IsValid())
	{
		Create( scriptStruct );
		scriptStruct->CopyScriptStruct( structMemory, Other.structMemory );
	}
}

bool FTimStructBox::Serialize( FArchive& Ar )
{
	auto OldStruct = scriptStruct;
	Ar << scriptStruct;
	bool bValidBox = IsValid();
	Ar << bValidBox;

	if(Ar.IsLoading())
	{
		if(OldStruct != scriptStruct)
		{
			Destroy( OldStruct );
		}
		if(scriptStruct && !structMemory && bValidBox)
		{
			Create( scriptStruct );
		}
	}

	ensure( bValidBox || !IsValid() );
	if(IsValid() && bValidBox)
	{
		scriptStruct->SerializeItem( Ar, structMemory, nullptr );
	}

	return true;
}

bool FTimStructBox::Identical( const FTimStructBox* Other, uint32 PortFlags ) const
{
	if(!Other)
	{
		return false;
	}

	if(scriptStruct != Other->scriptStruct)
	{
		return false;
	}

	if(!scriptStruct)
	{
		return true;
	}

	if(!structMemory && !Other->structMemory)
	{
		return true;
	}

	return scriptStruct->CompareScriptStruct( structMemory, Other->structMemory, PortFlags );
}

void FTimStructBox::AddStructReferencedObjects( class FReferenceCollector& Collector ) const
{
	Collector.AddReferencedObject( const_cast<FTimStructBox*>(this)->scriptStruct );
	if(scriptStruct && structMemory && scriptStruct->RefLink)
	{
		scriptStruct->SerializeBin( Collector.GetVerySlowReferenceCollectorArchive(), structMemory );
	}
}
