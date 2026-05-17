#pragma once

#include "Mesh/Fbx/FbxImportTypes.h"
#include "Render/Types/VertexTypes.h"
#include "Mesh/Fbx/FbxMorphTargetImporter.h"

#include <fbxsdk.h>

struct FFbxImportContext
{
	FString                SourcePath;
	FSkeletalImportSummary Summary;

	TArray<FbxNode*> AllNodes;
	TArray<FbxNode*> MeshNodes;

	TArray<FFbxImportedMaterialInfo> Materials;
	TMap<FbxSurfaceMaterial*, int32> MaterialToSlotIndex;

	TArray<FBone> Bones;
	TMap<FbxNode*, int32> BoneNodeToIndex;
	FReferenceSkeleton ReferenceSkeleton;

	TArray<FVertexPNCTBW> SkeletalVertices;
	TArray<uint32> SkeletalIndices;
	TArray<FSkeletalMeshSection> SkeletalSections;
	TArray<FSkeletalMeshRange> SkeletalMeshRanges;

	TArray<FVector> TangentSums;
	TArray<FVector> BitangentSums;

	TArray<TArray<FFbxImportedMorphSourceVertex>> MorphSourcesByLOD;
	TArray<FMorphTarget>                          MorphTargets;

	TArray<UAnimSequence*> AnimSequences;

	void AddWarning(ESkeletalImportWarningType Type, const FString& Message)
	{
		FSkeletalImportWarning Warning;
		Warning.Type    = Type;
		Warning.Message = Message;
		Summary.Warnings.push_back(Warning);
	}

	bool HasWarningType(ESkeletalImportWarningType Type) const
	{
		for (const FSkeletalImportWarning& Warning : Summary.Warnings)
		{
			if (Warning.Type == Type) return true;
		}
		return false;
	}

	void AddWarningOnce(ESkeletalImportWarningType Type, const FString& Message)
	{
		if (!HasWarningType(Type)) AddWarning(Type, Message);
	}
};
