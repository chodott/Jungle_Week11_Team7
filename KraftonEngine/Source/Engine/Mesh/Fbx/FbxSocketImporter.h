#pragma once

#include "Core/CoreTypes.h"
#include "Math/Matrix.h"
#include "Animation/SkeletonTypes.h"
#include "Mesh/SkeletalMeshAsset.h"

#include <fbxsdk.h>

struct FFbxImportContext;

class FFbxSocketImporter
{
public:
	static void ImportSockets(
		FbxScene*                    Scene,
		const TMap<FbxNode*, int32>& BoneNodeToIndex,
		const FMatrix&               ReferenceMeshBindInverse,
		const FReferenceSkeleton&    ReferenceSkeleton,
		TArray<FSkeletalSocket>&     OutSockets,
		FFbxImportContext&           BuildContext
		);
};
