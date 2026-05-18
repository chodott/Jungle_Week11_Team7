#include "Mesh/Fbx/FbxImportValidation.h"

#include <algorithm>
#include <cmath>

float FFbxImportValidation::ValidateBindPoseSkinningError(const FSkeletalMesh& Mesh)
{
	if (Mesh.Vertices.empty() || Mesh.Bones.empty())
	{
		return 0.0f;
	}

	float MaxError = 0.0f;
	const int32 BoneCount = static_cast<int32>(Mesh.Bones.size());

	TArray<FMatrix> RuntimeBindGlobals;
	RuntimeBindGlobals.resize(BoneCount, FMatrix::Identity);
	for (int32 BoneIndex = 0; BoneIndex < BoneCount; ++BoneIndex)
	{
		const FBone& Bone             = Mesh.Bones[BoneIndex];
		RuntimeBindGlobals[BoneIndex] = (Bone.ParentIndex >= 0 && Bone.ParentIndex < BoneCount)
		? Bone.LocalMatrix * RuntimeBindGlobals[Bone.ParentIndex] : Bone.LocalMatrix;
	}

	for (const FVertexPNCTBW& Src : Mesh.Vertices)
	{
		FVector SkinnedPos(0.0f, 0.0f, 0.0f);
		float   TotalWeight = 0.0f;

		for (int32 InfluenceIndex = 0; InfluenceIndex < 4; ++InfluenceIndex)
		{
			const int32 BoneIndex = Src.BoneIndices[InfluenceIndex];
			const float Weight    = Src.BoneWeights[InfluenceIndex];
			if (BoneIndex < 0 || BoneIndex >= BoneCount || !std::isfinite(Weight) || Weight <= 0.0f)
			{
				continue;
			}

			const FMatrix SkinningMatrix = Mesh.Bones[BoneIndex].InverseBindPoseMatrix * RuntimeBindGlobals[BoneIndex];
			const FVector WeightedPos    = SkinningMatrix.TransformPositionWithW(Src.Position) * Weight;
			if (!std::isfinite(WeightedPos.X) || !std::isfinite(WeightedPos.Y) || !std::isfinite(WeightedPos.Z))
			{
				continue;
			}

			SkinnedPos  += WeightedPos;
			TotalWeight += Weight;
		}

		if (TotalWeight <= 1.0e-6f)
		{
			continue;
		}

		const float Error = (SkinnedPos - Src.Position).Length();
		if (std::isfinite(Error))
		{
			MaxError = (std::max)(MaxError, Error);
		}
	}

	return MaxError;
}
