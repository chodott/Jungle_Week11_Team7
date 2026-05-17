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

	auto ValidateVertexRange = [&](uint32 VertexStart, uint32 VertexEnd, const FMatrix& MeshBindGlobal)
	{
		VertexEnd = (std::min)(VertexEnd, static_cast<uint32>(Mesh.Vertices.size()));
		for (uint32 VertexIndex = VertexStart; VertexIndex < VertexEnd; ++VertexIndex)
		{
			const FVertexPNCTBW& Src = Mesh.Vertices[VertexIndex];
			FVector SkinnedPos(0.0f, 0.0f, 0.0f);
			float TotalWeight = 0.0f;

			for (int32 InfluenceIndex = 0; InfluenceIndex < 4; ++InfluenceIndex)
			{
				const int32 BoneIndex = Src.BoneIndices[InfluenceIndex];
				const float Weight = Src.BoneWeights[InfluenceIndex];
				if (BoneIndex < 0 || BoneIndex >= BoneCount || !std::isfinite(Weight) || Weight <= 0.0f)
				{
					continue;
				}

				const FBone& Bone = Mesh.Bones[BoneIndex];
				const FMatrix SkinningMatrix = MeshBindGlobal * Bone.InverseBindPoseMatrix * Bone.GlobalMatrix;
				const FVector WeightedPos = SkinningMatrix.TransformPositionWithW(Src.Position) * Weight;
				if (!std::isfinite(WeightedPos.X) || !std::isfinite(WeightedPos.Y) || !std::isfinite(WeightedPos.Z))
				{
					continue;
				}

				SkinnedPos += WeightedPos;
				TotalWeight += Weight;
			}

			if (TotalWeight <= 1.0e-6f)
			{
				continue;
			}

			const FVector ReferencePos = MeshBindGlobal.TransformPositionWithW(Src.Position);
			const float Error = (SkinnedPos - ReferencePos).Length();
			if (std::isfinite(Error))
			{
				MaxError = (std::max)(MaxError, Error);
			}
		}
	};

	if (!Mesh.MeshRanges.empty())
	{
		for (const FSkeletalMeshRange& Range : Mesh.MeshRanges)
		{
			ValidateVertexRange(Range.VertexStart, Range.VertexEnd, Range.MeshBindGlobal);
		}
	}
	else
	{
		ValidateVertexRange(0, static_cast<uint32>(Mesh.Vertices.size()), FMatrix::Identity);
	}

	return MaxError;
}
