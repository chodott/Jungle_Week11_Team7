#pragma once

#include "Core/CoreTypes.h"
#include "Render/Types/VertexTypes.h"
#include "Render/Resource/Buffer.h"
#include "Math/Matrix.h"
#include "Serialization/Archive.h"
#include "Materials/Material.h"
#include "Materials/MaterialManager.h"
#include "Mesh/StaticMeshAsset.h"
#include "Mesh/MeshCollisionAsset.h"

#include <algorithm>
#include <memory>

inline void SerializeSkeletalMatrix(FArchive& Ar, FMatrix& Matrix)
{
	Ar.Serialize(Matrix.Data, sizeof(float) * 16);
}

struct FBone
{
	FString Name;
	int32 ParentIndex = -1;

	FMatrix LocalMatrix = FMatrix::Identity;
	FMatrix GlobalMatrix = FMatrix::Identity;
	FMatrix InverseBindPoseMatrix = FMatrix::Identity;

	friend FArchive& operator<<(FArchive& Ar, FBone& Bone)
	{
		Ar << Bone.Name;
		Ar << Bone.ParentIndex;
		SerializeSkeletalMatrix(Ar, Bone.LocalMatrix);
		SerializeSkeletalMatrix(Ar, Bone.GlobalMatrix);
		SerializeSkeletalMatrix(Ar, Bone.InverseBindPoseMatrix);
		return Ar;
	}
};

struct FSkeletalMeshSection
{
	int32 MaterialIndex = -1;
	FString MaterialSlotName;
	uint32 FirstIndex;
	uint32 IndexCount;

	friend FArchive& operator<<(FArchive& Ar, FSkeletalMeshSection& Section)
	{
		Ar << Section.MaterialSlotName;
		Ar << Section.FirstIndex;
		Ar << Section.IndexCount;
		return Ar;
	}
};

struct FSkeletalMaterial
{
	UMaterial* MaterialInterface = nullptr;
	FString MaterialSlotName = "None";
	FString MaterialPath;

	friend FArchive& operator<<(FArchive& Ar, FSkeletalMaterial& Mat)
	{
		Ar << Mat.MaterialSlotName;

		// Material 포인터는 실행마다 달라질 수 있다.
		// .sketbin에는 다시 찾을 수 있는 .mat 경로만 저장한다.
		if (Ar.IsSaving() && Mat.MaterialInterface)
		{
			Mat.MaterialPath = Mat.MaterialInterface->GetAssetPathFileName();
		}
		Ar << Mat.MaterialPath;

		if (Ar.IsLoading())
		{
			if (!Mat.MaterialPath.empty())
			{
				Mat.MaterialInterface = FMaterialManager::Get().GetOrCreateMaterial(Mat.MaterialPath);
			}
			else
			{
				Mat.MaterialInterface = nullptr;
			}
		}

		return Ar;
	}
};

struct FSkeletalMeshRange
{
	uint32 VertexStart = 0;
	uint32 VertexEnd = 0;
	uint32 FirstIndex = 0;
	uint32 IndexCount = 0;
	FMatrix MeshBindGlobal = FMatrix::Identity;

	friend FArchive& operator<<(FArchive& Ar, FSkeletalMeshRange& Range)
	{
		Ar << Range.VertexStart;
		Ar << Range.VertexEnd;
		Ar << Range.FirstIndex;
		Ar << Range.IndexCount;
		SerializeSkeletalMatrix(Ar, Range.MeshBindGlobal);
		return Ar;
	}
};

struct FMorphTargetDelta
{
	uint32   VertexIndex = 0;
	FVector  PositionDelta;
	FVector  NormalDelta;
	FVector4 TangentDelta;

	friend FArchive& operator<<(FArchive& Ar, FMorphTargetDelta& Delta)
	{
		Ar << Delta.VertexIndex;
		Ar << Delta.PositionDelta;
		Ar << Delta.NormalDelta;
		Ar << Delta.TangentDelta;
		return Ar;
	}
};

struct FMorphTargetShape
{
	float                     FullWeight = 100.0f;
	TArray<FMorphTargetDelta> Deltas;

	friend FArchive& operator<<(FArchive& Ar, FMorphTargetShape& Shape)
	{
		Ar << Shape.FullWeight;
		Ar << Shape.Deltas;
		return Ar;
	}
};

struct FMorphTargetLOD
{
	TArray<FMorphTargetShape> Shapes;

	friend FArchive& operator<<(FArchive& Ar, FMorphTargetLOD& LOD)
	{
		Ar << LOD.Shapes;
		return Ar;
	}
};

struct FMorphTargetSourceInfo
{
	FString SourceMeshNodeName;
	FString SourceBlendShapeName;
	FString SourceChannelName;

	friend FArchive& operator<<(FArchive& Ar, FMorphTargetSourceInfo& SourceInfo)
	{
		Ar << SourceInfo.SourceMeshNodeName;
		Ar << SourceInfo.SourceBlendShapeName;
		Ar << SourceInfo.SourceChannelName;
		return Ar;
	}
};

struct FMorphTarget
{
	FString                        Name;
	TArray<FMorphTargetSourceInfo> SourceInfos;
	TArray<FMorphTargetLOD>        LODModels;

	friend FArchive& operator<<(FArchive& Ar, FMorphTarget& Morph)
	{
		Ar << Morph.Name;
		Ar << Morph.SourceInfos;
		Ar << Morph.LODModels;
		return Ar;
	}
};

enum class ESkeletalStaticChildImportAction : uint8
{
	MergeAsRigidPart = 0,
	KeepAsAttachedStaticMesh,
	Ignore
};

struct FSkeletalStaticChildMesh
{
	FString                          SourceNodeName;
	int32                            ParentBoneIndex = -1;
	FString                          ParentBoneName;
	FMatrix                          LocalMatrixToParentBone = FMatrix::Identity;
	ESkeletalStaticChildImportAction ImportAction            = ESkeletalStaticChildImportAction::MergeAsRigidPart;
	FString                          StaticMeshAssetPath;

	friend FArchive& operator<<(FArchive& Ar, FSkeletalStaticChildMesh& Mesh)
	{
		Ar << Mesh.SourceNodeName;
		Ar << Mesh.ParentBoneIndex;
		Ar << Mesh.ParentBoneName;
		SerializeSkeletalMatrix(Ar, Mesh.LocalMatrixToParentBone);
		uint8 ActionValue = static_cast<uint8>(Mesh.ImportAction);
		Ar << ActionValue;
		if (Ar.IsLoading()) Mesh.ImportAction = static_cast<ESkeletalStaticChildImportAction>(ActionValue);
		Ar << Mesh.StaticMeshAssetPath;
		return Ar;
	}
};

struct FFbxSplitStaticMeshReference
{
	FString SourceNodeName;
	FString StaticMeshAssetPath;
	FMatrix GlobalMatrix = FMatrix::Identity;

	friend FArchive& operator<<(FArchive& Ar, FFbxSplitStaticMeshReference& Ref)
	{
		Ar << Ref.SourceNodeName;
		Ar << Ref.StaticMeshAssetPath;
		SerializeSkeletalMatrix(Ar, Ref.GlobalMatrix);
		return Ar;
	}
};

struct FSkeletalSocket
{
	FString Name;
	FString SourceNodeName;
	int32   ParentBoneIndex = -1;
	FString ParentBoneName;
	FMatrix LocalMatrixToParentBone = FMatrix::Identity;

	friend FArchive& operator<<(FArchive& Ar, FSkeletalSocket& Socket)
	{
		Ar << Socket.Name;
		Ar << Socket.SourceNodeName;
		Ar << Socket.ParentBoneIndex;
		Ar << Socket.ParentBoneName;
		SerializeSkeletalMatrix(Ar, Socket.LocalMatrixToParentBone);
		return Ar;
	}
};

enum class ESkeletalImportWarningType : uint8
{
	None = 0,
	MissingNormal,
	GeneratedNormal,
	MissingTangent,
	GeneratedTangent,
	MissingUV,
	MissingSkinWeight,
	MoreThanFourInfluences,
	BoneIndexOverflow,
	UnsupportedSkinningType,
	UnsupportedClusterLinkMode,
	MissingBindPose,
	UsedClusterBindPoseFallback,
	UsedSceneTransformFallback,
	UnsupportedMorphInBetween,
	UnsupportedMorphAnimation,
	UnsupportedMaterialProperty,
	StaticChildOfBone,
	CollisionProxySkippedFromRenderLOD,
	SocketWithoutParentBone,
	DuplicateSocketName,
	BindPoseValidationError,
};

struct FSkeletalImportWarning
{
	ESkeletalImportWarningType Type = ESkeletalImportWarningType::None;
	FString                    Message;

	friend FArchive& operator<<(FArchive& Ar, FSkeletalImportWarning& Warning)
	{
		uint8 TypeValue = static_cast<uint8>(Warning.Type);
		Ar << TypeValue;
		if (Ar.IsLoading()) Warning.Type = static_cast<ESkeletalImportWarningType>(TypeValue);
		Ar << Warning.Message;
		return Ar;
	}
};

struct FSkeletalImportSummary
{
	FString                        SourcePath;
	int32                          SourceMeshCount          = 0;
	int32                          ImportedSkinnedMeshCount = 0;
	int32                          BoneCount                = 0;
	int32                          LODCount                 = 0;
	int32                          MaterialSlotCount        = 0;
	int32                          VertexCount              = 0;
	int32                          TriangleCount            = 0;
	int32                          AnimationClipCount       = 0;
	int32                          MorphTargetCount         = 0;
	int32                          MorphTargetShapeCount    = 0;
	int32                          MorphTargetDeltaCount    = 0;
	int32                          StaticChildMeshCount     = 0;
	int32                          SplitStaticMeshCount     = 0;
	int32                          SocketCount              = 0;
	int32                          CollisionProxyMeshCount  = 0;
	int32                          MetadataNodeCount        = 0;
	int32                          SceneNodeCount           = 0;
	float                          MaxBindPoseValidationError = 0.0f;
	TArray<FSkeletalImportWarning> Warnings;

	friend FArchive& operator<<(FArchive& Ar, FSkeletalImportSummary& Summary)
	{
		Ar << Summary.SourcePath;
		Ar << Summary.SourceMeshCount;
		Ar << Summary.ImportedSkinnedMeshCount;
		Ar << Summary.BoneCount;
		Ar << Summary.LODCount;
		Ar << Summary.MaterialSlotCount;
		Ar << Summary.VertexCount;
		Ar << Summary.TriangleCount;
		Ar << Summary.AnimationClipCount;
		Ar << Summary.MorphTargetCount;
		Ar << Summary.MorphTargetShapeCount;
		Ar << Summary.MorphTargetDeltaCount;
		Ar << Summary.StaticChildMeshCount;
		Ar << Summary.SplitStaticMeshCount;
		Ar << Summary.SocketCount;
		Ar << Summary.CollisionProxyMeshCount;
		Ar << Summary.MetadataNodeCount;
		Ar << Summary.SceneNodeCount;
		Ar << Summary.MaxBindPoseValidationError;
		Ar << Summary.Warnings;
		return Ar;
	}
};

struct FSkeletalMesh
{
	FString PathFileName;
	FString SkeletonPath = "None";
	FString SkeletonAssetGuid;
	FString SkeletonCompatibilitySignature;

	TArray<FVertexPNCTBW> Vertices;
	TArray<uint32> Indices;

	TArray<FSkeletalMeshSection> Sections;
	TArray<FSkeletalMeshRange> MeshRanges;

	TArray<FBone> Bones;

	TArray<FMorphTarget>                 MorphTargets;
	TArray<FSkeletalStaticChildMesh>     StaticChildMeshes;
	TArray<FFbxSplitStaticMeshReference> SplitStaticMeshes;
	TArray<FImportedCollisionShape>      CollisionShapes;
	TArray<FSkeletalSocket>              Sockets;
	TArray<FFbxImportedNodeMetadata>     NodeMetadata;
	TArray<FFbxImportedSceneNode>        SceneNodes;
	FSkeletalImportSummary               ImportSummary;

	std::unique_ptr<FMeshBuffer> RenderBuffer;

	FVector BoundsCenter = FVector(0, 0, 0);
	FVector BoundsExtent = FVector(0, 0, 0);
	bool    bBoundsValid = false;

	void CacheBounds()
	{
		bBoundsValid = false;
		if (Vertices.empty()) return;

		FVector LocalMin = Vertices[0].Position;
		FVector LocalMax = Vertices[0].Position;
		for (const FVertexPNCTBW& Vertex : Vertices)
		{
			LocalMin.X = std::min<float>(LocalMin.X, Vertex.Position.X);
			LocalMin.Y = std::min<float>(LocalMin.Y, Vertex.Position.Y);
			LocalMin.Z = std::min<float>(LocalMin.Z, Vertex.Position.Z);
			LocalMax.X = std::max<float>(LocalMax.X, Vertex.Position.X);
			LocalMax.Y = std::max<float>(LocalMax.Y, Vertex.Position.Y);
			LocalMax.Z = std::max<float>(LocalMax.Z, Vertex.Position.Z);
		}

		BoundsCenter = (LocalMin + LocalMax) * 0.5f;
		BoundsExtent = (LocalMax - LocalMin) * 0.5f;
		bBoundsValid = true;
	}
};
