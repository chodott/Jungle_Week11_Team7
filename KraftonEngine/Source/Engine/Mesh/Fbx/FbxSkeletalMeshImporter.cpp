#include "Mesh/Fbx/FbxSkeletalMeshImporter.h"
#include "Mesh/Fbx/FbxSceneQuery.h"
#include "Mesh/Fbx/FbxMaterialImporter.h"
#include "Mesh/Fbx/FbxSkeletonImporter.h"
#include "Mesh/Fbx/FbxSkinWeightImporter.h"
#include "Mesh/Fbx/FbxAnimationImporter.h"
#include "Mesh/Fbx/FbxMorphTargetImporter.h"
#include "Mesh/Fbx/FbxImportValidation.h"
#include "Mesh/Fbx/FbxCollisionImporter.h"
#include "Mesh/Fbx/FbxMetadataImporter.h"
#include "Mesh/Fbx/FbxSceneHierarchyImporter.h"
#include "Mesh/Fbx/FbxSocketImporter.h"
#include "Mesh/Fbx/FbxTransformUtils.h"
#include "Animation/AnimationRuntime.h"
#include "Animation/AnimDataModel.h"
#include "Animation/AnimSequence.h"
#include "Math/Transform.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

namespace
{

	static FMatrix GetReferenceMeshBindInverse(const FFbxImportContext& Context)
	{
		return Context.ReferenceMeshBindInverse;
	}

	enum class EFbxSkeletalImportMeshKind : uint8
	{
		Skinned,
		StaticChildOfBone,
		Loose,
		CollisionProxy,
		Ignored
	};

	struct FFbxSkeletalImportMeshNode
	{
		FbxNode*                         MeshNode = nullptr;
		FString                          SourceNodeName;
		EFbxSkeletalImportMeshKind       Kind                    = EFbxSkeletalImportMeshKind::Ignored;
		FbxNode*                         ParentBoneNode          = nullptr;
		int32                            ParentBoneIndex         = -1;
		FMatrix                          LocalMatrixToParentBone = FMatrix::Identity;
		ESkeletalStaticChildImportAction StaticChildAction       = ESkeletalStaticChildImportAction::MergeAsRigidPart;
	};

	static ESkeletalStaticChildImportAction ReadStaticChildAction(FbxNode* Node)
	{
		const FString ImportKind = FFbxSceneQuery::ReadStringProperty(Node, "ImportKind");
		if (ImportKind == "Attachment" || ImportKind == "KeepAsAttachedStaticMesh")
		{
			return ESkeletalStaticChildImportAction::KeepAsAttachedStaticMesh;
		}
		if (ImportKind == "Ignore")
		{
			return ESkeletalStaticChildImportAction::Ignore;
		}
		return ESkeletalStaticChildImportAction::MergeAsRigidPart;
	}

	static bool IsExplicitIgnoredNode(FbxNode* Node)
	{
		return FFbxSceneQuery::ReadStringProperty(Node, "ImportKind") == "Ignore";
	}

	static FMatrix ComputeMeshLocalMatrixToBone(FbxNode* MeshNode, FbxNode* BoneNode)
	{
		if (!MeshNode || !BoneNode)
		{
			return FMatrix::Identity;
		}

		const FMatrix MeshGlobalScene = FFbxTransformUtils::ToEngineMatrix(
			FFbxTransformUtils::GetNodeGeometryTransform(MeshNode)
		) * FFbxTransformUtils::ToEngineMatrix(MeshNode->EvaluateGlobalTransform());
		const FMatrix BoneGlobalScene = FFbxTransformUtils::ToEngineMatrix(BoneNode->EvaluateGlobalTransform());
		return MeshGlobalScene * BoneGlobalScene.GetInverse();
	}

	static void ClassifyMeshNodes(const FFbxImportContext& Context, TArray<FFbxSkeletalImportMeshNode>& OutImportNodes)
	{
		OutImportNodes.clear();
		for (FbxNode* MeshNode : Context.MeshNodes)
		{
			FbxMesh* Mesh = MeshNode ? MeshNode->GetMesh() : nullptr;
			if (!Mesh)
			{
				continue;
			}

			FFbxSkeletalImportMeshNode ImportNode;
			ImportNode.MeshNode       = MeshNode;
			ImportNode.SourceNodeName = MeshNode->GetName() ? FString(MeshNode->GetName()) : FString();

			FbxNode*   ParentBoneNode  = nullptr;
			int32      ParentBoneIndex = -1;
			const bool bHasParentBone  = FFbxSceneQuery::FindNearestParentBoneIndex(
				MeshNode,
				Context.BoneNodeToIndex,
				ParentBoneNode,
				ParentBoneIndex
			);

			if (IsExplicitIgnoredNode(MeshNode))
			{
				ImportNode.Kind = EFbxSkeletalImportMeshKind::Ignored;
				OutImportNodes.push_back(ImportNode);
				continue;
			}

			if (FFbxSceneQuery::MeshHasSkin(Mesh))
			{
				ImportNode.Kind = EFbxSkeletalImportMeshKind::Skinned;
				OutImportNodes.push_back(ImportNode);
				continue;
			}

			if (FFbxSceneQuery::IsCollisionProxyNode(MeshNode))
			{
				ImportNode.Kind                    = EFbxSkeletalImportMeshKind::CollisionProxy;
				ImportNode.ParentBoneIndex         = ParentBoneIndex;
				ImportNode.ParentBoneNode          = ParentBoneNode;
				ImportNode.LocalMatrixToParentBone = bHasParentBone
				? ComputeMeshLocalMatrixToBone(MeshNode, ParentBoneNode) : FMatrix::Identity;
				OutImportNodes.push_back(ImportNode);
				continue;
			}

			if (bHasParentBone)
			{
				ImportNode.Kind                    = EFbxSkeletalImportMeshKind::StaticChildOfBone;
				ImportNode.ParentBoneIndex         = ParentBoneIndex;
				ImportNode.ParentBoneNode          = ParentBoneNode;
				ImportNode.LocalMatrixToParentBone = ComputeMeshLocalMatrixToBone(MeshNode, ParentBoneNode);
				ImportNode.StaticChildAction       = ReadStaticChildAction(MeshNode);
				OutImportNodes.push_back(ImportNode);
				continue;
			}

			ImportNode.Kind = EFbxSkeletalImportMeshKind::Loose;
			OutImportNodes.push_back(ImportNode);
		}
	}

	static void RebuildReferenceSkeletonFromBones(const TArray<FBone>& Bones, FReferenceSkeleton& OutReferenceSkeleton)
	{
		OutReferenceSkeleton.Bones.clear();
		OutReferenceSkeleton.Bones.reserve(Bones.size());
		for (const FBone& Bone : Bones)
		{
			FReferenceBone RefBone;
			RefBone.Name            = Bone.Name;
			RefBone.ParentIndex     = Bone.ParentIndex;
			RefBone.LocalBindPose   = Bone.LocalMatrix;
			RefBone.GlobalBindPose  = Bone.GlobalMatrix;
			RefBone.InverseBindPose = Bone.InverseBindPoseMatrix;
			OutReferenceSkeleton.Bones.push_back(RefBone);
		}
	}

	static void ImportSkeletalCollisionShapes(
		FbxScene* Scene,
		const FFbxImportContext& Context,
		const FReferenceSkeleton& ReferenceSkeleton,
		const FMatrix& ReferenceMeshBindInverse,
		TArray<FImportedCollisionShape>& OutCollisionShapes
		)
	{
		(void)Scene;
		(void)ReferenceMeshBindInverse;
		OutCollisionShapes.clear();

		TArray<FFbxSkeletalImportMeshNode> ImportNodes;
		ClassifyMeshNodes(Context, ImportNodes);

		for (const FFbxSkeletalImportMeshNode& ImportNode : ImportNodes)
		{
			if (ImportNode.Kind != EFbxSkeletalImportMeshKind::CollisionProxy || !ImportNode.MeshNode)
			{
				continue;
			}

			FString ParentBoneName;
			if (ImportNode.ParentBoneIndex >= 0 && ImportNode.ParentBoneIndex < static_cast<int32>(ReferenceSkeleton.
				Bones.size()))
			{
				ParentBoneName = ReferenceSkeleton.Bones[ImportNode.ParentBoneIndex].Name;
			}

			FImportedCollisionShape Shape;
			if (FFbxCollisionImporter::ImportCollisionShape(
				ImportNode.MeshNode,
				ImportNode.LocalMatrixToParentBone,
				ImportNode.ParentBoneIndex,
				ParentBoneName,
				Shape
			))
			{
				OutCollisionShapes.push_back(std::move(Shape));
			}
		}
	}


	static void ImportSkeletalStaticChildReferences(
		const FFbxImportContext& Context,
		const FReferenceSkeleton& ReferenceSkeleton,
		const FMatrix& ReferenceMeshBindInverse,
		TArray<FSkeletalStaticChildMesh>& OutStaticChildMeshes,
		TArray<FFbxSplitStaticMeshReference>& OutSplitStaticMeshes
		)
	{
		(void)ReferenceMeshBindInverse;
		OutStaticChildMeshes.clear();
		OutSplitStaticMeshes.clear();

		TArray<FFbxSkeletalImportMeshNode> ImportNodes;
		ClassifyMeshNodes(Context, ImportNodes);

		for (const FFbxSkeletalImportMeshNode& ImportNode : ImportNodes)
		{
			if (!ImportNode.MeshNode)
			{
				continue;
			}

			if (ImportNode.Kind == EFbxSkeletalImportMeshKind::StaticChildOfBone)
			{
				if (ImportNode.StaticChildAction == ESkeletalStaticChildImportAction::Ignore)
				{
					continue;
				}

				FSkeletalStaticChildMesh Child;
				Child.SourceNodeName  = ImportNode.SourceNodeName;
				Child.ParentBoneIndex = ImportNode.ParentBoneIndex;
				Child.ParentBoneName  = (ImportNode.ParentBoneIndex >= 0 && ImportNode.ParentBoneIndex < static_cast<
					int32>(ReferenceSkeleton.Bones.size())) ? ReferenceSkeleton.Bones[ImportNode.ParentBoneIndex].Name
				: FString();
				Child.LocalMatrixToParentBone = ImportNode.LocalMatrixToParentBone;
				Child.ImportAction            = ImportNode.StaticChildAction;
				OutStaticChildMeshes.push_back(Child);
				continue;
			}

			if (ImportNode.Kind == EFbxSkeletalImportMeshKind::Loose)
			{
				FFbxSplitStaticMeshReference Split;
				Split.SourceNodeName = ImportNode.SourceNodeName;
				Split.GlobalMatrix = FFbxTransformUtils::ToEngineMatrix(ImportNode.MeshNode->EvaluateGlobalTransform());
				OutSplitStaticMeshes.push_back(Split);
			}
		}
	}

	static void FlipIndexWinding(TArray<uint32>& Indices)
	{
		for (size_t Index = 0; Index + 2 < Indices.size(); Index += 3)
		{
			std::swap(Indices[Index + 1], Indices[Index + 2]);
		}
	}

	static void RecomputeBoneLocalMatrices(TArray<FBone>& Bones)
	{
		for (int32 BoneIndex = 0; BoneIndex < static_cast<int32>(Bones.size()); ++BoneIndex)
		{
			FBone& Bone = Bones[BoneIndex];
			if (Bone.ParentIndex >= 0 && Bone.ParentIndex < static_cast<int32>(Bones.size()))
			{
				Bone.LocalMatrix = Bone.GlobalMatrix * Bones[Bone.ParentIndex].GlobalMatrix.GetInverse();
			}
			else
			{
				Bone.LocalMatrix = Bone.GlobalMatrix;
			}
			Bone.InverseBindPoseMatrix = Bone.GlobalMatrix.GetInverse();
		}
	}

	static void TransformVerticesToEngineAssetSpace(FSkeletalMesh& Mesh, const FMatrix& AssetCorrection)
	{
		const FMatrix DirectionCorrection = FFbxTransformUtils::RemoveTranslationFromMatrix(AssetCorrection);
		const FMatrix NormalCorrection    = DirectionCorrection.GetInverse().GetTransposed();
		const bool    bReverseWinding     = FFbxTransformUtils::Determinant3x3(DirectionCorrection) < 0.0f;

		for (FVertexPNCTBW& Vertex : Mesh.Vertices)
		{
			Vertex.Position = FFbxTransformUtils::TransformPositionByMatrix(Vertex.Position, AssetCorrection);
			Vertex.Normal   = FFbxTransformUtils::TransformNormalByMatrix(Vertex.Normal, NormalCorrection);

			const FVector Tangent = FFbxTransformUtils::TransformTangentByMatrix(
				FVector(Vertex.Tangent.X, Vertex.Tangent.Y, Vertex.Tangent.Z),
				DirectionCorrection,
				Vertex.Normal
			);
			Vertex.Tangent = FVector4(
				Tangent.X,
				Tangent.Y,
				Tangent.Z,
				bReverseWinding ? -Vertex.Tangent.W : Vertex.Tangent.W
			);
		}

		if (bReverseWinding)
		{
			FlipIndexWinding(Mesh.Indices);
		}
	}

	static void TransformMorphTargetsToEngineAssetSpace(
		TArray<FMorphTarget>& MorphTargets,
		const FMatrix&        AssetCorrection
		)
	{
		const FMatrix DirectionCorrection = FFbxTransformUtils::RemoveTranslationFromMatrix(AssetCorrection);
		const FMatrix NormalCorrection    = DirectionCorrection.GetInverse().GetTransposed();

		for (FMorphTarget& MorphTarget : MorphTargets)
		{
			for (FMorphTargetLOD& MorphLOD : MorphTarget.LODModels)
			{
				for (FMorphTargetShape& Shape : MorphLOD.Shapes)
				{
					for (FMorphTargetDelta& Delta : Shape.Deltas)
					{
						Delta.PositionDelta = FFbxTransformUtils::TransformVectorNoNormalizeByMatrix(
							Delta.PositionDelta,
							DirectionCorrection
						);
						Delta.NormalDelta = FFbxTransformUtils::TransformNormalByMatrix(
							Delta.NormalDelta,
							NormalCorrection
						);

						const FVector TangentDelta = FFbxTransformUtils::TransformVectorNoNormalizeByMatrix(
							FVector(Delta.TangentDelta.X, Delta.TangentDelta.Y, Delta.TangentDelta.Z),
							DirectionCorrection
						);
						Delta.TangentDelta = FVector4(
							TangentDelta.X,
							TangentDelta.Y,
							TangentDelta.Z,
							Delta.TangentDelta.W
						);
					}
				}
			}
		}
	}

	static void TransformSkeletonToEngineAssetSpace(FSkeletalMesh& Mesh, const FMatrix& AssetCorrection)
	{
		for (FBone& Bone : Mesh.Bones)
		{
			Bone.GlobalMatrix          = Bone.GlobalMatrix * AssetCorrection;
			Bone.InverseBindPoseMatrix = Bone.GlobalMatrix.GetInverse();
		}
		RecomputeBoneLocalMatrices(Mesh.Bones);
	}

	static void TransformSceneNodesToEngineAssetSpace(
		TArray<FFbxImportedSceneNode>& SceneNodes,
		const FMatrix&                 AssetCorrection
		)
	{
		for (FFbxImportedSceneNode& SceneNode : SceneNodes)
		{
			SceneNode.GlobalMatrix         = SceneNode.GlobalMatrix * AssetCorrection;
			SceneNode.GlobalGeometryMatrix = SceneNode.GlobalGeometryMatrix * AssetCorrection;
		}
	}

	static void TransformSplitStaticMeshesToEngineAssetSpace(
		TArray<FFbxSplitStaticMeshReference>& SplitStaticMeshes,
		const FMatrix&                        AssetCorrection
		)
	{
		for (FFbxSplitStaticMeshReference& SplitRef : SplitStaticMeshes)
		{
			SplitRef.GlobalMatrix = SplitRef.GlobalMatrix * AssetCorrection;
		}
	}

	static void TransformAnimationRootTracksToEngineAssetSpace(
		const FSkeletalMesh&    Mesh,
		TArray<UAnimSequence*>& AnimSequences,
		const FMatrix&          AssetCorrection
		)
	{
		for (UAnimSequence* Sequence : AnimSequences)
		{
			UAnimDataModel* DataModel = Sequence ? Sequence->GetDataModel() : nullptr;
			if (!DataModel)
			{
				continue;
			}

			for (FBoneAnimationTrack& Track : DataModel->GetMutableBoneAnimationTracks())
			{
				const int32 BoneIndex = Track.BoneTreeIndex;
				if (BoneIndex < 0 || BoneIndex >= static_cast<int32>(Mesh.Bones.size()) || Mesh.Bones[BoneIndex].
					ParentIndex >= 0)
				{
					continue;
				}

				FRawAnimSequenceTrack& Raw      = Track.InternalTrackData;
				const size_t           KeyCount = (std::min)(
					Raw.PosKeys.size(),
					(std::min)(Raw.RotKeys.size(), Raw.ScaleKeys.size())
				);
				for (size_t KeyIndex = 0; KeyIndex < KeyCount; ++KeyIndex)
				{
					const FMatrix LocalMatrix = FTransform(
						Raw.PosKeys[KeyIndex],
						Raw.RotKeys[KeyIndex],
						Raw.ScaleKeys[KeyIndex]
					).ToMatrix();
					const FTransform Corrected = FAnimationRuntime::DecomposeMatrix(LocalMatrix * AssetCorrection);
					Raw.PosKeys[KeyIndex]      = Corrected.Location;
					Raw.RotKeys[KeyIndex]      = Corrected.Rotation.GetNormalized();
					Raw.ScaleKeys[KeyIndex]    = Corrected.Scale;
				}
			}
		}
	}

	static FMatrix GetEngineAssetCorrection(FbxScene* Scene, const FFbxImportContext& Context)
	{
		const FbxAxisSystem NormalizedAxisSystem = Scene ? Scene->GetGlobalSettings().GetAxisSystem() : FbxAxisSystem(
			FbxAxisSystem::eZAxis,
			FbxAxisSystem::eParityOdd,
			FbxAxisSystem::eLeftHanded
		);
		const bool bMirrorHandedness = Context.bHasSourceCoordSystem && Context.SourceCoordSystem !=
		NormalizedAxisSystem.GetCoorSystem();
		return FFbxTransformUtils::MakeAxisSystemToEngineAssetMatrix(NormalizedAxisSystem, bMirrorHandedness);
	}

	static void BakeNormalizedSkeletalSpaceToEngineAssetSpace(
		FbxScene*                Scene,
		const FFbxImportContext& Context,
		FSkeletalMesh&           Mesh,
		TArray<UAnimSequence*>*  AnimSequences,
		FReferenceSkeleton*      OutReferenceSkeleton
		)
	{
		const FMatrix AssetCorrection = GetEngineAssetCorrection(Scene, Context);

		TransformVerticesToEngineAssetSpace(Mesh, AssetCorrection);
		TransformMorphTargetsToEngineAssetSpace(Mesh.MorphTargets, AssetCorrection);
		TransformSkeletonToEngineAssetSpace(Mesh, AssetCorrection);
		TransformSplitStaticMeshesToEngineAssetSpace(Mesh.SplitStaticMeshes, AssetCorrection);
		TransformSceneNodesToEngineAssetSpace(Mesh.SceneNodes, AssetCorrection);

		if (AnimSequences)
		{
			TransformAnimationRootTracksToEngineAssetSpace(Mesh, *AnimSequences, AssetCorrection);
		}

		if (OutReferenceSkeleton)
		{
			RebuildReferenceSkeletonFromBones(Mesh.Bones, *OutReferenceSkeleton);
		}
	}

	static void RefreshPostBakeValidation(FSkeletalMesh& Mesh)
	{
		Mesh.ImportSummary.MaxBindPoseValidationError = FFbxImportValidation::ValidateBindPoseSkinningError(Mesh);
	}


	static bool ImportMeshCore(
		FbxScene*                         Scene,
		FFbxImportContext&                Context,
		FSkeletalMesh&                    OutMesh,
		TArray<FSkeletalMaterial>&        OutMaterials,
		FReferenceSkeleton&               OutSourceSkeleton,
		TArray<FFbxImportedMaterialInfo>& OutSourceMaterials,
		FString*                          OutMessage
		)
	{
		FbxNode* RootNode = Scene ? Scene->GetRootNode() : nullptr;
		if (!RootNode)
		{
			if (OutMessage) *OutMessage = "FBX skeletal mesh import failed: root node not found.";
			return false;
		}

		Context.AllNodes.clear();
		Context.MeshNodes.clear();
		Context.AnimSequences.clear();
		Context.Summary            = FSkeletalImportSummary();
		Context.Summary.SourcePath = Context.SourcePath;
		FFbxSceneQuery::CollectAllNodes(RootNode, Context.AllNodes);
		FFbxSceneQuery::CollectMeshNodes(RootNode, Context.MeshNodes);
		FFbxSceneHierarchyImporter::CollectSceneNodes(Scene, OutMesh.SceneNodes);
		FFbxMetadataImporter::CollectSceneNodeMetadata(Scene, OutMesh.NodeMetadata);

		FFbxMaterialImporter::CollectMaterials(Scene, Context);

		if (!FFbxSkeletonImporter::ImportSkeleton(Scene, Context, OutMessage))
		{
			return false;
		}

		if (!FFbxSkinWeightImporter::ImportSkin(Scene, Context, OutMessage))
		{
			return false;
		}

		FFbxMorphTargetImporter::ImportMorphTargets(Context.MorphSourcesByLOD, Context.MorphTargets, Context);
		// Skin import can refine inverse bind poses from FBX clusters, so rebuild the reference skeleton after skin data is processed.
		RebuildReferenceSkeletonFromBones(Context.Bones, Context.ReferenceSkeleton);

		const FMatrix ReferenceMeshBindInverse = GetReferenceMeshBindInverse(Context);
		ImportSkeletalCollisionShapes(Scene, Context, Context.ReferenceSkeleton, ReferenceMeshBindInverse, OutMesh.CollisionShapes);
		ImportSkeletalStaticChildReferences(Context, Context.ReferenceSkeleton, ReferenceMeshBindInverse, OutMesh.StaticChildMeshes, OutMesh.SplitStaticMeshes);
		FFbxSocketImporter::ImportSockets(Scene, Context.BoneNodeToIndex, ReferenceMeshBindInverse, Context.ReferenceSkeleton, OutMesh.Sockets, Context);

		OutMesh.Vertices     = std::move(Context.SkeletalVertices);
		OutMesh.Indices      = std::move(Context.SkeletalIndices);
		OutMesh.Sections     = std::move(Context.SkeletalSections);
		OutMesh.MeshRanges   = std::move(Context.SkeletalMeshRanges);
		OutMesh.Bones        = Context.Bones;
		OutMesh.MorphTargets = std::move(Context.MorphTargets);
		OutMesh.PathFileName = Context.SourcePath;

		OutMesh.ImportSummary                          = Context.Summary;
		OutMesh.ImportSummary.SourcePath               = Context.SourcePath;
		OutMesh.ImportSummary.SourceMeshCount          = static_cast<int32>(Context.MeshNodes.size());
		OutMesh.ImportSummary.ImportedSkinnedMeshCount = static_cast<int32>(Context.SkeletalMeshRanges.size());
		OutMesh.ImportSummary.BoneCount                = static_cast<int32>(OutMesh.Bones.size());
		OutMesh.ImportSummary.VertexCount              = static_cast<int32>(OutMesh.Vertices.size());
		OutMesh.ImportSummary.TriangleCount            = static_cast<int32>(OutMesh.Indices.size() / 3);
		OutMesh.ImportSummary.MaterialSlotCount        = static_cast<int32>(Context.Materials.size());
		OutMesh.ImportSummary.LODCount                 = 1;
		OutMesh.ImportSummary.CollisionProxyMeshCount  = static_cast<int32>(OutMesh.CollisionShapes.size());
		OutMesh.ImportSummary.StaticChildMeshCount     = static_cast<int32>(OutMesh.StaticChildMeshes.size());
		OutMesh.ImportSummary.SplitStaticMeshCount     = static_cast<int32>(OutMesh.SplitStaticMeshes.size());
		OutMesh.ImportSummary.SocketCount              = static_cast<int32>(OutMesh.Sockets.size());
		OutMesh.ImportSummary.MetadataNodeCount        = static_cast<int32>(OutMesh.NodeMetadata.size());
		OutMesh.ImportSummary.SceneNodeCount           = static_cast<int32>(OutMesh.SceneNodes.size());
		OutMesh.ImportSummary.MorphTargetCount         = static_cast<int32>(OutMesh.MorphTargets.size());
		for (const FMorphTarget& Morph : OutMesh.MorphTargets)
		{
			for (const FMorphTargetLOD& LOD : Morph.LODModels)
			{
				for (const FMorphTargetShape& Shape : LOD.Shapes)
				{
					++OutMesh.ImportSummary.MorphTargetShapeCount;
					OutMesh.ImportSummary.MorphTargetDeltaCount += static_cast<int32>(Shape.Deltas.size());
				}
			}
		}

		OutMesh.ImportSummary.MaxBindPoseValidationError = FFbxImportValidation::ValidateBindPoseSkinningError(OutMesh);
		if (OutMesh.ImportSummary.MaxBindPoseValidationError > 0.001f)
		{
			FSkeletalImportWarning Warning;
			Warning.Type = ESkeletalImportWarningType::BindPoseValidationError;
			Warning.Message = "Bind pose validation error is larger than tolerance: " + std::to_string(OutMesh.ImportSummary.MaxBindPoseValidationError);
			OutMesh.ImportSummary.Warnings.push_back(Warning);
		}

		OutSourceSkeleton  = Context.ReferenceSkeleton;
		OutSourceMaterials = Context.Materials;
		FFbxMaterialImporter::BuildSkeletalMaterials(Context, OutMesh.Sections, OutMaterials, OutMesh.Sections);
		return true;
	}
}

bool FFbxSkeletalMeshImporter::Import(FbxScene* Scene, FFbxImportContext& Context, FFbxSkeletalMeshImportResult& OutResult, FString* OutMessage)
{
	OutResult = FFbxSkeletalMeshImportResult();

	if (!ImportMeshCore(Scene, Context, OutResult.Mesh, OutResult.Materials, OutResult.Skeleton, OutResult.SourceMaterials, OutMessage))
	{
		return false;
	}

	if (!FFbxAnimationImporter::ImportAnimations(Scene, Context, OutMessage))
	{
		return false;
	}

	OutResult.AnimSequences = std::move(Context.AnimSequences);
	BakeNormalizedSkeletalSpaceToEngineAssetSpace(
		Scene,
		Context,
		OutResult.Mesh,
		&OutResult.AnimSequences,
		&OutResult.Skeleton
	);
	RefreshPostBakeValidation(OutResult.Mesh);
	return true;
}

bool FFbxSkeletalMeshImporter::ImportMeshOnly(FbxScene* Scene, FFbxImportContext& Context, FFbxSkeletalMeshOnlyImportResult& OutResult, FString* OutMessage)
{
	OutResult = FFbxSkeletalMeshOnlyImportResult();
	if (!ImportMeshCore(
		Scene,
		Context,
		OutResult.Mesh,
		OutResult.Materials,
		OutResult.SourceSkeleton,
		OutResult.SourceMaterials,
		OutMessage
	))
	{
		return false;
	}
	BakeNormalizedSkeletalSpaceToEngineAssetSpace(Scene, Context, OutResult.Mesh, nullptr, &OutResult.SourceSkeleton);
	RefreshPostBakeValidation(OutResult.Mesh);
	return true;
}

void FFbxSkeletalMeshImporter::BakeAnimationOnlyToEngineAssetSpace(
	FbxScene*                Scene,
	const FFbxImportContext& Context,
	FReferenceSkeleton&      InOutSourceSkeleton,
	TArray<UAnimSequence*>&  InOutAnimSequences
	)
{
	FSkeletalMesh TemporaryMesh;
	TemporaryMesh.Bones = Context.Bones;

	BakeNormalizedSkeletalSpaceToEngineAssetSpace(
		Scene,
		Context,
		TemporaryMesh,
		&InOutAnimSequences,
		&InOutSourceSkeleton
	);
}
