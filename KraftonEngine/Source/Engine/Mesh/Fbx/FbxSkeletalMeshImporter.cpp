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

#include <string>
#include <utility>

namespace
{

	static FMatrix GetReferenceMeshBindInverse(const FFbxImportContext& Context)
	{
		if (!Context.SkeletalMeshRanges.empty())
		{
			return Context.SkeletalMeshRanges[0].MeshBindGlobal.GetInverse();
		}
		return FMatrix::Identity;
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
		OutCollisionShapes.clear();

		for (FbxNode* Node : Context.MeshNodes)
		{
			if (!Node || !FFbxSceneQuery::IsCollisionProxyNode(Node))
			{
				continue;
			}

			FbxNode* ParentBoneNode = nullptr;
			int32 ParentBoneIndex = -1;
			const FMatrix GlobalReferenceMatrix = FFbxTransformUtils::ToEngineMatrix(Node->EvaluateGlobalTransform() * FFbxTransformUtils::GetGeometryTransform(Node)) * ReferenceMeshBindInverse;
			FMatrix LocalMatrix = GlobalReferenceMatrix;
			FString ParentBoneName;

			if (FFbxSceneQuery::FindNearestParentBoneIndex(Node, Context.BoneNodeToIndex, ParentBoneNode, ParentBoneIndex)
				&& ParentBoneIndex >= 0
				&& ParentBoneIndex < static_cast<int32>(ReferenceSkeleton.Bones.size()))
			{
				ParentBoneName = ReferenceSkeleton.Bones[ParentBoneIndex].Name;
				LocalMatrix = GlobalReferenceMatrix * ReferenceSkeleton.Bones[ParentBoneIndex].GlobalBindPose.GetInverse();
			}
			else
			{
				ParentBoneIndex = -1;
			}

			FImportedCollisionShape Shape;
			if (FFbxCollisionImporter::ImportCollisionShape(Node, LocalMatrix, ParentBoneIndex, ParentBoneName, Shape))
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
		OutStaticChildMeshes.clear();
		OutSplitStaticMeshes.clear();

		for (FbxNode* Node : Context.MeshNodes)
		{
			FbxMesh* Mesh = Node ? Node->GetMesh() : nullptr;
			if (!Node || !Mesh || FFbxSceneQuery::IsCollisionProxyNode(Node) || FFbxSceneQuery::MeshHasSkin(Mesh))
			{
				continue;
			}

			const FMatrix GlobalMatrix = FFbxTransformUtils::ToEngineMatrix(Node->EvaluateGlobalTransform() * FFbxTransformUtils::GetGeometryTransform(Node));
			const FMatrix GlobalReferenceMatrix = GlobalMatrix * ReferenceMeshBindInverse;
			FbxNode* ParentBoneNode = nullptr;
			int32 ParentBoneIndex = -1;
			if (FFbxSceneQuery::FindNearestParentBoneIndex(Node, Context.BoneNodeToIndex, ParentBoneNode, ParentBoneIndex)
				&& ParentBoneIndex >= 0
				&& ParentBoneIndex < static_cast<int32>(ReferenceSkeleton.Bones.size()))
			{
				FSkeletalStaticChildMesh Child;
				Child.SourceNodeName = Node->GetName() ? FString(Node->GetName()) : FString();
				Child.ParentBoneIndex = ParentBoneIndex;
				Child.ParentBoneName = ReferenceSkeleton.Bones[ParentBoneIndex].Name;
				Child.LocalMatrixToParentBone = GlobalReferenceMatrix * ReferenceSkeleton.Bones[ParentBoneIndex].GlobalBindPose.GetInverse();
				Child.ImportAction = ESkeletalStaticChildImportAction::MergeAsRigidPart;
				OutStaticChildMeshes.push_back(Child);
			}
			else
			{
				FFbxSplitStaticMeshReference Split;
				Split.SourceNodeName = Node->GetName() ? FString(Node->GetName()) : FString();
				Split.GlobalMatrix = GlobalReferenceMatrix;
				OutSplitStaticMeshes.push_back(Split);
			}
		}
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
		Context.ReferenceSkeleton.Bones.clear();
		Context.ReferenceSkeleton.Bones.reserve(Context.Bones.size());
		for (const FBone& Bone : Context.Bones)
		{
			FReferenceBone RefBone;
			RefBone.Name            = Bone.Name;
			RefBone.ParentIndex     = Bone.ParentIndex;
			RefBone.LocalBindPose   = Bone.LocalMatrix;
			RefBone.GlobalBindPose  = Bone.GlobalMatrix;
			RefBone.InverseBindPose = Bone.InverseBindPoseMatrix;
			Context.ReferenceSkeleton.Bones.push_back(RefBone);
		}

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
	return true;
}

bool FFbxSkeletalMeshImporter::ImportMeshOnly(FbxScene* Scene, FFbxImportContext& Context, FFbxSkeletalMeshOnlyImportResult& OutResult, FString* OutMessage)
{
	OutResult = FFbxSkeletalMeshOnlyImportResult();
	return ImportMeshCore(Scene, Context, OutResult.Mesh, OutResult.Materials, OutResult.SourceSkeleton, OutResult.SourceMaterials, OutMessage);
}
