#include "Mesh/Fbx/FbxSkeletonImporter.h"
#include "Mesh/Fbx/FbxSceneQuery.h"
#include "Mesh/Fbx/FbxTransformUtils.h"

#include <algorithm>
#include <limits>

namespace
{
	static bool IsValidBoneIndex(const FFbxImportContext& Context, int32 BoneIndex)
	{
		return BoneIndex >= 0 && BoneIndex < static_cast<int32>(Context.Bones.size());
	}

	static void BuildReferenceSkeleton(FFbxImportContext& Context)
	{
		Context.ReferenceSkeleton.Bones.clear();
		Context.ReferenceSkeleton.Bones.reserve(Context.Bones.size());

		for (const FBone& Bone : Context.Bones)
		{
			FReferenceBone RefBone;
			RefBone.Name = Bone.Name;
			RefBone.ParentIndex = Bone.ParentIndex;
			RefBone.LocalBindPose = Bone.LocalMatrix;
			RefBone.GlobalBindPose = Bone.GlobalMatrix;
			RefBone.InverseBindPose = Bone.InverseBindPoseMatrix;
			Context.ReferenceSkeleton.Bones.push_back(RefBone);
		}
	}

	static bool TryGetBindPoseMatrixForNode(FbxScene* Scene, FbxNode* Node, FMatrix& OutPoseMatrix)
	{
		if (!Scene || !Node)
		{
			return false;
		}

		const int32 PoseCount = Scene->GetPoseCount();
		for (int32 PoseIndex = 0; PoseIndex < PoseCount; ++PoseIndex)
		{
			FbxPose* Pose = Scene->GetPose(PoseIndex);
			if (!Pose || !Pose->IsBindPose())
			{
				continue;
			}

			const int32 NodeIndex = Pose->Find(Node);
			if (NodeIndex < 0)
			{
				continue;
			}

			OutPoseMatrix = FFbxTransformUtils::ToEngineMatrix(Pose->GetMatrix(NodeIndex));
			return true;
		}

		return false;
	}

	static FMatrix GetSceneGlobalMatrix(FbxNode* Node)
	{
		return Node ? FFbxTransformUtils::ToEngineMatrix(Node->EvaluateGlobalTransform()) : FMatrix::Identity;
	}

	static bool TryGetBindPoseGlobalMatrix(FbxScene* Scene, FbxNode* Node, FMatrix& OutGlobalMatrix)
	{
		return TryGetBindPoseMatrixForNode(Scene, Node, OutGlobalMatrix);
	}

	static FMatrix GetSceneLocalMatrixFromGlobals(FbxNode* ChildNode, FbxNode* ParentNode)
	{
		if (!ChildNode)
		{
			return FMatrix::Identity;
		}

		const FMatrix ChildGlobal = GetSceneGlobalMatrix(ChildNode);
		if (!ParentNode || FFbxSceneQuery::IsSceneRootNode(ParentNode))
		{
			return ChildGlobal;
		}

		const FMatrix ParentGlobal = GetSceneGlobalMatrix(ParentNode);
		return ChildGlobal * ParentGlobal.GetInverse();
	}

	static bool TryGetSourceLocalBindMatrix(
		FbxScene* Scene,
		FbxNode*  ChildNode,
		FbxNode*  ParentNode,
		FMatrix&  OutLocalMatrix
		)
	{
		if (!ChildNode)
		{
			return false;
		}

		FMatrix ChildPoseGlobal;
		if (!TryGetBindPoseGlobalMatrix(Scene, ChildNode, ChildPoseGlobal))
		{
			return false;
		}

		if (!ParentNode || FFbxSceneQuery::IsSceneRootNode(ParentNode))
		{
			OutLocalMatrix = ChildPoseGlobal;
			return true;
		}

		FMatrix ParentPoseGlobal;
		if (!TryGetBindPoseGlobalMatrix(Scene, ParentNode, ParentPoseGlobal))
		{
			return false;
		}

		OutLocalMatrix = ChildPoseGlobal * ParentPoseGlobal.GetInverse();
		return true;
	}

	static FMatrix GetBestSourceLocalBindMatrix(
		FbxScene* Scene,
		FbxNode*  ChildNode,
		FbxNode*  ParentNode,
		bool&     bUsedSceneFallback
		)
	{
		FMatrix LocalMatrix;
		if (TryGetSourceLocalBindMatrix(Scene, ChildNode, ParentNode, LocalMatrix))
		{
			return LocalMatrix;
		}

		bUsedSceneFallback = true;
		return GetSceneLocalMatrixFromGlobals(ChildNode, ParentNode);
	}

	static bool IsBlenderArmatureRootToStrip(FbxNode* Node, const TArray<FbxNode*>& ClusterLinkNodes)
	{
		if (!Node || FFbxSceneQuery::IsSceneRootNode(Node) || !FFbxSceneQuery::IsSkeletonNode(Node))
		{
			return false;
		}

		if (FFbxSceneQuery::ContainsNode(ClusterLinkNodes, Node))
		{
			return false;
		}

		FbxNode* Parent = Node->GetParent();
		if (!Parent || !FFbxSceneQuery::IsSceneRootNode(Parent))
		{
			return false;
		}

		const FString NodeName = Node->GetName() ? FString(Node->GetName()) : FString();
		if (NodeName != "Armature")
		{
			return false;
		}

		for (int32 ChildIndex = 0; ChildIndex < Node->GetChildCount(); ++ChildIndex)
		{
			if (FFbxSceneQuery::IsSkeletonNode(Node->GetChild(ChildIndex)))
			{
				return true;
			}
		}

		return false;
	}

	static int32 StripBlenderArmatureRoots(TArray<FbxNode*>& InOutBoneNodes, const TArray<FbxNode*>& ClusterLinkNodes)
	{
		const size_t OriginalCount = InOutBoneNodes.size();
		InOutBoneNodes.erase(
			std::remove_if(
				InOutBoneNodes.begin(),
				InOutBoneNodes.end(),
				[&](FbxNode* Node)
				{
					return IsBlenderArmatureRootToStrip(Node, ClusterLinkNodes);
				}
			),
			InOutBoneNodes.end()
		);
		return static_cast<int32>(OriginalCount - InOutBoneNodes.size());
	}

	static int32 AddImportedBoneRecursive(
		FbxNode*                BoneNode,
		int32                   ParentIndex,
		const TArray<FbxNode*>& ImportedBoneNodes,
		FFbxImportContext&      Context
		)
	{
		if (!BoneNode || !FFbxSceneQuery::ContainsNode(ImportedBoneNodes, BoneNode))
		{
			return -1;
		}

		auto Existing = Context.BoneNodeToIndex.find(BoneNode);
		if (Existing != Context.BoneNodeToIndex.end())
		{
			return Existing->second;
		}

		FBone Bone;
		Bone.Name         = BoneNode->GetName() ? FString(BoneNode->GetName()) : FString();
		Bone.ParentIndex  = ParentIndex;
		Bone.LocalMatrix  = FFbxTransformUtils::ToEngineMatrix(BoneNode->EvaluateLocalTransform());
		Bone.GlobalMatrix = FFbxTransformUtils::ToEngineMatrix(BoneNode->EvaluateGlobalTransform()) * Context.
		ReferenceMeshBindInverse;
		Bone.InverseBindPoseMatrix = Bone.GlobalMatrix.GetInverse();

		const int32 BoneIndex = static_cast<int32>(Context.Bones.size());
		Context.Bones.push_back(Bone);
		Context.BoneNodeToIndex[BoneNode] = BoneIndex;

		for (int32 ChildIndex = 0; ChildIndex < BoneNode->GetChildCount(); ++ChildIndex)
		{
			FbxNode* Child = BoneNode->GetChild(ChildIndex);
			if (FFbxSceneQuery::ContainsNode(ImportedBoneNodes, Child))
			{
				AddImportedBoneRecursive(Child, BoneIndex, ImportedBoneNodes, Context);
			}
		}

		return BoneIndex;
	}

	static bool BuildSkeletonFromSkinClusters(FbxScene* Scene, FFbxImportContext& Context)
	{
		TArray<FbxNode*> LinkNodes;
		for (FbxNode* MeshNode : Context.MeshNodes)
		{
			FbxMesh* Mesh = MeshNode ? MeshNode->GetMesh() : nullptr;
			if (Mesh && FFbxSceneQuery::MeshHasSkin(Mesh))
			{
				FFbxSceneQuery::CollectSkinClusterLinksFromMesh(Mesh, LinkNodes);
			}
		}

		if (LinkNodes.empty())
		{
			return false;
		}

		TArray<FbxNode*> ImportedBoneNodes;
		for (FbxNode* LinkNode : LinkNodes)
		{
			FFbxSceneQuery::AddNodeAndParentsUntilSceneRoot(LinkNode, ImportedBoneNodes);
		}

		for (FbxNode* MeshNode : Context.MeshNodes)
		{
			FbxMesh* Mesh = MeshNode ? MeshNode->GetMesh() : nullptr;
			if (!Mesh || FFbxSceneQuery::MeshHasSkin(Mesh))
			{
				continue;
			}

			FbxNode* RigidParentBone = FFbxSceneQuery::FindNearestParentSkeletonNode(MeshNode);
			if (RigidParentBone)
			{
				FFbxSceneQuery::AddNodeAndParentsUntilSceneRoot(RigidParentBone, ImportedBoneNodes);
			}
		}

		const int32 StrippedRootCount = StripBlenderArmatureRoots(ImportedBoneNodes, LinkNodes);
		if (StrippedRootCount > 0)
		{
			Context.AddWarningOnce(
				ESkeletalImportWarningType::UsedClusterBindPoseFallback,
				"Blender Armature root was treated as an import container and stripped from the runtime skeleton."
			);
		}

		TArray<FbxNode*> RootBones;
		FFbxSceneQuery::FindImportedBoneRoot(ImportedBoneNodes, RootBones);

		TArray<FbxNode*> FullBoneNodes;
		FFbxSceneQuery::CollectFullSkeletonHierarchyFromRoots(RootBones, ImportedBoneNodes, FullBoneNodes);
		if (FullBoneNodes.empty())
		{
			FullBoneNodes = ImportedBoneNodes;
		}

		TArray<FbxNode*> FullRootBones;
		FFbxSceneQuery::FindImportedBoneRoot(FullBoneNodes, FullRootBones);

		for (FbxNode* RootBone : FullRootBones)
		{
			AddImportedBoneRecursive(RootBone, -1, FullBoneNodes, Context);
		}

		return !Context.Bones.empty();
	}

	static bool BuildSkeletonFromSceneSkeletonNodes(FFbxImportContext& Context)
	{
		TArray<FbxNode*> SkeletonNodes;
		for (FbxNode* Node : Context.AllNodes)
		{
			if (FFbxSceneQuery::IsSkeletonNode(Node))
			{
				SkeletonNodes.push_back(Node);
			}
		}

		if (SkeletonNodes.empty())
		{
			return false;
		}

		TArray<FbxNode*> ImportedBoneNodes;
		for (FbxNode* SkeletonNode : SkeletonNodes)
		{
			FFbxSceneQuery::AddNodeAndParentsUntilSceneRoot(SkeletonNode, ImportedBoneNodes);
		}

		const int32 StrippedRootCount = StripBlenderArmatureRoots(ImportedBoneNodes, SkeletonNodes);
		if (StrippedRootCount > 0)
		{
			Context.AddWarningOnce(
				ESkeletalImportWarningType::UsedClusterBindPoseFallback,
				"Blender Armature root was treated as an import container and stripped from the runtime skeleton."
			);
		}

		TArray<FbxNode*> RootBones;
		FFbxSceneQuery::FindImportedBoneRoot(ImportedBoneNodes, RootBones);

		TArray<FbxNode*> FullBoneNodes;
		FFbxSceneQuery::CollectFullSkeletonHierarchyFromRoots(RootBones, ImportedBoneNodes, FullBoneNodes);
		if (FullBoneNodes.empty())
		{
			FullBoneNodes = ImportedBoneNodes;
		}

		TArray<FbxNode*> FullRootBones;
		FFbxSceneQuery::FindImportedBoneRoot(FullBoneNodes, FullRootBones);

		for (FbxNode* RootBone : FullRootBones)
		{
			AddImportedBoneRecursive(RootBone, -1, FullBoneNodes, Context);
		}

		return !Context.Bones.empty();
	}

	static TArray<FbxNode*> BuildBoneNodesByIndex(const FFbxImportContext& Context)
	{
		TArray<FbxNode*> BoneNodesByIndex;
		BoneNodesByIndex.resize(Context.Bones.size(), nullptr);

		for (const auto& Pair : Context.BoneNodeToIndex)
		{
			FbxNode*    BoneNode  = Pair.first;
			const int32 BoneIndex = Pair.second;
			if (BoneNode && IsValidBoneIndex(Context, BoneIndex))
			{
				BoneNodesByIndex[BoneIndex] = BoneNode;
			}
		}

		return BoneNodesByIndex;
	}

	static void InitializeBoneBindPoseFromSceneNodes(FFbxImportContext& Context)
	{
		for (const auto& Pair : Context.BoneNodeToIndex)
		{
			FbxNode*    BoneNode  = Pair.first;
			const int32 BoneIndex = Pair.second;
			if (!BoneNode || !IsValidBoneIndex(Context, BoneIndex))
			{
				continue;
			}

			const FMatrix BoneGlobal = FFbxTransformUtils::ToEngineMatrix(BoneNode->EvaluateGlobalTransform());
			Context.Bones[BoneIndex].GlobalMatrix = BoneGlobal * Context.ReferenceMeshBindInverse;
			Context.Bones[BoneIndex].InverseBindPoseMatrix = Context.Bones[BoneIndex].GlobalMatrix.GetInverse();
		}
	}

	static void ApplyBindPoseFromFbxPose(
		FbxScene*          Scene,
		FFbxImportContext& Context,
		TArray<bool>&      InOutAppliedBoneMask
		)
	{
		if (!Scene)
		{
			return;
		}

		bool bFoundAnyBindPose = false;
		for (const auto& Pair : Context.BoneNodeToIndex)
		{
			FbxNode*    BoneNode  = Pair.first;
			const int32 BoneIndex = Pair.second;
			if (!BoneNode || !IsValidBoneIndex(Context, BoneIndex))
			{
				continue;
			}

			FMatrix BonePoseMatrix;
			if (!TryGetBindPoseMatrixForNode(Scene, BoneNode, BonePoseMatrix))
			{
				continue;
			}

			bFoundAnyBindPose                              = true;
			Context.Bones[BoneIndex].GlobalMatrix          = BonePoseMatrix * Context.ReferenceMeshBindInverse;
			Context.Bones[BoneIndex].InverseBindPoseMatrix = Context.Bones[BoneIndex].GlobalMatrix.GetInverse();

			if (BoneIndex < static_cast<int32>(InOutAppliedBoneMask.size()))
			{
				InOutAppliedBoneMask[BoneIndex] = true;
			}
		}

		if (!bFoundAnyBindPose)
		{
			Context.AddWarningOnce(
				ESkeletalImportWarningType::MissingBindPose,
				"No explicit FBX bind pose was found. Falling back to skin cluster bind matrices."
			);
		}
	}

	static void ApplyBindPoseFromSkinClusters(
		const TArray<FbxNode*>& MeshNodes,
		FFbxImportContext&      Context,
		TArray<bool>*           InOutAppliedBoneMask,
		bool                    bOverrideExisting
		)
	{
		for (FbxNode* MeshNode : MeshNodes)
		{
			FbxMesh* Mesh = MeshNode ? MeshNode->GetMesh() : nullptr;
			if (!Mesh)
			{
				continue;
			}

			const int32 SkinCount = Mesh->GetDeformerCount(FbxDeformer::eSkin);
			for (int32 SkinIndex = 0; SkinIndex < SkinCount; ++SkinIndex)
			{
				FbxSkin* Skin = static_cast<FbxSkin*>(Mesh->GetDeformer(SkinIndex, FbxDeformer::eSkin));
				if (!Skin)
				{
					continue;
				}

				const int32 ClusterCount = Skin->GetClusterCount();
				for (int32 ClusterIndex = 0; ClusterIndex < ClusterCount; ++ClusterIndex)
				{
					FbxCluster* Cluster = Skin->GetCluster(ClusterIndex);
					if (!Cluster || !Cluster->GetLink())
					{
						continue;
					}

					auto BoneIt = Context.BoneNodeToIndex.find(Cluster->GetLink());
					if (BoneIt == Context.BoneNodeToIndex.end())
					{
						continue;
					}

					const int32 BoneIndex = BoneIt->second;
					if (!IsValidBoneIndex(Context, BoneIndex))
					{
						continue;
					}

					if (!bOverrideExisting && InOutAppliedBoneMask && BoneIndex < static_cast<int32>(
						InOutAppliedBoneMask->size()) && (*InOutAppliedBoneMask)[BoneIndex])
					{
						continue;
					}

					FbxAMatrix LinkBindFbx;
					Cluster->GetTransformLinkMatrix(LinkBindFbx);
					const FMatrix LinkBind                         = FFbxTransformUtils::ToEngineMatrix(LinkBindFbx);
					Context.Bones[BoneIndex].GlobalMatrix          = LinkBind * Context.ReferenceMeshBindInverse;
					Context.Bones[BoneIndex].InverseBindPoseMatrix = Context.Bones[BoneIndex].GlobalMatrix.GetInverse();

					if (InOutAppliedBoneMask && BoneIndex < static_cast<int32>(InOutAppliedBoneMask->size()))
					{
						(*InOutAppliedBoneMask)[BoneIndex] = true;
					}
				}
			}
		}
	}

	static void FinalizeNonClusterBoneBindPose(
		FbxScene*           Scene,
		const TArray<bool>& ClusterAppliedBoneMask,
		FFbxImportContext&  Context
		)
	{
		const TArray<FbxNode*> BoneNodesByIndex       = BuildBoneNodesByIndex(Context);
		int32                  FinalizedBoneCount     = 0;
		int32                  SceneFallbackBoneCount = 0;

		for (int32 BoneIndex = 0; BoneIndex < static_cast<int32>(Context.Bones.size()); ++BoneIndex)
		{
			const bool bClusterApplied = BoneIndex < static_cast<int32>(ClusterAppliedBoneMask.size()) &&
			ClusterAppliedBoneMask[BoneIndex];
			if (bClusterApplied)
			{
				Context.Bones[BoneIndex].InverseBindPoseMatrix = Context.Bones[BoneIndex].GlobalMatrix.GetInverse();
				continue;
			}

			FbxNode* BoneNode = BoneIndex < static_cast<int32>(BoneNodesByIndex.size()) ? BoneNodesByIndex[BoneIndex]
			: nullptr;
			if (!BoneNode)
			{
				continue;
			}

			FBone& Bone               = Context.Bones[BoneIndex];
			bool   bUsedSceneFallback = false;

			if (IsValidBoneIndex(Context, Bone.ParentIndex))
			{
				FbxNode* ParentNode = Bone.ParentIndex < static_cast<int32>(BoneNodesByIndex.size())
				? BoneNodesByIndex[Bone.ParentIndex] : nullptr;
				Bone.LocalMatrix  = GetBestSourceLocalBindMatrix(Scene, BoneNode, ParentNode, bUsedSceneFallback);
				Bone.GlobalMatrix = Bone.LocalMatrix * Context.Bones[Bone.ParentIndex].GlobalMatrix;
			}
			else
			{
				FMatrix RootPoseGlobal;
				if (TryGetBindPoseGlobalMatrix(Scene, BoneNode, RootPoseGlobal))
				{
					Bone.GlobalMatrix = RootPoseGlobal * Context.ReferenceMeshBindInverse;
				}
				else
				{
					bUsedSceneFallback = true;
					Bone.GlobalMatrix  = GetSceneGlobalMatrix(BoneNode) * Context.ReferenceMeshBindInverse;
				}
				Bone.LocalMatrix = Bone.GlobalMatrix;
			}

			Bone.InverseBindPoseMatrix = Bone.GlobalMatrix.GetInverse();
			++FinalizedBoneCount;
			if (bUsedSceneFallback)
			{
				++SceneFallbackBoneCount;
			}
		}

		if (FinalizedBoneCount > 0)
		{
			Context.AddWarningOnce(
				ESkeletalImportWarningType::UsedClusterBindPoseFallback,
				"One or more non-weighted bones were aligned to their parent bind pose hierarchy."
			);
		}
		if (SceneFallbackBoneCount > 0)
		{
			Context.AddWarningOnce(
				ESkeletalImportWarningType::UsedSceneTransformFallback,
				"One or more bones had no FBX bind pose. Scene transforms were used only as local hierarchy fallback."
			);
		}
	}

	static void RecomputeLocalBindPose(FFbxImportContext& Context)
	{
		for (int32 BoneIndex = 0; BoneIndex < static_cast<int32>(Context.Bones.size()); ++BoneIndex)
		{
			FBone& Bone = Context.Bones[BoneIndex];
			if (Bone.ParentIndex >= 0 && Bone.ParentIndex < static_cast<int32>(Context.Bones.size()))
			{
				const FMatrix ParentGlobal = Context.Bones[Bone.ParentIndex].GlobalMatrix;
				Bone.LocalMatrix           = Bone.GlobalMatrix * ParentGlobal.GetInverse();
			}
			else
			{
				Bone.LocalMatrix = Bone.GlobalMatrix;
			}
			Bone.InverseBindPoseMatrix = Bone.GlobalMatrix.GetInverse();
		}
	}

	static TArray<FbxNode*> CollectSkinnedReferenceLODNodes(const FFbxImportContext& Context)
	{
		TArray<FbxNode*> Result;
		int32            BaseLODIndex = std::numeric_limits<int32>::max();
		for (FbxNode* MeshNode : Context.MeshNodes)
		{
			FbxMesh* Mesh = MeshNode ? MeshNode->GetMesh() : nullptr;
			if (!Mesh || FFbxSceneQuery::IsCollisionProxyNode(MeshNode) || !FFbxSceneQuery::MeshHasSkin(Mesh))
			{
				continue;
			}

			const int32 LODIndex = FFbxSceneQuery::GetMeshLODIndex(MeshNode);
			if (LODIndex < BaseLODIndex)
			{
				BaseLODIndex = LODIndex;
				Result.clear();
			}
			if (LODIndex == BaseLODIndex)
			{
				Result.push_back(MeshNode);
			}
		}
		return Result;
	}
}

bool FFbxSkeletonImporter::TryGetFirstMeshBindMatrix(FbxScene* Scene, FbxNode* MeshNode, FMatrix& OutMeshBindMatrix)
{
	FbxMesh* Mesh = MeshNode ? MeshNode->GetMesh() : nullptr;
	if (!Mesh)
	{
		return false;
	}

	const FMatrix GeometryTransform = FFbxTransformUtils::ToEngineMatrix(
		FFbxTransformUtils::GetNodeGeometryTransform(MeshNode)
	);
	const int32 SkinCount = Mesh->GetDeformerCount(FbxDeformer::eSkin);
	for (int32 SkinIndex = 0; SkinIndex < SkinCount; ++SkinIndex)
	{
		FbxSkin* Skin = static_cast<FbxSkin*>(Mesh->GetDeformer(SkinIndex, FbxDeformer::eSkin));
		if (!Skin)
		{
			continue;
		}

		const int32 ClusterCount = Skin->GetClusterCount();
		for (int32 ClusterIndex = 0; ClusterIndex < ClusterCount; ++ClusterIndex)
		{
			FbxCluster* Cluster = Skin->GetCluster(ClusterIndex);
			if (!Cluster)
			{
				continue;
			}

			FbxAMatrix MeshNodeBindFbx;
			Cluster->GetTransformMatrix(MeshNodeBindFbx);
			const FMatrix MeshNodeBind = FFbxTransformUtils::ToEngineMatrix(MeshNodeBindFbx);
			OutMeshBindMatrix          = MeshNodeBind; // Cluster transform matrix is the mesh node global bind matrix.
			OutMeshBindMatrix          = GeometryTransform * OutMeshBindMatrix;
			return true;
		}
	}

	FMatrix PoseMatrix;
	if (TryGetBindPoseMatrixForNode(Scene, MeshNode, PoseMatrix))
	{
		OutMeshBindMatrix = GeometryTransform * PoseMatrix;
		return true;
	}

	return false;
}

bool FFbxSkeletonImporter::TryGetReferenceMeshBindMatrix(
	FbxScene*               Scene,
	const TArray<FbxNode*>& SkinnedMeshNodes,
	FMatrix&                OutReferenceMeshBindMatrix
	)
{
	for (FbxNode* MeshNode : SkinnedMeshNodes)
	{
		if (TryGetFirstMeshBindMatrix(Scene, MeshNode, OutReferenceMeshBindMatrix))
		{
			return true;
		}
	}
	return false;
}

bool FFbxSkeletonImporter::ImportSkeleton(FbxScene* Scene, FFbxImportContext& Context, FString* OutMessage)
{
	Context.Bones.clear();
	Context.BoneNodeToIndex.clear();
	Context.ReferenceSkeleton.Bones.clear();
	Context.ReferenceMeshBind        = FMatrix::Identity;
	Context.ReferenceMeshBindInverse = FMatrix::Identity;
	Context.bHasReferenceMeshBind    = false;

	const TArray<FbxNode*> ReferenceLODNodes = CollectSkinnedReferenceLODNodes(Context);
	FMatrix                ReferenceMeshBind;
	if (TryGetReferenceMeshBindMatrix(Scene, ReferenceLODNodes, ReferenceMeshBind))
	{
		Context.ReferenceMeshBind        = ReferenceMeshBind;
		Context.ReferenceMeshBindInverse = ReferenceMeshBind.GetInverse();
		Context.bHasReferenceMeshBind    = true;
	}
	else
	{
		Context.AddWarningOnce(
			ESkeletalImportWarningType::UsedSceneTransformFallback,
			"Reference mesh bind matrix was not found. Skeletal import used identity reference space."
		);
	}

	if (!BuildSkeletonFromSkinClusters(Scene, Context))
	{
		BuildSkeletonFromSceneSkeletonNodes(Context);
	}

	if (Context.Bones.empty())
	{
		if (OutMessage) *OutMessage = "FBX skeletal import failed: no skeleton nodes found.";
		return false;
	}

	InitializeBoneBindPoseFromSceneNodes(Context);

	TArray<bool> AppliedBindPoseMask(Context.Bones.size(), false);
	ApplyBindPoseFromFbxPose(Scene, Context, AppliedBindPoseMask);

	TArray<bool> ClusterAppliedBoneMask(Context.Bones.size(), false);
	ApplyBindPoseFromSkinClusters(ReferenceLODNodes, Context, &ClusterAppliedBoneMask, true);
	ApplyBindPoseFromSkinClusters(Context.MeshNodes, Context, &ClusterAppliedBoneMask, false);

	FinalizeNonClusterBoneBindPose(Scene, ClusterAppliedBoneMask, Context);
	RecomputeLocalBindPose(Context);
	BuildReferenceSkeleton(Context);
	return true;
}

int32 FFbxSkeletonImporter::FindNearestParentBoneIndex(FbxNode* Node, const TMap<FbxNode*, int32>& NodeToIndex)
{
	FbxNode* Parent = Node ? Node->GetParent() : nullptr;
	while (Parent)
	{
		auto It = NodeToIndex.find(Parent);
		if (It != NodeToIndex.end())
		{
			return It->second;
		}
		Parent = Parent->GetParent();
	}
	return -1;
}
