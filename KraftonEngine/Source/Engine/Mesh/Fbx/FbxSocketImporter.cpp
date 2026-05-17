#include "Mesh/Fbx/FbxSocketImporter.h"

#include "Mesh/Fbx/FbxImportContext.h"
#include "Mesh/Fbx/FbxSceneQuery.h"
#include "Mesh/Fbx/FbxTransformUtils.h"

namespace
{
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

	static FMatrix GetSocketSourceGlobalMatrix(FbxScene* Scene, FbxNode* SocketNode)
	{
		FMatrix BindPoseMatrix;
		if (TryGetBindPoseMatrixForNode(Scene, SocketNode, BindPoseMatrix))
		{
			return BindPoseMatrix;
		}
		return SocketNode ? FFbxTransformUtils::ToEngineMatrix(SocketNode->EvaluateGlobalTransform()) : FMatrix::Identity;
	}

	static FMatrix ComputeSocketLocalMatrix(FbxScene* Scene, FbxNode* SocketNode, const FMatrix& ReferenceMeshBindInverse, const FReferenceSkeleton& ReferenceSkeleton, int32 ParentBoneIndex)
	{
		if (!SocketNode || ParentBoneIndex < 0 || ParentBoneIndex >= static_cast<int32>(ReferenceSkeleton.Bones.size()))
		{
			return FMatrix::Identity;
		}

		const FMatrix SocketGlobalReferenceSpace = GetSocketSourceGlobalMatrix(Scene, SocketNode) * ReferenceMeshBindInverse;
		const FMatrix& ParentBoneGlobalBindPose = ReferenceSkeleton.Bones[ParentBoneIndex].GlobalBindPose;
		return SocketGlobalReferenceSpace * ParentBoneGlobalBindPose.GetInverse();
	}

	static bool HasSocketName(const TArray<FSkeletalSocket>& Sockets, const FString& Name)
	{
		for (const FSkeletalSocket& Socket : Sockets)
		{
			if (Socket.Name == Name)
			{
				return true;
			}
		}
		return false;
	}

	static FString MakeUniqueSocketName(const TArray<FSkeletalSocket>& ExistingSockets, const FString& RequestedName)
	{
		const FString BaseName = RequestedName.empty() ? FString("Socket") : RequestedName;
		if (!HasSocketName(ExistingSockets, BaseName))
		{
			return BaseName;
		}

		for (int32 Suffix = 1; Suffix < 100000; ++Suffix)
		{
			const FString Candidate = BaseName + "_" + std::to_string(Suffix);
			if (!HasSocketName(ExistingSockets, Candidate))
			{
				return Candidate;
			}
		}

		return BaseName;
	}
}

void FFbxSocketImporter::ImportSockets(
	FbxScene*                    Scene,
	const TMap<FbxNode*, int32>& BoneNodeToIndex,
	const FMatrix&               ReferenceMeshBindInverse,
	const FReferenceSkeleton&    ReferenceSkeleton,
	TArray<FSkeletalSocket>&     OutSockets,
	FFbxImportContext&           BuildContext
	)
{
	OutSockets.clear();
	if (!Scene || !Scene->GetRootNode())
	{
		BuildContext.Summary.SocketCount = 0;
		return;
	}

	TArray<FbxNode*> SocketNodes;
	FFbxSceneQuery::CollectSocketNodes(Scene->GetRootNode(), SocketNodes);

	for (FbxNode* SocketNode : SocketNodes)
	{
		if (!SocketNode)
		{
			continue;
		}

		FbxNode* ParentBoneNode = nullptr;
		int32 ParentBoneIndex = -1;
		if (!FFbxSceneQuery::FindNearestParentBoneIndex(SocketNode, BoneNodeToIndex, ParentBoneNode, ParentBoneIndex))
		{
			BuildContext.AddWarning(ESkeletalImportWarningType::SocketWithoutParentBone, "Socket node has no imported parent bone and was skipped: " + FString(SocketNode->GetName()));
			continue;
		}

		if (ParentBoneIndex < 0 || ParentBoneIndex >= static_cast<int32>(ReferenceSkeleton.Bones.size()))
		{
			BuildContext.AddWarning(ESkeletalImportWarningType::SocketWithoutParentBone, "Socket node resolved invalid parent bone and was skipped: " + FString(SocketNode->GetName()));
			continue;
		}

		const FString RequestedSocketName = FFbxSceneQuery::GetSocketName(SocketNode);
		const FString UniqueSocketName = MakeUniqueSocketName(OutSockets, RequestedSocketName);
		if (UniqueSocketName != RequestedSocketName)
		{
			BuildContext.AddWarning(ESkeletalImportWarningType::DuplicateSocketName, "Duplicate socket name was renamed: " + RequestedSocketName + " -> " + UniqueSocketName);
		}

		FSkeletalSocket Socket;
		Socket.Name = UniqueSocketName;
		Socket.SourceNodeName = SocketNode->GetName();
		Socket.ParentBoneIndex = ParentBoneIndex;
		Socket.ParentBoneName = ReferenceSkeleton.Bones[ParentBoneIndex].Name;
		Socket.LocalMatrixToParentBone = ComputeSocketLocalMatrix(Scene, SocketNode, ReferenceMeshBindInverse, ReferenceSkeleton, ParentBoneIndex);
		OutSockets.push_back(Socket);
	}

	BuildContext.Summary.SocketCount = static_cast<int32>(OutSockets.size());
}
