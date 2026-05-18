#pragma once

#include "Core/CoreTypes.h"

#include <fbxsdk.h>

class FFbxSceneQuery
{
public:
	static void CollectAllNodes(FbxNode* RootNode, TArray<FbxNode*>& OutNodes);
	static void CollectMeshNodes(FbxNode* RootNode, TArray<FbxNode*>& OutMeshNodes);
	static bool IsSkeletonNode(FbxNode* Node);
	static bool MeshHasSkin(FbxMesh* Mesh);
	static bool SceneHasSkinDeformer(FbxScene* Scene);
	static bool IsValidControlPointIndex(const FbxMesh* Mesh, int32 ControlPointIndex);

	static bool    ContainsNode(const TArray<FbxNode*>& Nodes, const FbxNode* Node);
	static void    AddUniqueNode(TArray<FbxNode*>& Nodes, FbxNode* Node);
	static bool    IsSceneRootNode(FbxNode* Node);
	static FString ReadStringProperty(FbxNode* Node, const char* PropertyName);

	static void CollectSkinClusterLinksFromMesh(FbxMesh* Mesh, TArray<FbxNode*>& OutClusterNodes);
	static FbxNode* FindNearestParentSkeletonNode(FbxNode* MeshNode);
	static void AddNodeAndParentsUntilSceneRoot(FbxNode* Node, TArray<FbxNode*>& OutNodes);
	static void FindImportedBoneRoot(const TArray<FbxNode*>& Nodes, TArray<FbxNode*>& OutRoots);
	static void CollectFullSkeletonHierarchyFromRoots(const TArray<FbxNode*>& RootNodes, const TArray<FbxNode*>& SeedNodes, TArray<FbxNode*>& OutBoneNodes);

	static int32 ParseLODIndexFromName(const FString& Name);
	static int32 GetMeshLODIndex(FbxNode* MeshNode);
	static bool  TryGetFloatProperty(FbxNode* Node, const char* PropertyName, float& OutValue);
	static bool  TryGetLODSettings(FbxNode* MeshNode, float& OutScreenSize, float& OutDistanceThreshold);

	static bool IsCollisionProxyName(const FString& Name);
	static bool IsCollisionProxyNode(FbxNode* Node);

	static bool    IsSocketName(const FString& Name);
	static bool    IsSocketNode(FbxNode* Node);
	static FString GetSocketName(FbxNode* Node);
	static void    CollectSocketNodes(FbxNode* Node, TArray<FbxNode*>& OutSocketNodes);

	static bool FindNearestParentBoneIndex(FbxNode* Node, const TMap<FbxNode*, int32>& BoneNodeToIndex, FbxNode*& OutBoneNode, int32& OutBoneIndex);
};
