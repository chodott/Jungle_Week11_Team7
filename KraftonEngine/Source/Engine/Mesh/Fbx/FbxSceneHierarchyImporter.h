#pragma once

#include "Core/CoreTypes.h"
#include "Mesh/StaticMeshAsset.h"

#include <fbxsdk.h>

class FFbxSceneHierarchyImporter
{
public:
    static void CollectSceneNodes(FbxScene* Scene, TArray<FFbxImportedSceneNode>& OutSceneNodes);
};
