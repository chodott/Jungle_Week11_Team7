#pragma once

#include "Core/CoreTypes.h"
#include "Mesh/StaticMeshAsset.h"

#include <fbxsdk.h>

class FFbxMetadataImporter
{
public:
    static void CollectSceneNodeMetadata(FbxScene* Scene, TArray<FFbxImportedNodeMetadata>& OutMetadata);
    static void CollectNodeMetadata(FbxNode* Node, TArray<FFbxImportedNodeMetadata>& OutMetadata);
    static void CollectObjectMetadata(FbxObject* Object, const FString& SourceName, const FString& NodePath, TArray<FFbxImportedNodeMetadata>& OutMetadata);
    static TArray<FFbxImportedMetadataValue> ExtractMetadataValues(FbxObject* Object);
};
