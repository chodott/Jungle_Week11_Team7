#include "Mesh/Fbx/FbxSceneHierarchyImporter.h"

#include "Mesh/Fbx/FbxSceneQuery.h"
#include "Mesh/Fbx/FbxTransformUtils.h"

namespace
{
    static FString GetAttributeTypeName(FbxNode* Node)
    {
        if (!Node || !Node->GetNodeAttribute())
        {
            return "None";
        }

        switch (Node->GetNodeAttribute()->GetAttributeType())
        {
        case FbxNodeAttribute::eMesh:
            return "Mesh";
        case FbxNodeAttribute::eSkeleton:
            return "Skeleton";
        case FbxNodeAttribute::eNull:
            return "Null";
        case FbxNodeAttribute::eMarker:
            return "Marker";
        case FbxNodeAttribute::eLODGroup:
            return "LODGroup";
        case FbxNodeAttribute::eCamera:
            return "Camera";
        case FbxNodeAttribute::eLight:
            return "Light";
        default:
            return "Other";
        }
    }

    static FString BuildNodePath(const FString& ParentPath, FbxNode* Node)
    {
        const FString Name = Node && Node->GetName() ? FString(Node->GetName()) : FString();
        if (ParentPath.empty())
        {
            return Name;
        }
        if (Name.empty())
        {
            return ParentPath;
        }
        return ParentPath + "/" + Name;
    }

    static void CollectRecursive(FbxNode* Node, int32 ParentIndex, const FString& ParentPath, TArray<FFbxImportedSceneNode>& OutSceneNodes)
    {
        if (!Node)
        {
            return;
        }

        FFbxImportedSceneNode ImportedNode;
        ImportedNode.Name                 = Node->GetName() ? FString(Node->GetName()) : FString();
        ImportedNode.NodePath             = BuildNodePath(ParentPath, Node);
        ImportedNode.AttributeType        = GetAttributeTypeName(Node);
        ImportedNode.ParentIndex          = ParentIndex;
        ImportedNode.SourceLODIndex       = Node->GetMesh() ? FFbxSceneQuery::GetMeshLODIndex(Node) : 0;
        ImportedNode.bHasMesh             = Node->GetMesh() != nullptr;
        ImportedNode.bHasSkeleton         = FFbxSceneQuery::IsSkeletonNode(Node);
        ImportedNode.bIsSocket            = FFbxSceneQuery::IsSocketNode(Node);
        ImportedNode.bIsCollision         = FFbxSceneQuery::IsCollisionProxyNode(Node);
        ImportedNode.LocalMatrix          = FFbxTransformUtils::ToEngineMatrix(Node->EvaluateLocalTransform());
        ImportedNode.GlobalMatrix         = FFbxTransformUtils::ToEngineMatrix(Node->EvaluateGlobalTransform());
        ImportedNode.GeometryMatrix       = FFbxTransformUtils::ToEngineMatrix(FFbxTransformUtils::GetNodeGeometryTransform(Node));
        ImportedNode.GlobalGeometryMatrix = ImportedNode.GeometryMatrix * ImportedNode.GlobalMatrix;

        const int32 ThisIndex = static_cast<int32>(OutSceneNodes.size());
        OutSceneNodes.push_back(ImportedNode);

        for (int32 ChildIndex = 0; ChildIndex < Node->GetChildCount(); ++ChildIndex)
        {
            CollectRecursive(Node->GetChild(ChildIndex), ThisIndex, ImportedNode.NodePath, OutSceneNodes);
        }
    }
}

void FFbxSceneHierarchyImporter::CollectSceneNodes(FbxScene* Scene, TArray<FFbxImportedSceneNode>& OutSceneNodes)
{
    OutSceneNodes.clear();
    if (!Scene || !Scene->GetRootNode())
    {
        return;
    }

    CollectRecursive(Scene->GetRootNode(), -1, FString(), OutSceneNodes);
}
