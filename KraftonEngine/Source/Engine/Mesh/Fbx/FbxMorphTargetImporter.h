#pragma once

#include "Core/CoreTypes.h"

#include <fbxsdk.h>
#include "Math/Matrix.h"
#include "Mesh/SkeletalMeshAsset.h"

struct FFbxImportedMorphSourceVertex
{
    FbxNode* MeshNode = nullptr;
    FbxMesh* Mesh     = nullptr;

    FString SourceMeshNodeName;
    
    int32    ControlPointIndex  = -1;
    int32    PolygonIndex       = -1;
    int32    CornerIndex        = -1;
    int32    PolygonVertexIndex = -1;
    
    uint32   VertexIndex        = 0;
    
    FMatrix  MeshToReference;
    FMatrix  NormalToReference;
    
    FVector  BaseNormalInReference;
    FVector4 BaseTangentInReference;
};

struct FFbxImportContext;

class FFbxMorphTargetImporter
{
public:
    // LOD별 morph source vertex와 FBX blend shape을 엔진 morph target으로 변환한다.
    static void ImportMorphTargets(
        const TArray<TArray<FFbxImportedMorphSourceVertex>>& MorphSourcesByLOD,
        TArray<FMorphTarget>&                                OutMorphTargets,
        FFbxImportContext&                                   BuildContext
        );
};
