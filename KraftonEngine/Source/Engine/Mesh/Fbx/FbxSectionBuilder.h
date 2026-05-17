#pragma once

#include "Core/CoreTypes.h"
#include "Mesh/StaticMeshAsset.h"
#include "Mesh/SkeletalMeshAsset.h"

#include <fbxsdk.h>

struct FFbxImportedSectionBuild
{
    int32          MaterialIndex = -1;
    TArray<uint32> Indices;
};

class FFbxSectionBuilder
{
public:
    // material index에 해당하는 임시 section을 찾거나 새로 추가한다.
    FFbxImportedSectionBuild* FindOrAddSection(int32 MaterialIndex);

    // Static mesh용 최종 section/index buffer 생성. 잘못된 material index는 None slot으로 보낸다.
    void BuildFinalStaticSections(
        const TArray<FString>& MaterialSlotNames,
        TArray<uint32>& OutIndices,
        TArray<FStaticMeshSection>& OutSections,
        bool* bOutNeedsNoneSlot = nullptr
        ) const;

    // Skeletal mesh는 여러 mesh node를 한 index buffer에 누적하므로 append 방식으로 붙인다.
    void AppendFinalSkeletalSections(
        const TArray<FString>& MaterialSlotNames,
        TArray<uint32>& InOutIndices,
        TArray<FSkeletalMeshSection>& InOutSections,
        bool* bOutNeedsNoneSlot = nullptr
        ) const;

    // 임시 section이 비어 있는지 확인한다.
    bool IsEmpty() const;

private:
    TArray<FFbxImportedSectionBuild> Sections;
};

class FFbxMaterialSlotResolver
{
public:
    // polygon index에 대응하는 FBX material layer index를 읽는다.
    static int32 GetPolygonMaterialIndex(FbxMesh* Mesh, int32 PolygonIndex);

    // polygon index에 대응하는 FbxSurfaceMaterial 포인터를 resolve한다.
    static FbxSurfaceMaterial* ResolvePolygonMaterial(FbxNode* MeshNode, FbxMesh* Mesh, int32 PolygonIndex);
};
