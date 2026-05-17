#include "Mesh/Fbx/FbxSectionBuilder.h"

namespace
{
    // mesh의 첫 material layer element를 찾는다.
    FbxLayerElementMaterial* FindFirstMaterialLayerElement(FbxMesh* Mesh)
    {
        if (!Mesh)
        {
            return nullptr;
        }

        const int32 LayerCount = Mesh->GetLayerCount();
        for (int32 LayerIndex = 0; LayerIndex < LayerCount; ++LayerIndex)
        {
            FbxLayer* Layer = Mesh->GetLayer(LayerIndex);
            if (!Layer)
            {
                continue;
            }

            FbxLayerElementMaterial* MaterialElement = Layer->GetMaterials();
            if (MaterialElement)
            {
                return MaterialElement;
            }
        }

        return nullptr;
    }

    FString ResolveMaterialSlotName(
        int32 MaterialIndex,
        const TArray<FString>& MaterialSlotNames,
        int32& OutMaterialIndex,
        bool* bOutNeedsNoneSlot
        )
    {
        if (MaterialIndex >= 0 && MaterialIndex < static_cast<int32>(MaterialSlotNames.size()))
        {
            OutMaterialIndex = MaterialIndex;
            return MaterialSlotNames[MaterialIndex];
        }

        OutMaterialIndex = -1;
        if (bOutNeedsNoneSlot)
        {
            *bOutNeedsNoneSlot = true;
        }
        return "None";
    }
}

// material index에 해당하는 임시 section을 찾거나 새로 추가한다.
FFbxImportedSectionBuild* FFbxSectionBuilder::FindOrAddSection(int32 MaterialIndex)
{
    for (FFbxImportedSectionBuild& Section : Sections)
    {
        if (Section.MaterialIndex == MaterialIndex)
        {
            return &Section;
        }
    }

    FFbxImportedSectionBuild NewSection;
    NewSection.MaterialIndex = MaterialIndex;
    Sections.push_back(NewSection);
    return &Sections.back();
}

void FFbxSectionBuilder::BuildFinalStaticSections(
    const TArray<FString>& MaterialSlotNames,
    TArray<uint32>& OutIndices,
    TArray<FStaticMeshSection>& OutSections,
    bool* bOutNeedsNoneSlot
    ) const
{
    OutIndices.clear();
    OutSections.clear();

    for (const FFbxImportedSectionBuild& BuildSection : Sections)
    {
        if (BuildSection.Indices.empty())
        {
            continue;
        }

        FStaticMeshSection Section;
        Section.MaterialSlotName = ResolveMaterialSlotName(BuildSection.MaterialIndex, MaterialSlotNames, Section.MaterialIndex, bOutNeedsNoneSlot);
        Section.FirstIndex = static_cast<uint32>(OutIndices.size());
        Section.NumTriangles = static_cast<uint32>(BuildSection.Indices.size() / 3);

        OutIndices.insert(OutIndices.end(), BuildSection.Indices.begin(), BuildSection.Indices.end());
        OutSections.push_back(Section);
    }
}

void FFbxSectionBuilder::AppendFinalSkeletalSections(
    const TArray<FString>& MaterialSlotNames,
    TArray<uint32>& InOutIndices,
    TArray<FSkeletalMeshSection>& InOutSections,
    bool* bOutNeedsNoneSlot
    ) const
{
    for (const FFbxImportedSectionBuild& BuildSection : Sections)
    {
        if (BuildSection.Indices.empty())
        {
            continue;
        }

        FSkeletalMeshSection Section;
        Section.MaterialSlotName = ResolveMaterialSlotName(BuildSection.MaterialIndex, MaterialSlotNames, Section.MaterialIndex, bOutNeedsNoneSlot);
        Section.FirstIndex = static_cast<uint32>(InOutIndices.size());
        Section.IndexCount = static_cast<uint32>(BuildSection.Indices.size());

        InOutIndices.insert(InOutIndices.end(), BuildSection.Indices.begin(), BuildSection.Indices.end());
        InOutSections.push_back(Section);
    }
}

// 임시 section이 비어 있는지 확인한다.
bool FFbxSectionBuilder::IsEmpty() const
{
    return Sections.empty();
}

int32 FFbxMaterialSlotResolver::GetPolygonMaterialIndex(FbxMesh* Mesh, int32 PolygonIndex)
{
    FbxLayerElementMaterial* MaterialElement = FindFirstMaterialLayerElement(Mesh);
    if (!MaterialElement)
    {
        return -1;
    }

    const FbxLayerElement::EMappingMode   MappingMode   = MaterialElement->GetMappingMode();
    const FbxLayerElement::EReferenceMode ReferenceMode = MaterialElement->GetReferenceMode();

    int32 ElementIndex = 0;

    switch (MappingMode)
    {
    case FbxLayerElement::eByPolygon:
        ElementIndex = PolygonIndex;
        break;

    case FbxLayerElement::eAllSame:
        ElementIndex = 0;
        break;

    default:
        return -1;
    }

    if (ElementIndex < 0)
    {
        return -1;
    }

    if (ReferenceMode == FbxLayerElement::eIndexToDirect || ReferenceMode == FbxLayerElement::eIndex)
    {
        if (ElementIndex >= 0 && ElementIndex < MaterialElement->GetIndexArray().GetCount())
        {
            return MaterialElement->GetIndexArray().GetAt(ElementIndex);
        }

        return -1;
    }

    return ElementIndex;
}

FbxSurfaceMaterial* FFbxMaterialSlotResolver::ResolvePolygonMaterial(FbxNode* MeshNode, FbxMesh* Mesh, int32 PolygonIndex)
{
    if (!MeshNode || !Mesh)
    {
        return nullptr;
    }

    const int32 LocalMaterialIndex = FFbxMaterialSlotResolver::GetPolygonMaterialIndex(Mesh, PolygonIndex);
    if (LocalMaterialIndex < 0 || LocalMaterialIndex >= MeshNode->GetMaterialCount())
    {
        return nullptr;
    }

    return MeshNode->GetMaterial(LocalMaterialIndex);
}
