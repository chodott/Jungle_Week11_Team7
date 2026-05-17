#include "Mesh/Fbx/FbxMorphTargetImporter.h"

#include "Mesh/Fbx/FbxImportContext.h"
#include "Mesh/Fbx/FbxTransformUtils.h"

#include <fbxsdk.h>

#include <cmath>
#include <utility>
#include <algorithm>

namespace
{
    bool IsNearlyZeroVector(const FVector& V, float Tolerance = 1.0e-6f)
    {
        return std::fabs(V.X) <= Tolerance && std::fabs(V.Y) <= Tolerance && std::fabs(V.Z) <= Tolerance;
    }

    bool IsNearlyZeroVector4(const FVector4& V, float Tolerance = 1.0e-6f)
    {
        return std::fabs(V.X) <= Tolerance && std::fabs(V.Y) <= Tolerance && std::fabs(V.Z) <= Tolerance && std::fabs(V.W) <= Tolerance;
    }

    template <typename LayerElementType>
    bool TryGetLayerElementVector4(LayerElementType* Element, int32 ControlPointIndex, int32 PolygonVertexIndex, FbxVector4& OutValue)
    {
        if (!Element)
        {
            return false;
        }

        int32 ElementIndex = 0;

        switch (Element->GetMappingMode())
        {
        case FbxLayerElement::eByControlPoint:
            ElementIndex = ControlPointIndex;
            break;

        case FbxLayerElement::eByPolygonVertex:
            ElementIndex = PolygonVertexIndex;
            break;

        case FbxLayerElement::eAllSame:
            ElementIndex = 0;
            break;

        default:
            return false;
        }

        if (ElementIndex < 0)
        {
            return false;
        }

        if (Element->GetReferenceMode() == FbxLayerElement::eIndexToDirect)
        {
            if (ElementIndex >= Element->GetIndexArray().GetCount())
            {
                return false;
            }

            ElementIndex = Element->GetIndexArray().GetAt(ElementIndex);
        }

        if (ElementIndex < 0 || ElementIndex >= Element->GetDirectArray().GetCount())
        {
            return false;
        }

        OutValue = Element->GetDirectArray().GetAt(ElementIndex);
        return true;
    }

    static bool TryReadShapeNormal(FbxShape* Shape, int32 ControlPointIndex, int32 PolygonVertexIndex, FVector& OutNormal)
    {
        if (!Shape || !Shape->GetLayer(0))
        {
            return false;
        }

        FbxVector4 Value;
        if (!TryGetLayerElementVector4(Shape->GetLayer(0)->GetNormals(), ControlPointIndex, PolygonVertexIndex, Value))
        {
            return false;
        }

        OutNormal = FVector(static_cast<float>(Value[0]), static_cast<float>(Value[1]), static_cast<float>(Value[2])).Normalized();
        return true;
    }

    static bool TryReadShapeTangent(FbxShape* Shape, int32 ControlPointIndex, int32 PolygonVertexIndex, FVector4& OutTangent)
    {
        if (!Shape || !Shape->GetLayer(0))
        {
            return false;
        }

        FbxVector4 Value;
        if (!TryGetLayerElementVector4(Shape->GetLayer(0)->GetTangents(), ControlPointIndex, PolygonVertexIndex, Value))
        {
            return false;
        }

        FVector Tangent(static_cast<float>(Value[0]), static_cast<float>(Value[1]), static_cast<float>(Value[2]));
        Tangent = Tangent.Normalized();

        const float W = (std::fabs(static_cast<float>(Value[3])) > 1.0e-6f) ? static_cast<float>(Value[3]) : 1.0f;
        OutTangent    = FVector4(Tangent.X, Tangent.Y, Tangent.Z, W);
        return true;
    }

    static FMorphTargetShape& FindOrAddShapeByFullWeight(FMorphTargetLOD& MorphLOD, float FullWeight)
    {
        for (FMorphTargetShape& ExistingShape : MorphLOD.Shapes)
        {
            if (std::fabs(ExistingShape.FullWeight - FullWeight) <= 1.0e-4f)
            {
                return ExistingShape;
            }
        }

        FMorphTargetShape NewShape;
        NewShape.FullWeight = FullWeight;
        MorphLOD.Shapes.push_back(std::move(NewShape));
        return MorphLOD.Shapes.back();
    }

    static bool BuildShapeDelta(FbxMesh* Mesh, FbxShape* Shape, const FFbxImportedMorphSourceVertex& Source, FMorphTargetDelta& OutDelta)
    {
        FbxVector4* BaseControlPoints   = Mesh ? Mesh->GetControlPoints() : nullptr;
        FbxVector4* TargetControlPoints = Shape ? Shape->GetControlPoints() : nullptr;

        if (!BaseControlPoints || !TargetControlPoints)
        {
            return false;
        }

        const int32 CPIndex           = Source.ControlPointIndex;
        const int32 ControlPointCount = Mesh->GetControlPointsCount();

        if (CPIndex < 0 || CPIndex >= ControlPointCount)
        {
            return false;
        }

        const FbxVector4 BaseP   = BaseControlPoints[CPIndex];
        const FbxVector4 TargetP = TargetControlPoints[CPIndex];

        const FVector LocalDelta(
            static_cast<float>(TargetP[0] - BaseP[0]),
            static_cast<float>(TargetP[1] - BaseP[1]),
            static_cast<float>(TargetP[2] - BaseP[2])
        );

        OutDelta.VertexIndex   = Source.VertexIndex;
        OutDelta.PositionDelta = FFbxTransformUtils::TransformVectorNoNormalizeByMatrix(LocalDelta, Source.MeshToReference);
        OutDelta.NormalDelta   = FVector(0.0f, 0.0f, 0.0f);
        OutDelta.TangentDelta  = FVector4(0.0f, 0.0f, 0.0f, 0.0f);

        FVector TargetNormalInReference = Source.BaseNormalInReference;

        FVector TargetLocalNormal;
        if (TryReadShapeNormal(Shape, CPIndex, Source.PolygonVertexIndex, TargetLocalNormal))
        {
            TargetNormalInReference = FFbxTransformUtils::TransformNormalByMatrix(TargetLocalNormal, Source.NormalToReference);
            OutDelta.NormalDelta    = TargetNormalInReference - Source.BaseNormalInReference;
        }

        FVector4 TargetLocalTangent;
        if (TryReadShapeTangent(Shape, CPIndex, Source.PolygonVertexIndex, TargetLocalTangent))
        {
            const FVector TargetTangentInReference = FFbxTransformUtils::TransformTangentByMatrix(
                FVector(TargetLocalTangent.X, TargetLocalTangent.Y, TargetLocalTangent.Z),
                Source.MeshToReference,
                TargetNormalInReference
            );

            OutDelta.TangentDelta = FVector4(
                TargetTangentInReference.X - Source.BaseTangentInReference.X,
                TargetTangentInReference.Y - Source.BaseTangentInReference.Y,
                TargetTangentInReference.Z - Source.BaseTangentInReference.Z,
                TargetLocalTangent.W - Source.BaseTangentInReference.W
            );
        }

        return !(IsNearlyZeroVector(OutDelta.PositionDelta) && IsNearlyZeroVector(OutDelta.NormalDelta) && IsNearlyZeroVector4(OutDelta.TangentDelta));
    }

    static bool HasSourceInfo(const FMorphTarget& Morph, const FMorphTargetSourceInfo& Info)
    {
        for (const FMorphTargetSourceInfo& Existing : Morph.SourceInfos)
        {
            if (Existing.SourceMeshNodeName == Info.SourceMeshNodeName && Existing.SourceBlendShapeName == Info.SourceBlendShapeName && Existing.
                SourceChannelName == Info.SourceChannelName)
            {
                return true;
            }
        }

        return false;
    }

    static void AddUniqueSourceInfo(FMorphTarget& Morph, const FMorphTargetSourceInfo& Info)
    {
        if (!HasSourceInfo(Morph, Info))
        {
            Morph.SourceInfos.push_back(Info);
        }
    }

    struct FMorphSourceMeshGroup
    {
        FbxNode* MeshNode = nullptr;
        FbxMesh* Mesh     = nullptr;

        FString SourceMeshNodeName;

        TArray<const FFbxImportedMorphSourceVertex*> Sources;
    };

    static FMorphSourceMeshGroup* FindOrAddSourceMeshGroup(
        TArray<FMorphSourceMeshGroup>& Groups,
        FbxNode*                       MeshNode,
        FbxMesh*                       Mesh,
        const FString&                 SourceMeshNodeName
        )
    {
        for (FMorphSourceMeshGroup& Group : Groups)
        {
            if (Group.MeshNode == MeshNode && Group.Mesh == Mesh)
            {
                return &Group;
            }
        }

        FMorphSourceMeshGroup NewGroup;
        NewGroup.MeshNode           = MeshNode;
        NewGroup.Mesh               = Mesh;
        NewGroup.SourceMeshNodeName = SourceMeshNodeName;

        Groups.push_back(std::move(NewGroup));
        return &Groups.back();
    }
}

void FFbxMorphTargetImporter::ImportMorphTargets(
    const TArray<TArray<FFbxImportedMorphSourceVertex>>& MorphSourcesByLOD,
    TArray<FMorphTarget>&                                OutMorphTargets,
    FFbxImportContext&                                   BuildContext
    )
{
    OutMorphTargets.clear();

    TMap<FString, int32> MorphNameToIndex;

    for (int32 LODIndex = 0; LODIndex < static_cast<int32>(MorphSourcesByLOD.size()); ++LODIndex)
    {
        const TArray<FFbxImportedMorphSourceVertex>& Sources = MorphSourcesByLOD[LODIndex];

        TArray<FMorphSourceMeshGroup> SourceMeshGroups;
        
        for (const FFbxImportedMorphSourceVertex& Source : Sources)
        {
            if (Source.Mesh)
            {
                FMorphSourceMeshGroup* Group = FindOrAddSourceMeshGroup(SourceMeshGroups, Source.MeshNode, Source.Mesh, Source.SourceMeshNodeName);

                if (Group)
                {
                    Group->Sources.push_back(&Source);
                }
            }
        }

        for (const FMorphSourceMeshGroup& Group : SourceMeshGroups)
        {
            FbxMesh*                                            Mesh        = Group.Mesh;
            const TArray<const FFbxImportedMorphSourceVertex*>& MeshSources = Group.Sources;
            
            if (!Mesh)
            {
                continue;
            }

            const int32 BlendShapeCount = Mesh->GetDeformerCount(FbxDeformer::eBlendShape);

            for (int32 BlendShapeIndex = 0; BlendShapeIndex < BlendShapeCount; ++BlendShapeIndex)
            {
                FbxBlendShape* BlendShape = static_cast<FbxBlendShape*>(Mesh->GetDeformer(BlendShapeIndex, FbxDeformer::eBlendShape));

                if (!BlendShape)
                {
                    continue;
                }

                const int32 ChannelCount = BlendShape->GetBlendShapeChannelCount();

                for (int32 ChannelIndex = 0; ChannelIndex < ChannelCount; ++ChannelIndex)
                {
                    FbxBlendShapeChannel* Channel = BlendShape->GetBlendShapeChannel(ChannelIndex);

                    if (!Channel)
                    {
                        continue;
                    }

                    const int32 TargetShapeCount = Channel->GetTargetShapeCount();

                    if (TargetShapeCount <= 0)
                    {
                        continue;
                    }

                    if (TargetShapeCount > 1)
                    {
                        BuildContext.AddWarningOnce(
                            ESkeletalImportWarningType::UnsupportedMorphInBetween,
                            "Morph target has in-between shapes. FullWeight data is imported; runtime interpolation must use it."
                        );
                    }

                    FString MorphName = Channel->GetName();
                    if (MorphName.empty() && Channel->GetTargetShape(0))
                    {
                        MorphName = Channel->GetTargetShape(0)->GetName();
                    }

                    if (MorphName.empty())
                    {
                        continue;
                    }

                    int32 MorphIndex = -1;

                    auto Existing = MorphNameToIndex.find(MorphName);
                    if (Existing != MorphNameToIndex.end())
                    {
                        MorphIndex = Existing->second;
                    }
                    else
                    {
                        FMorphTarget NewMorph;
                        NewMorph.Name = MorphName;
                        NewMorph.LODModels.resize(MorphSourcesByLOD.size());

                        OutMorphTargets.push_back(std::move(NewMorph));

                        MorphIndex                  = static_cast<int32>(OutMorphTargets.size()) - 1;
                        MorphNameToIndex[MorphName] = MorphIndex;
                    }

                    FMorphTarget&    Morph    = OutMorphTargets[MorphIndex];
                    FMorphTargetLOD& MorphLOD = Morph.LODModels[LODIndex];

                    FMorphTargetSourceInfo SourceInfo;
                    SourceInfo.SourceMeshNodeName   = Group.SourceMeshNodeName;
                    SourceInfo.SourceBlendShapeName = BlendShape && BlendShape->GetName() ? FString(BlendShape->GetName()) : MorphName;
                    SourceInfo.SourceChannelName    = Channel && Channel->GetName() ? FString(Channel->GetName()) : MorphName;

                    AddUniqueSourceInfo(Morph, SourceInfo);

                    double* FullWeights = Channel->GetTargetShapeFullWeights();

                    for (int32 TargetShapeIndex = 0; TargetShapeIndex < TargetShapeCount; ++TargetShapeIndex)
                    {
                        FbxShape* Shape = Channel->GetTargetShape(TargetShapeIndex);

                        if (!Shape)
                        {
                            continue;
                        }

                        const float        FullWeight = FullWeights ? static_cast<float>(FullWeights[TargetShapeIndex]) : 100.0f;
                        FMorphTargetShape& MorphShape = FindOrAddShapeByFullWeight(MorphLOD, FullWeight);

                        for (const FFbxImportedMorphSourceVertex* Source : MeshSources)
                        {
                            if (!Source)
                            {
                                continue;
                            }

                            FMorphTargetDelta Delta;
                            if (BuildShapeDelta(Mesh, Shape, *Source, Delta))
                            {
                                MorphShape.Deltas.push_back(Delta);
                            }
                        }
                    }
                }
            }
        }
    }

    for (FMorphTarget& Morph : OutMorphTargets)
    {
        std::sort(
            Morph.SourceInfos.begin(),
            Morph.SourceInfos.end(),
            [](const FMorphTargetSourceInfo& A, const FMorphTargetSourceInfo& B)
            {
                if (A.SourceMeshNodeName != B.SourceMeshNodeName)
                {
                    return A.SourceMeshNodeName < B.SourceMeshNodeName;
                }
                if (A.SourceBlendShapeName != B.SourceBlendShapeName)
                {
                    return A.SourceBlendShapeName < B.SourceBlendShapeName;
                }
                return A.SourceChannelName < B.SourceChannelName;
            }
        );

        for (FMorphTargetLOD& LOD : Morph.LODModels)
        {
            std::sort(
                LOD.Shapes.begin(),
                LOD.Shapes.end(),
                [](const FMorphTargetShape& A, const FMorphTargetShape& B)
                {
                    return A.FullWeight < B.FullWeight;
                }
            );
            for (FMorphTargetShape& Shape : LOD.Shapes)
            {
                std::sort(
                    Shape.Deltas.begin(),
                    Shape.Deltas.end(),
                    [](const FMorphTargetDelta& A, const FMorphTargetDelta& B)
                    {
                        return A.VertexIndex < B.VertexIndex;
                    }
                );

                Shape.Deltas.erase(
                    std::unique(
                        Shape.Deltas.begin(),
                        Shape.Deltas.end(),
                        [](const FMorphTargetDelta& A, const FMorphTargetDelta& B)
                        {
                            return A.VertexIndex == B.VertexIndex;
                        }
                    ),
                    Shape.Deltas.end()
                );
            }
        }
    }

    OutMorphTargets.erase(
        std::remove_if(
            OutMorphTargets.begin(),
            OutMorphTargets.end(),
            [](const FMorphTarget& Morph)
            {
                for (const FMorphTargetLOD& LOD : Morph.LODModels)
                {
                    for (const FMorphTargetShape& Shape : LOD.Shapes)
                    {
                        if (!Shape.Deltas.empty())
                        {
                            return false;
                        }
                    }
                }
                return true;
            }
        ),
        OutMorphTargets.end()
    );
}

