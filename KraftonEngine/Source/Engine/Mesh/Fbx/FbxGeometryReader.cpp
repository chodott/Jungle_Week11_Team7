#include "Mesh/Fbx/FbxGeometryReader.h"

#include "Mesh/Fbx/FbxTransformUtils.h"

#include <cmath>

namespace
{
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

		if (Element->GetReferenceMode() == FbxLayerElement::eIndexToDirect || Element->GetReferenceMode() == FbxLayerElement::eIndex)
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

	bool TryGetLayerElementColor(FbxLayerElementVertexColor* Element, int32 ControlPointIndex, int32 PolygonVertexIndex, FbxColor& OutValue)
	{
		if (!Element)
		{
			return false;
		}

		int32 ColorIndex = 0;
		switch (Element->GetMappingMode())
		{
		case FbxLayerElement::eByControlPoint:
			ColorIndex = ControlPointIndex;
			break;
		case FbxLayerElement::eByPolygonVertex:
			ColorIndex = PolygonVertexIndex;
			break;
		case FbxLayerElement::eAllSame:
			ColorIndex = 0;
			break;
		default:
			return false;
		}

		if (ColorIndex < 0)
		{
			return false;
		}

		if (Element->GetReferenceMode() == FbxLayerElement::eIndexToDirect || Element->GetReferenceMode() == FbxLayerElement::eIndex)
		{
			if (ColorIndex >= Element->GetIndexArray().GetCount())
			{
				return false;
			}
			ColorIndex = Element->GetIndexArray().GetAt(ColorIndex);
		}

		if (ColorIndex < 0 || ColorIndex >= Element->GetDirectArray().GetCount())
		{
			return false;
		}

		OutValue = Element->GetDirectArray().GetAt(ColorIndex);
		return true;
	}
}

FFbxMeshImportSpace FFbxMeshImportSpace::IdentitySpace()
{
	FFbxMeshImportSpace Result;
	return Result;
}

FFbxMeshImportSpace FFbxMeshImportSpace::FromStaticMeshNode(FbxNode* MeshNode)
{
	FFbxMeshImportSpace Result;
	if (!MeshNode)
	{
		return Result;
	}

	const FbxAMatrix GeometryTransform = FFbxTransformUtils::GetGeometryTransform(MeshNode);
	const FbxAMatrix GlobalTransform   = MeshNode->EvaluateGlobalTransform();
	Result.VertexTransform            = FFbxTransformUtils::ToEngineMatrix(GlobalTransform * GeometryTransform);
	Result.NormalTransform            = Result.VertexTransform.GetInverse().GetTransposed();
	return Result;
}

FVector FFbxGeometryReader::ReadPosition(FbxMesh* Mesh, int32 ControlPointIndex)
{
	if (!Mesh || ControlPointIndex < 0 || ControlPointIndex >= Mesh->GetControlPointsCount())
	{
		return FVector(0.0f, 0.0f, 0.0f);
	}

	const FbxVector4 P = Mesh->GetControlPointAt(ControlPointIndex);
	return FVector(static_cast<float>(P[0]), static_cast<float>(P[1]), static_cast<float>(P[2]));
}

bool FFbxGeometryReader::TryReadNormal(FbxMesh* Mesh, int32 PolygonIndex, int32 CornerIndex, FVector& OutNormal)
{
	FbxVector4 Normal;
	if (Mesh && Mesh->GetPolygonVertexNormal(PolygonIndex, CornerIndex, Normal))
	{
		OutNormal = FVector(static_cast<float>(Normal[0]), static_cast<float>(Normal[1]), static_cast<float>(Normal[2]));
		if (!OutNormal.IsNearlyZero())
		{
			OutNormal.Normalize();
		}
		else
		{
			OutNormal = FVector::UpVector;
		}
		return true;
	}

	OutNormal = FVector::UpVector;
	return false;
}

FVector FFbxGeometryReader::ComputeTriangleNormal(const FVector& P0, const FVector& P1, const FVector& P2)
{
	FVector N = (P1 - P0).Cross(P2 - P0);
	if (N.IsNearlyZero(1.0e-6f))
	{
		return FVector::UpVector;
	}
	N.Normalize();
	return N;
}

int32 FFbxGeometryReader::GetUVSetCount(FbxMesh* Mesh)
{
	if (!Mesh)
	{
		return 0;
	}

	FbxStringList UVSetNames;
	Mesh->GetUVSetNames(UVSetNames);
	return static_cast<int32>(UVSetNames.GetCount());
}

void FFbxGeometryReader::GetUVSetNames(FbxMesh* Mesh, TArray<FString>& OutUVSetNames)
{
	OutUVSetNames.clear();
	if (!Mesh)
	{
		return;
	}

	FbxStringList UVSetNames;
	Mesh->GetUVSetNames(UVSetNames);
	OutUVSetNames.reserve(static_cast<size_t>(UVSetNames.GetCount()));
	for (int32 UVSetIndex = 0; UVSetIndex < UVSetNames.GetCount(); ++UVSetIndex)
	{
		const char* Name = UVSetNames.GetStringAt(UVSetIndex);
		OutUVSetNames.push_back(Name ? FString(Name) : FString());
	}
}

FVector2 FFbxGeometryReader::ReadUVByName(FbxMesh* Mesh, int32 PolygonIndex, int32 CornerIndex, const char* UVSetName)
{
	if (!Mesh || !UVSetName || UVSetName[0] == '\0')
	{
		return FVector2(0.0f, 0.0f);
	}

	FbxVector2 UV;
	bool       bUnmapped = false;
	if (Mesh->GetPolygonVertexUV(PolygonIndex, CornerIndex, UVSetName, UV, bUnmapped) && !bUnmapped)
	{
		return FVector2(static_cast<float>(UV[0]), 1.0f - static_cast<float>(UV[1]));
	}

	return FVector2(0.0f, 0.0f);
}

FVector2 FFbxGeometryReader::ReadUVByChannel(FbxMesh* Mesh, int32 PolygonIndex, int32 CornerIndex, int32 ChannelIndex)
{
	if (!Mesh)
	{
		return FVector2(0.0f, 0.0f);
	}

	FbxStringList UVSetNames;
	Mesh->GetUVSetNames(UVSetNames);
	if (ChannelIndex < 0 || ChannelIndex >= UVSetNames.GetCount())
	{
		return FVector2(0.0f, 0.0f);
	}

	return ReadUVByName(Mesh, PolygonIndex, CornerIndex, UVSetNames.GetStringAt(ChannelIndex));
}

FVector4 FFbxGeometryReader::ReadVertexColor(FbxMesh* Mesh, int32 ControlPointIndex, int32 PolygonVertexIndex)
{
	if (!Mesh || !Mesh->GetLayer(0))
	{
		return FVector4(1.0f, 1.0f, 1.0f, 1.0f);
	}

	FbxColor Color;
	if (TryGetLayerElementColor(Mesh->GetLayer(0)->GetVertexColors(), ControlPointIndex, PolygonVertexIndex, Color))
	{
		return FVector4(static_cast<float>(Color.mRed), static_cast<float>(Color.mGreen), static_cast<float>(Color.mBlue), static_cast<float>(Color.mAlpha));
	}

	return FVector4(1.0f, 1.0f, 1.0f, 1.0f);
}

bool FFbxGeometryReader::TryReadTangent(FbxMesh* Mesh, int32 ControlPointIndex, int32 PolygonVertexIndex, FVector4& OutTangent)
{
	if (!Mesh || !Mesh->GetLayer(0))
	{
		return false;
	}

	FbxVector4 TangentValue;
	if (!TryGetLayerElementVector4(Mesh->GetLayer(0)->GetTangents(), ControlPointIndex, PolygonVertexIndex, TangentValue))
	{
		return false;
	}

	FVector Tangent(static_cast<float>(TangentValue[0]), static_cast<float>(TangentValue[1]), static_cast<float>(TangentValue[2]));
	if (Tangent.IsNearlyZero(1.0e-6f))
	{
		return false;
	}
	Tangent.Normalize();

	const float W = (std::fabs(static_cast<float>(TangentValue[3])) > 1.0e-6f) ? static_cast<float>(TangentValue[3]) : 1.0f;
	OutTangent = FVector4(Tangent.X, Tangent.Y, Tangent.Z, W);
	return true;
}

FVector FFbxGeometryReader::ComputeTriangleTangent(
	const FVector&  P0,
	const FVector&  P1,
	const FVector&  P2,
	const FVector2& UV0,
	const FVector2& UV1,
	const FVector2& UV2
	)
{
	const FVector Edge1 = P1 - P0;
	const FVector Edge2 = P2 - P0;
	const float   DU1   = UV1.X - UV0.X;
	const float   DU2   = UV2.X - UV0.X;
	const float   DV1   = UV1.Y - UV0.Y;
	const float   DV2   = UV2.Y - UV0.Y;
	const float   Denom = DU1 * DV2 - DU2 * DV1;
	if (std::fabs(Denom) <= 1.0e-6f)
	{
		return FVector::ForwardVector;
	}

	FVector Tangent = (Edge1 * DV2 - Edge2 * DV1) * (1.0f / Denom);
	if (Tangent.IsNearlyZero(1.0e-6f))
	{
		return FVector::ForwardVector;
	}
	Tangent.Normalize();
	return Tangent;
}

bool FFbxGeometryReader::ReadTriangleSample(FbxMesh* Mesh, int32 PolygonIndex, const FFbxMeshImportSpace& ImportSpace, FFbxTriangleSample& OutTriangle)
{
	return ReadTriangleSample(Mesh, PolygonIndex, ImportSpace, nullptr, OutTriangle);
}

bool FFbxGeometryReader::ReadTriangleSample(FbxMesh* Mesh, int32 PolygonIndex, const FFbxMeshImportSpace& ImportSpace, const char* UV0SetName, FFbxTriangleSample& OutTriangle)
{
	if (!Mesh || Mesh->GetPolygonSize(PolygonIndex) != 3)
	{
		return false;
	}

	for (int32 CornerIndex = 0; CornerIndex < 3; ++CornerIndex)
	{
		const int32 ControlPointIndex = Mesh->GetPolygonVertex(PolygonIndex, CornerIndex);
		OutTriangle.ControlPointIndices[CornerIndex] = ControlPointIndex;
		const FVector LocalPosition = ReadPosition(Mesh, ControlPointIndex);
		OutTriangle.Positions[CornerIndex] = FFbxTransformUtils::TransformPositionByMatrix(LocalPosition, ImportSpace.VertexTransform);
		OutTriangle.UV0[CornerIndex] = UV0SetName
			? ReadUVByName(Mesh, PolygonIndex, CornerIndex, UV0SetName)
			: ReadUVByChannel(Mesh, PolygonIndex, CornerIndex, 0);
	}

	OutTriangle.FallbackNormal = ComputeTriangleNormal(OutTriangle.Positions[0], OutTriangle.Positions[1], OutTriangle.Positions[2]);
	OutTriangle.FallbackTangent = ComputeTriangleTangent(
		OutTriangle.Positions[0],
		OutTriangle.Positions[1],
		OutTriangle.Positions[2],
		OutTriangle.UV0[0],
		OutTriangle.UV0[1],
		OutTriangle.UV0[2]
		);
	return true;
}
