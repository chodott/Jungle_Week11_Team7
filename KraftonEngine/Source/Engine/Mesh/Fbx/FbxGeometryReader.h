#pragma once

#include "Core/CoreTypes.h"
#include "Math/Matrix.h"
#include "Math/Vector.h"

#include <fbxsdk.h>

struct FFbxMeshImportSpace
{
	FMatrix VertexTransform = FMatrix::Identity;
	FMatrix NormalTransform = FMatrix::Identity;

	static FFbxMeshImportSpace FromStaticMeshNode(FbxNode* MeshNode);
	static FFbxMeshImportSpace IdentitySpace();
};

struct FFbxTriangleSample
{
	int32    ControlPointIndices[3] = { -1, -1, -1 };
	FVector  Positions[3];
	FVector2 UV0[3];
	FVector  FallbackNormal;
	FVector  FallbackTangent;
};

class FFbxGeometryReader
{
public:
	static FVector ReadPosition(FbxMesh* Mesh, int32 ControlPointIndex);
	static bool    TryReadNormal(FbxMesh* Mesh, int32 PolygonIndex, int32 CornerIndex, FVector& OutNormal);
	static FVector ComputeTriangleNormal(const FVector& P0, const FVector& P1, const FVector& P2);

	static int32 GetUVSetCount(FbxMesh* Mesh);
	static void  GetUVSetNames(FbxMesh* Mesh, TArray<FString>& OutUVSetNames);
	static FVector2 ReadUVByName(FbxMesh* Mesh, int32 PolygonIndex, int32 CornerIndex, const char* UVSetName);
	static FVector2 ReadUVByChannel(FbxMesh* Mesh, int32 PolygonIndex, int32 CornerIndex, int32 ChannelIndex);

	static FVector4 ReadVertexColor(FbxMesh* Mesh, int32 ControlPointIndex, int32 PolygonVertexIndex);
	static bool     TryReadTangent(FbxMesh* Mesh, int32 ControlPointIndex, int32 PolygonVertexIndex, FVector4& OutTangent);

	static FVector ComputeTriangleTangent(
		const FVector&  P0,
		const FVector&  P1,
		const FVector&  P2,
		const FVector2& UV0,
		const FVector2& UV1,
		const FVector2& UV2
		);

	static bool ReadTriangleSample(FbxMesh* Mesh, int32 PolygonIndex, const FFbxMeshImportSpace& ImportSpace, FFbxTriangleSample& OutTriangle);
	static bool ReadTriangleSample(FbxMesh* Mesh, int32 PolygonIndex, const FFbxMeshImportSpace& ImportSpace, const char* UV0SetName, FFbxTriangleSample& OutTriangle);
};
