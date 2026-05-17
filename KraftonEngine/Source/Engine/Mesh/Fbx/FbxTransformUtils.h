#pragma once

#include "Core/CoreTypes.h"
#include "Math/Matrix.h"

#include <fbxsdk.h>

class FFbxTransformUtils
{
public:
	static FMatrix ToEngineMatrix(const FbxMatrix& Matrix);
	static FMatrix ToEngineMatrix(const FbxAMatrix& Matrix);
	static FbxAMatrix GetGeometryTransform(FbxNode* Node);

	static FbxAMatrix GetNodeGeometryTransform(FbxNode* Node)
	{
		return GetGeometryTransform(Node);
	}

	static FVector TransformPositionByMatrix(const FVector& P, const FMatrix& M);
	static FVector TransformDirectionByMatrix(const FVector& V, const FMatrix& M);
	static FVector TransformNormalByMatrix(const FVector& V, const FMatrix& M);
	static FVector OrthogonalizeTangentToNormal(const FVector& Tangent, const FVector& Normal);
	static FVector TransformTangentByMatrix(const FVector& Tangent, const FMatrix& TangentMatrix, const FVector& ReferenceNormal);
	static FVector TransformVectorNoNormalizeByMatrix(const FVector& V, const FMatrix& M);
	static float   Determinant3x3(const FMatrix& Matrix); 
};
