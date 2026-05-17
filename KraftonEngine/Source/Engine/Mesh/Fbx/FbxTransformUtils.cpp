#include "Mesh/Fbx/FbxTransformUtils.h"
#include <cmath>

FMatrix FFbxTransformUtils::ToEngineMatrix(const FbxMatrix& FbxMat)
{
	FMatrix Mat;
	for (int32 Row = 0; Row < 4; ++Row)
	{
		for (int32 Col = 0; Col < 4; ++Col)
		{
			Mat.M[Row][Col] = static_cast<float>(FbxMat.Get(Row, Col));
		}
	}
	return Mat;
}

FMatrix FFbxTransformUtils::ToEngineMatrix(const FbxAMatrix& FbxMat)
{
	FMatrix Mat;
	for (int32 Row = 0; Row < 4; ++Row)
	{
		for (int32 Col = 0; Col < 4; ++Col)
		{
			Mat.M[Row][Col] = static_cast<float>(FbxMat.Get(Row, Col));
		}
	}
	return Mat;
}

FbxAMatrix FFbxTransformUtils::GetGeometryTransform(FbxNode* Node)
{
	FbxAMatrix GeometryTransform;
	GeometryTransform.SetIdentity();
	if (!Node)
	{
		return GeometryTransform;
	}

	GeometryTransform.SetT(Node->GetGeometricTranslation(FbxNode::eSourcePivot));
	GeometryTransform.SetR(Node->GetGeometricRotation(FbxNode::eSourcePivot));
	GeometryTransform.SetS(Node->GetGeometricScaling(FbxNode::eSourcePivot));
	return GeometryTransform;
}

FVector FFbxTransformUtils::TransformPositionByMatrix(const FVector& P, const FMatrix& M)
{
	return M.TransformPositionWithW(P);
}

FVector FFbxTransformUtils::TransformDirectionByMatrix(const FVector& V, const FMatrix& M)
{
	FVector Result = M.TransformVector(V);
	if (!Result.IsNearlyZero()) Result.Normalize();
	return Result;
}

FVector FFbxTransformUtils::TransformNormalByMatrix(const FVector& V, const FMatrix& M)
{
	return TransformDirectionByMatrix(V, M);
}

FVector FFbxTransformUtils::OrthogonalizeTangentToNormal(const FVector& Tangent, const FVector& Normal)
{
	FVector N = Normal;
	if (!N.IsNearlyZero()) N.Normalize();
	else N = FVector::UpVector;

	FVector T = Tangent - (N * Tangent.Dot(N));
	if (T.IsNearlyZero(1.0e-6f))
	{
		const FVector Candidate = (std::fabs(N.Z) < 0.999f) ? FVector::UpVector : FVector::RightVector;
		T                       = Candidate - (N * Candidate.Dot(N));
	}
	if (!T.IsNearlyZero()) T.Normalize();
	return T;
}

FVector FFbxTransformUtils::TransformTangentByMatrix(const FVector& Tangent, const FMatrix& TangentMatrix, const FVector& ReferenceNormal)
{
	return OrthogonalizeTangentToNormal(TransformDirectionByMatrix(Tangent, TangentMatrix), ReferenceNormal);
}

FVector FFbxTransformUtils::TransformVectorNoNormalizeByMatrix(const FVector& V, const FMatrix& M)
{
	return M.TransformVector(V);
}

float FFbxTransformUtils::Determinant3x3(const FMatrix& Matrix)
{
	return Matrix.M[0][0] * (Matrix.M[1][1] * Matrix.M[2][2] - Matrix.M[1][2] * Matrix.M[2][1]) - Matrix.M[0][1] * (Matrix.M[1][0] * Matrix.M[2][2] - Matrix.M[
		1][2] * Matrix.M[2][0]) + Matrix.M[0][2] * (Matrix.M[1][0] * Matrix.M[2][1] - Matrix.M[1][1] * Matrix.M[2][0]);
}
