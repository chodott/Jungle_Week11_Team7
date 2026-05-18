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

FMatrix FFbxTransformUtils::RemoveTranslationFromMatrix(const FMatrix& Matrix)
{
	FMatrix Result = Matrix;
	Result.M[3][0] = 0.0f;
	Result.M[3][1] = 0.0f;
	Result.M[3][2] = 0.0f;
	Result.M[3][3] = 1.0f;
	return Result;
}

FMatrix FFbxTransformUtils::RemoveScaleFromMatrix(const FMatrix& Matrix)
{
	FVector XAxis(Matrix.M[0][0], Matrix.M[0][1], Matrix.M[0][2]);
	FVector YAxis(Matrix.M[1][0], Matrix.M[1][1], Matrix.M[1][2]);
	FVector ZAxis(Matrix.M[2][0], Matrix.M[2][1], Matrix.M[2][2]);

	if (!XAxis.IsNearlyZero()) XAxis.Normalize();
	if (!YAxis.IsNearlyZero()) YAxis.Normalize();
	if (!ZAxis.IsNearlyZero()) ZAxis.Normalize();

	FMatrix Result = FMatrix::Identity;
	Result.M[0][0] = XAxis.X;
	Result.M[0][1] = XAxis.Y;
	Result.M[0][2] = XAxis.Z;
	Result.M[1][0] = YAxis.X;
	Result.M[1][1] = YAxis.Y;
	Result.M[1][2] = YAxis.Z;
	Result.M[2][0] = ZAxis.X;
	Result.M[2][1] = ZAxis.Y;
	Result.M[2][2] = ZAxis.Z;
	return Result;
}

void FFbxTransformUtils::DecomposeMatrixPreserveMirror(
	const FMatrix& Matrix,
	FVector&       OutTranslation,
	FQuat&         OutRotation,
	FVector&       OutScale
	)
{
	constexpr float Epsilon = 1.0e-6f;
	OutTranslation          = Matrix.GetLocation();
	OutScale                = Matrix.GetScale();

	if (Determinant3x3(Matrix) < 0.0f)
	{
		OutScale.X = -OutScale.X;
	}

	FMatrix     RotationMatrix = FMatrix::Identity;
	const float ScaleValues[3] = { OutScale.X, OutScale.Y, OutScale.Z };
	for (int32 Row = 0; Row < 3; ++Row)
	{
		const float Scale = ScaleValues[Row];
		if (std::fabs(Scale) > Epsilon)
		{
			RotationMatrix.M[Row][0] = Matrix.M[Row][0] / Scale;
			RotationMatrix.M[Row][1] = Matrix.M[Row][1] / Scale;
			RotationMatrix.M[Row][2] = Matrix.M[Row][2] / Scale;
		}
	}

	OutRotation = RotationMatrix.ToQuat().GetNormalized();
}

namespace
{
	int32 AxisIndexFromUpVector(FbxAxisSystem::EUpVector UpVector)
	{
		switch (UpVector)
		{
		case FbxAxisSystem::eXAxis:
			return 0;
		case FbxAxisSystem::eYAxis:
			return 1;
		case FbxAxisSystem::eZAxis:
			return 2;
		default:
			return 2;
		}
	}

	int32 AxisIndexFromFrontVector(FbxAxisSystem::EUpVector UpVector, FbxAxisSystem::EFrontVector FrontVector)
	{
		const int32 UpAxis   = AxisIndexFromUpVector(UpVector);
		const int32 EvenAxis = (UpAxis + 1) % 3;
		const int32 OddAxis  = (UpAxis + 2) % 3;
		return FrontVector == FbxAxisSystem::eParityEven ? EvenAxis : OddAxis;
	}

	FVector AxisVectorFromIndexAndSign(int32 AxisIndex, int32 Sign)
	{
		const float S = Sign < 0 ? -1.0f : 1.0f;
		if (AxisIndex == 0) return FVector(S, 0.0f, 0.0f);
		if (AxisIndex == 1) return FVector(0.0f, S, 0.0f);
		return FVector(0.0f, 0.0f, S);
	}

	void AssignSourceAxisToTarget(FMatrix& Matrix, const FVector& SourceAxis, const FVector& TargetAxis)
	{
		const float AbsX = std::fabs(SourceAxis.X);
		const float AbsY = std::fabs(SourceAxis.Y);
		const float AbsZ = std::fabs(SourceAxis.Z);

		int32 SourceIndex = 0;
		float SourceSign  = SourceAxis.X < 0.0f ? -1.0f : 1.0f;
		if (AbsY > AbsX && AbsY >= AbsZ)
		{
			SourceIndex = 1;
			SourceSign  = SourceAxis.Y < 0.0f ? -1.0f : 1.0f;
		}
		else if (AbsZ > AbsX && AbsZ > AbsY)
		{
			SourceIndex = 2;
			SourceSign  = SourceAxis.Z < 0.0f ? -1.0f : 1.0f;
		}

		Matrix.M[SourceIndex][0] = TargetAxis.X * SourceSign;
		Matrix.M[SourceIndex][1] = TargetAxis.Y * SourceSign;
		Matrix.M[SourceIndex][2] = TargetAxis.Z * SourceSign;
	}
}

FMatrix FFbxTransformUtils::MakeAxisSystemToEngineAssetMatrix(const FbxAxisSystem& AxisSystem, bool bMirrorHandedness)
{
	int32                             UpSign      = 1;
	int32                             FrontSign   = 1;
	const FbxAxisSystem::EUpVector    UpVector    = AxisSystem.GetUpVector(UpSign);
	const FbxAxisSystem::EFrontVector FrontVector = AxisSystem.GetFrontVector(FrontSign);
	const FbxAxisSystem::ECoordSystem CoordSystem = AxisSystem.GetCoorSystem();

	const int32 UpAxisIndex    = AxisIndexFromUpVector(UpVector);
	const int32 FrontAxisIndex = AxisIndexFromFrontVector(UpVector, FrontVector);

	const FVector SourceUp = AxisVectorFromIndexAndSign(UpAxisIndex, UpSign).Normalized();
	const FVector SourceForward = AxisVectorFromIndexAndSign(FrontAxisIndex, -FrontSign).Normalized();
	FVector       SourceRight = (CoordSystem == FbxAxisSystem::eLeftHanded) ? SourceUp.Cross(SourceForward).Normalized()
	: SourceForward.Cross(SourceUp).Normalized();

	if (bMirrorHandedness)
	{
		SourceRight *= -1.0f;
	}

	FMatrix Result = FMatrix::Identity;
	for (int32 Row = 0; Row < 3; ++Row)
	{
		Result.M[Row][0] = 0.0f;
		Result.M[Row][1] = 0.0f;
		Result.M[Row][2] = 0.0f;
	}

	AssignSourceAxisToTarget(Result, SourceForward, FVector::ForwardVector);
	AssignSourceAxisToTarget(Result, SourceRight, FVector::RightVector);
	AssignSourceAxisToTarget(Result, SourceUp, FVector::UpVector);

	Result.M[3][0] = 0.0f;
	Result.M[3][1] = 0.0f;
	Result.M[3][2] = 0.0f;
	Result.M[3][3] = 1.0f;
	return Result;
}
