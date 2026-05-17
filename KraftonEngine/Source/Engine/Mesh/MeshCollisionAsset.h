#pragma once

#include "Core/CoreTypes.h"
#include "Math/Vector.h"
#include "Math/Matrix.h"
#include "Serialization/Archive.h"

enum class EImportedCollisionType : uint8
{
    Unknown = 0,
    Convex,
    Box,
    Sphere,
    Capsule,
};

namespace MeshSerializationUtils
{
    inline void SerializeMatrix(FArchive& Ar, FMatrix& Matrix)
    {
        Ar.Serialize(&Matrix.M[0][0], sizeof(Matrix.M));
    }
}

// FBX collision proxy를 engine asset 안에 보존하기 위한 공통 cooked data.
// Vertices/Indices는 collision node local space 기준으로 저장하고,
// LocalMatrix는 collision node local -> static mesh local 또는 parent bone local 변환이다.
struct FImportedCollisionShape
{
    FString SourceNodeName;
    FString TargetMeshName;
    int32   CollisionIndex = 0;

    EImportedCollisionType Type        = EImportedCollisionType::Unknown;
    FMatrix                LocalMatrix = FMatrix::Identity;

    // Primitive fitting은 Vertices의 local bounds 기준이다.
    FVector LocalCenter = FVector(0.0f, 0.0f, 0.0f);

    int32   ParentBoneIndex = -1;
    FString ParentBoneName;

    TArray<FVector> Vertices;
    TArray<uint32>  Indices;

    FVector BoxExtent         = FVector(0.0f, 0.0f, 0.0f);
    float   SphereRadius      = 0.0f;
    float   CapsuleRadius     = 0.0f;
    float   CapsuleHalfHeight = 0.0f;

    friend FArchive& operator<<(FArchive& Ar, FImportedCollisionShape& Shape)
    {
        Ar << Shape.SourceNodeName;
        Ar << Shape.TargetMeshName;
        Ar << Shape.CollisionIndex;

        uint8 TypeValue = static_cast<uint8>(Shape.Type);
        Ar << TypeValue;

        if (Ar.IsLoading())
        {
            Shape.Type = static_cast<EImportedCollisionType>(TypeValue);
        }

        MeshSerializationUtils::SerializeMatrix(Ar, Shape.LocalMatrix);
        Ar << Shape.LocalCenter;

        Ar << Shape.ParentBoneIndex;
        Ar << Shape.ParentBoneName;

        Ar << Shape.Vertices;
        Ar << Shape.Indices;

        Ar << Shape.BoxExtent;
        Ar << Shape.SphereRadius;
        Ar << Shape.CapsuleRadius;
        Ar << Shape.CapsuleHalfHeight;

        return Ar;
    }
};
