#pragma once

#include "Core/CoreTypes.h"
#include "Math/Vector.h"
#include "Engine/Object/Object.h"
#include "Render/Resource/Buffer.h"
#include "Serialization/Archive.h"
#include "Engine/Object/FName.h"
#include "Materials/Material.h"
#include "Materials/MaterialManager.h"
#include <memory>
#include <algorithm>
#include "Mesh/MeshCollisionAsset.h"

static constexpr int32 MAX_STATIC_MESH_UV_CHANNELS = 4;

// FBX imported metadata / scene hierarchy.
enum class EFbxImportedMetadataValueType : uint8
{
	String = 0,
	Bool,
	Int,
	Float,
	Vector3,
	Color,
};

struct FFbxImportedMetadataValue
{
	FString                       Key;
	FString                       StringValue;
	EFbxImportedMetadataValueType Type        = EFbxImportedMetadataValueType::String;
	bool                          BoolValue   = false;
	int32                         IntValue    = 0;
	float                         FloatValue  = 0.0f;
	FVector                       VectorValue = FVector(0.0f, 0.0f, 0.0f);
	FVector4                      ColorValue  = FVector4(0.0f, 0.0f, 0.0f, 1.0f);

	friend FArchive& operator<<(FArchive& Ar, FFbxImportedMetadataValue& Value)
	{
		Ar << Value.Key;
		Ar << Value.StringValue;
		uint8 TypeValue = static_cast<uint8>(Value.Type);
		Ar << TypeValue;
		if (Ar.IsLoading()) Value.Type = static_cast<EFbxImportedMetadataValueType>(TypeValue);
		Ar << Value.BoolValue;
		Ar << Value.IntValue;
		Ar << Value.FloatValue;
		Ar << Value.VectorValue;
		Ar << Value.ColorValue;
		return Ar;
	}
};

struct FFbxImportedNodeMetadata
{
	FString                           SourceNodeName;
	FString                           NodePath;
	TArray<FFbxImportedMetadataValue> Values;

	friend FArchive& operator<<(FArchive& Ar, FFbxImportedNodeMetadata& Metadata)
	{
		Ar << Metadata.SourceNodeName;
		Ar << Metadata.NodePath;
		Ar << Metadata.Values;
		return Ar;
	}
};

struct FFbxImportedSceneNode
{
	FString Name;
	FString NodePath;
	FString AttributeType;
	int32   ParentIndex          = -1;
	int32   SourceLODIndex       = 0;
	bool    bHasMesh             = false;
	bool    bHasSkeleton         = false;
	bool    bIsSocket            = false;
	bool    bIsCollision         = false;
	FMatrix LocalMatrix          = FMatrix::Identity;
	FMatrix GlobalMatrix         = FMatrix::Identity;
	FMatrix GeometryMatrix       = FMatrix::Identity;
	FMatrix GlobalGeometryMatrix = FMatrix::Identity;

	friend FArchive& operator<<(FArchive& Ar, FFbxImportedSceneNode& Node)
	{
		Ar << Node.Name;
		Ar << Node.NodePath;
		Ar << Node.AttributeType;
		Ar << Node.ParentIndex;
		Ar << Node.SourceLODIndex;
		Ar << Node.bHasMesh;
		Ar << Node.bHasSkeleton;
		Ar << Node.bIsSocket;
		Ar << Node.bIsCollision;
		MeshSerializationUtils::SerializeMatrix(Ar, Node.LocalMatrix);
		MeshSerializationUtils::SerializeMatrix(Ar, Node.GlobalMatrix);
		MeshSerializationUtils::SerializeMatrix(Ar, Node.GeometryMatrix);
		MeshSerializationUtils::SerializeMatrix(Ar, Node.GlobalGeometryMatrix);
		return Ar;
	}
};

// Cooked Data 내부용 정점
struct FNormalVertex
{
	FVector pos;
	FVector normal;
	FVector4 color;
	FVector2 tex;
	// TTT FBX importer preserves up to four UV channels; renderer still consumes UV0.
	FVector2 ExtraUV[MAX_STATIC_MESH_UV_CHANNELS - 1] = {};
	uint8    NumUVs                                   = 1;
	FVector4 tangent;
};


struct FStaticMeshSection
{
	int32 MaterialIndex = -1; // Index into UStaticMesh's FStaticMaterial array. Cached to avoid per-frame string comparison.
	FString MaterialSlotName;
	uint32 FirstIndex;
	uint32 NumTriangles;

	friend FArchive& operator<<(FArchive& Ar, FStaticMeshSection& Section)
	{
		Ar << Section.MaterialSlotName << Section.FirstIndex << Section.NumTriangles;
		return Ar;
	}
};

struct FStaticMaterial
{
	// std::shared_ptr<class UMaterialInterface> MaterialInterface;
	UMaterial* MaterialInterface = nullptr;
	FString MaterialSlotName = "None";	// "None"은 특별한 슬롯 이름으로, OBJ 파일에서 머티리얼이 지정되지 않은 섹션에 할당됩니다.
	FString MaterialPath;				// .mat 경로

	friend FArchive& operator<<(FArchive& Ar, FStaticMaterial& Mat)
	{
		// 1. 슬롯 이름 직렬화 (메시 섹션과 매핑용)
		Ar << Mat.MaterialSlotName;

		// 2. Material 포인터는 실행마다 달라질 수 있다.
		// .statbin에는 다시 찾을 수 있는 .mat 경로만 저장한다.
		if (Ar.IsSaving() && Mat.MaterialInterface)
		{
			Mat.MaterialPath = Mat.MaterialInterface->GetAssetPathFileName();
		}
		Ar << Mat.MaterialPath;

		// 3. 로딩 시 FMaterialManager를 통해 머티리얼 복원
		if (Ar.IsLoading())
		{
			if (!Mat.MaterialPath.empty())
			{
				Mat.MaterialInterface = FMaterialManager::Get().GetOrCreateMaterial(Mat.MaterialPath);
			}
			else
			{
				Mat.MaterialInterface = nullptr;
			}
		}

		return Ar;
	}
};

struct FStaticMeshLOD
{
	int32                      SourceLODIndex = 0;
	FString                    SourceLODName;
	float                      ScreenSize        = 1.0f;
	float                      DistanceThreshold = 0.0f;
	TArray<FNormalVertex>      Vertices;
	TArray<uint32>             Indices;
	TArray<FStaticMeshSection> Sections;
	FVector                    BoundsCenter = FVector(0.0f, 0.0f, 0.0f);
	FVector                    BoundsExtent = FVector(0.0f, 0.0f, 0.0f);
	bool                       bBoundsValid = false;

	void CacheBounds()
	{
		bBoundsValid = false;
		if (Vertices.empty()) return;
		FVector LocalMin = Vertices[0].pos;
		FVector LocalMax = Vertices[0].pos;
		for (const FNormalVertex& V : Vertices)
		{
			LocalMin.X = (std::min)(LocalMin.X, V.pos.X);
			LocalMin.Y = (std::min)(LocalMin.Y, V.pos.Y);
			LocalMin.Z = (std::min)(LocalMin.Z, V.pos.Z);
			LocalMax.X = (std::max)(LocalMax.X, V.pos.X);
			LocalMax.Y = (std::max)(LocalMax.Y, V.pos.Y);
			LocalMax.Z = (std::max)(LocalMax.Z, V.pos.Z);
		}
		BoundsCenter = (LocalMin + LocalMax) * 0.5f;
		BoundsExtent = (LocalMax - LocalMin) * 0.5f;
		bBoundsValid = true;
	}

	friend FArchive& operator<<(FArchive& Ar, FStaticMeshLOD& LOD)
	{
		Ar << LOD.SourceLODIndex;
		Ar << LOD.SourceLODName;
		Ar << LOD.ScreenSize;
		Ar << LOD.DistanceThreshold;
		Ar << LOD.Vertices;
		Ar << LOD.Indices;
		Ar << LOD.Sections;
		if (Ar.IsLoading()) LOD.CacheBounds();
		return Ar;
	}
};

// Cooked Data — GPU용 정점/인덱스
// FStaticMeshLODResources in UE5
struct FStaticMesh
{
	FString PathFileName;
	TArray<FNormalVertex> Vertices;
	TArray<uint32> Indices;

	TArray<FStaticMeshSection> Sections;

	TArray<FStaticMeshLOD>           LODModels;
	TArray<FImportedCollisionShape>  CollisionShapes;
	TArray<FFbxImportedNodeMetadata> NodeMetadata;
	TArray<FFbxImportedSceneNode>    SceneNodes;

	std::unique_ptr<FMeshBuffer> RenderBuffer;

	// 메시 로컬 바운드 캐시 (정점 순회 1회로 계산)
	FVector BoundsCenter = FVector(0, 0, 0);
	FVector BoundsExtent = FVector(0, 0, 0);
	bool    bBoundsValid = false;

	void CacheBounds()
	{
		bBoundsValid = false;
		if (Vertices.empty()) return;

		FVector LocalMin = Vertices[0].pos;
		FVector LocalMax = Vertices[0].pos;
		for (const FNormalVertex& V : Vertices)
		{
			LocalMin.X = (std::min)(LocalMin.X, V.pos.X);
			LocalMin.Y = (std::min)(LocalMin.Y, V.pos.Y);
			LocalMin.Z = (std::min)(LocalMin.Z, V.pos.Z);
			LocalMax.X = (std::max)(LocalMax.X, V.pos.X);
			LocalMax.Y = (std::max)(LocalMax.Y, V.pos.Y);
			LocalMax.Z = (std::max)(LocalMax.Z, V.pos.Z);
		}

		BoundsCenter = (LocalMin + LocalMax) * 0.5f;
		BoundsExtent = (LocalMax - LocalMin) * 0.5f;
		bBoundsValid = true;
	}

	void Serialize(FArchive& Ar)
	{
		Ar << PathFileName;
		Ar << Vertices;
		Ar << Indices;
		Ar << Sections;
		Ar << LODModels;
		Ar << CollisionShapes;
		Ar << NodeMetadata;
		Ar << SceneNodes;
		if (Ar.IsLoading())
		{
			CacheBounds();
		}
	}
};
