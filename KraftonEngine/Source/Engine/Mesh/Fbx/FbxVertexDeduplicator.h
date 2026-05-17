#pragma once

#include "Core/CoreTypes.h"
#include "Mesh/StaticMeshAsset.h"
#include "Mesh/SkeletalMeshAsset.h"
#include "Render/Types/VertexTypes.h"

#include <fbxsdk.h>
#include <array>
#include <cstddef>
#include <cstring>
#include <functional>
#include <unordered_map>

namespace FbxVertexDedupInternal
{
	inline uint32 FloatToStableBits(float Value)
	{
		if (Value == 0.0f)
		{
			Value = 0.0f;
		}

		uint32 Bits = 0;
		static_assert(sizeof(Bits) == sizeof(Value), "float and uint32 size mismatch");
		std::memcpy(&Bits, &Value, sizeof(float));
		return Bits;
	}

	inline void HashCombineSizeT(std::size_t& Seed, std::size_t Value)
	{
		Seed ^= Value + 0x9e3779b97f4a7c15ull + (Seed << 6) + (Seed >> 2);
	}

	inline void HashCombineUInt32(std::size_t& Seed, uint32 Value)
	{
		HashCombineSizeT(Seed, static_cast<std::size_t>(Value));
	}

	inline void HashCombineInt32(std::size_t& Seed, int32 Value)
	{
		HashCombineSizeT(Seed, static_cast<std::size_t>(static_cast<uint32>(Value)));
	}

	inline void HashCombinePointer(std::size_t& Seed, const void* Pointer)
	{
		HashCombineSizeT(Seed, std::hash<const void*>{}(Pointer));
	}

	inline std::array<uint32, 2> MakeVector2Bits(const FVector2& V)
	{
		return { FloatToStableBits(V.X), FloatToStableBits(V.Y) };
	}

	inline std::array<uint32, 3> MakeVector3Bits(const FVector& V)
	{
		return { FloatToStableBits(V.X), FloatToStableBits(V.Y), FloatToStableBits(V.Z) };
	}

	inline std::array<uint32, 4> MakeVector4Bits(const FVector4& V)
	{
		return { FloatToStableBits(V.X), FloatToStableBits(V.Y), FloatToStableBits(V.Z), FloatToStableBits(V.W) };
	}
}

struct FFbxStaticVertexDedupKey
{
	const FbxMesh* Mesh              = nullptr;
	int32          ControlPointIndex = -1;
	int32          MaterialIndex     = -1;

	std::array<uint32, 3>                                          Position = {};
	std::array<uint32, 3>                                          Normal   = {};
	std::array<std::array<uint32, 2>, MAX_STATIC_MESH_UV_CHANNELS> UV       = {};
	uint8                                                          NumUV    = 1;
	std::array<uint32, 4>                                          Color    = {};
	std::array<uint32, 4>                                          Tangent  = {};

	bool operator==(const FFbxStaticVertexDedupKey& Other) const
	{
		return Mesh == Other.Mesh
			&& ControlPointIndex == Other.ControlPointIndex
			&& MaterialIndex == Other.MaterialIndex
			&& Position == Other.Position
			&& Normal == Other.Normal
			&& UV == Other.UV
			&& NumUV == Other.NumUV
			&& Color == Other.Color
			&& Tangent == Other.Tangent;
	}
};

struct FFbxStaticVertexDedupKeyHasher
{
	std::size_t operator()(const FFbxStaticVertexDedupKey& Key) const
	{
		std::size_t Seed = 0;
		FbxVertexDedupInternal::HashCombinePointer(Seed, Key.Mesh);
		FbxVertexDedupInternal::HashCombineInt32(Seed, Key.ControlPointIndex);
		FbxVertexDedupInternal::HashCombineInt32(Seed, Key.MaterialIndex);
		for (uint32 Value : Key.Position) FbxVertexDedupInternal::HashCombineUInt32(Seed, Value);
		for (uint32 Value : Key.Normal) FbxVertexDedupInternal::HashCombineUInt32(Seed, Value);
		for (const std::array<uint32, 2>& UV : Key.UV)
		{
			FbxVertexDedupInternal::HashCombineUInt32(Seed, UV[0]);
			FbxVertexDedupInternal::HashCombineUInt32(Seed, UV[1]);
		}
		FbxVertexDedupInternal::HashCombineInt32(Seed, static_cast<int32>(Key.NumUV));
		for (uint32 Value : Key.Color) FbxVertexDedupInternal::HashCombineUInt32(Seed, Value);
		for (uint32 Value : Key.Tangent) FbxVertexDedupInternal::HashCombineUInt32(Seed, Value);
		return Seed;
	}
};

class FFbxStaticVertexDeduplicator
{
public:
	uint32 FindOrAdd(
		const FNormalVertex&   Vertex,
		const FbxMesh*         Mesh,
		int32                  ControlPointIndex,
		int32                  MaterialIndex,
		TArray<FNormalVertex>& OutVertices,
		bool&                  bOutAddedNewVertex
		)
	{
		FFbxStaticVertexDedupKey Key;
		Key.Mesh              = Mesh;
		Key.ControlPointIndex = ControlPointIndex;
		Key.MaterialIndex     = MaterialIndex;
		Key.Position          = FbxVertexDedupInternal::MakeVector3Bits(Vertex.pos);
		Key.Normal            = FbxVertexDedupInternal::MakeVector3Bits(Vertex.normal);
		Key.NumUV             = Vertex.NumUVs;
		Key.UV[0]             = FbxVertexDedupInternal::MakeVector2Bits(Vertex.tex);
		for (int32 UVIndex = 1; UVIndex < static_cast<int32>(Vertex.NumUVs) && UVIndex < MAX_STATIC_MESH_UV_CHANNELS; ++UVIndex)
		{
			Key.UV[UVIndex] = FbxVertexDedupInternal::MakeVector2Bits(Vertex.ExtraUV[UVIndex - 1]);
		}
		Key.Color   = FbxVertexDedupInternal::MakeVector4Bits(Vertex.color);
		Key.Tangent = FbxVertexDedupInternal::MakeVector4Bits(Vertex.tangent);

		auto It = VertexToIndex.find(Key);
		if (It != VertexToIndex.end())
		{
			bOutAddedNewVertex = false;
			return It->second;
		}

		const uint32 NewIndex = static_cast<uint32>(OutVertices.size());
		OutVertices.push_back(Vertex);
		VertexToIndex.emplace(Key, NewIndex);
		bOutAddedNewVertex = true;
		return NewIndex;
	}

private:
	std::unordered_map<FFbxStaticVertexDedupKey, uint32, FFbxStaticVertexDedupKeyHasher> VertexToIndex;
};

struct FFbxSkeletalVertexDedupKey
{
	const FbxMesh* Mesh              = nullptr;
	int32          ControlPointIndex = -1;
	int32          MaterialIndex     = -1;

	std::array<uint32, 3> Position = {};
	std::array<uint32, 3> Normal   = {};
	std::array<uint32, 2> UV       = {};
	std::array<uint32, 4> Color    = {};
	std::array<uint32, 4> Tangent  = {};
	std::array<int32, 4>  BoneIndices = {};
	std::array<uint32, 4> BoneWeights = {};

	bool operator==(const FFbxSkeletalVertexDedupKey& Other) const
	{
		return Mesh == Other.Mesh
			&& ControlPointIndex == Other.ControlPointIndex
			&& MaterialIndex == Other.MaterialIndex
			&& Position == Other.Position
			&& Normal == Other.Normal
			&& UV == Other.UV
			&& Color == Other.Color
			&& Tangent == Other.Tangent
			&& BoneIndices == Other.BoneIndices
			&& BoneWeights == Other.BoneWeights;
	}
};

struct FFbxSkeletalVertexDedupKeyHasher
{
	std::size_t operator()(const FFbxSkeletalVertexDedupKey& Key) const
	{
		std::size_t Seed = 0;
		FbxVertexDedupInternal::HashCombinePointer(Seed, Key.Mesh);
		FbxVertexDedupInternal::HashCombineInt32(Seed, Key.ControlPointIndex);
		FbxVertexDedupInternal::HashCombineInt32(Seed, Key.MaterialIndex);
		for (uint32 Value : Key.Position) FbxVertexDedupInternal::HashCombineUInt32(Seed, Value);
		for (uint32 Value : Key.Normal) FbxVertexDedupInternal::HashCombineUInt32(Seed, Value);
		for (uint32 Value : Key.UV) FbxVertexDedupInternal::HashCombineUInt32(Seed, Value);
		for (uint32 Value : Key.Color) FbxVertexDedupInternal::HashCombineUInt32(Seed, Value);
		for (uint32 Value : Key.Tangent) FbxVertexDedupInternal::HashCombineUInt32(Seed, Value);
		for (int32 Value : Key.BoneIndices) FbxVertexDedupInternal::HashCombineInt32(Seed, Value);
		for (uint32 Value : Key.BoneWeights) FbxVertexDedupInternal::HashCombineUInt32(Seed, Value);
		return Seed;
	}
};

class FFbxSkeletalVertexDeduplicator
{
public:
	uint32 FindOrAdd(
		const FVertexPNCTBW&   Vertex,
		const FbxMesh*         Mesh,
		int32                  ControlPointIndex,
		int32                  MaterialIndex,
		TArray<FVertexPNCTBW>& OutVertices,
		bool&                  bOutAddedNewVertex
		)
	{
		FFbxSkeletalVertexDedupKey Key;
		Key.Mesh              = Mesh;
		Key.ControlPointIndex = ControlPointIndex;
		Key.MaterialIndex     = MaterialIndex;
		Key.Position          = FbxVertexDedupInternal::MakeVector3Bits(Vertex.Position);
		Key.Normal            = FbxVertexDedupInternal::MakeVector3Bits(Vertex.Normal);
		Key.UV                = FbxVertexDedupInternal::MakeVector2Bits(Vertex.UV);
		Key.Color             = FbxVertexDedupInternal::MakeVector4Bits(Vertex.Color);
		Key.Tangent           = FbxVertexDedupInternal::MakeVector4Bits(Vertex.Tangent);
		for (int32 InfluenceIndex = 0; InfluenceIndex < 4; ++InfluenceIndex)
		{
			Key.BoneIndices[InfluenceIndex] = Vertex.BoneIndices[InfluenceIndex];
			Key.BoneWeights[InfluenceIndex] = FbxVertexDedupInternal::FloatToStableBits(Vertex.BoneWeights[InfluenceIndex]);
		}

		auto It = VertexToIndex.find(Key);
		if (It != VertexToIndex.end())
		{
			bOutAddedNewVertex = false;
			return It->second;
		}

		const uint32 NewIndex = static_cast<uint32>(OutVertices.size());
		OutVertices.push_back(Vertex);
		VertexToIndex.emplace(Key, NewIndex);
		bOutAddedNewVertex = true;
		return NewIndex;
	}

private:
	std::unordered_map<FFbxSkeletalVertexDedupKey, uint32, FFbxSkeletalVertexDedupKeyHasher> VertexToIndex;
};
