#include "Mesh/Fbx/FbxSkinWeightImporter.h"

#include "Mesh/Fbx/FbxGeometryReader.h"
#include "Mesh/Fbx/FbxSceneQuery.h"
#include "Mesh/Fbx/FbxSectionBuilder.h"
#include "Mesh/Fbx/FbxSkeletonImporter.h"
#include "Mesh/Fbx/FbxTransformUtils.h"
#include "Mesh/Fbx/FbxVertexDeduplicator.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
	struct FWeightData
	{
		int32 BoneIndex = -1;
		float Weight = 0.0f;
	};

	static void NormalizeWeights(float* Weights, int32 Count)
	{
		float TotalWeight = 0.0f;
		for (int32 Index = 0; Index < Count; ++Index)
		{
			TotalWeight += Weights[Index];
		}

		if (TotalWeight > 0.0f)
		{
			for (int32 Index = 0; Index < Count; ++Index)
			{
				Weights[Index] /= TotalWeight;
			}
		}
	}

	static int32 ResolveGlobalMaterialIndex(FbxNode* Node, FbxMesh* Mesh, int32 PolygonIndex, const FFbxImportContext& Context)
	{
		FbxSurfaceMaterial* Material = FFbxMaterialSlotResolver::ResolvePolygonMaterial(Node, Mesh, PolygonIndex);
		auto MaterialIt = Context.MaterialToSlotIndex.find(Material);
		return (MaterialIt != Context.MaterialToSlotIndex.end()) ? MaterialIt->second : -1;
	}

	static TArray<FString> CollectImportedMaterialSlotNames(const TArray<FFbxImportedMaterialInfo>& Materials)
	{
		TArray<FString> SlotNames;
		SlotNames.reserve(Materials.size());
		for (const FFbxImportedMaterialInfo& Material : Materials)
		{
			SlotNames.push_back(Material.Name);
		}
		return SlotNames;
	}

	static FVertexPNCTBW MakeSkeletalVertex(
		FbxMesh* Mesh,
		int32 PolygonIndex,
		int32 CornerIndex,
		int32 PolygonVertexIndex,
		const FFbxTriangleSample& Triangle,
		const TArray<FWeightData>& Weights
		)
	{
		FVertexPNCTBW Vertex;
		const int32 ControlPointIndex = Triangle.ControlPointIndices[CornerIndex];
		Vertex.Position = FFbxGeometryReader::ReadPosition(Mesh, ControlPointIndex);

		FVector Normal;
		if (FFbxGeometryReader::TryReadNormal(Mesh, PolygonIndex, CornerIndex, Normal))
		{
			Vertex.Normal = Normal;
		}
		else
		{
			Vertex.Normal = Triangle.FallbackNormal;
		}
		if (Vertex.Normal.IsNearlyZero())
		{
			Vertex.Normal = FVector::UpVector;
		}

		Vertex.Color = FFbxGeometryReader::ReadVertexColor(Mesh, ControlPointIndex, PolygonVertexIndex);
		Vertex.UV = FFbxGeometryReader::ReadUVByChannel(Mesh, PolygonIndex, CornerIndex, 0);

		FVector4 ImportedTangent;
		if (FFbxGeometryReader::TryReadTangent(Mesh, ControlPointIndex, PolygonVertexIndex, ImportedTangent))
		{
			FVector Tangent(ImportedTangent.X, ImportedTangent.Y, ImportedTangent.Z);
			Tangent = FFbxTransformUtils::OrthogonalizeTangentToNormal(Tangent, Vertex.Normal);
			Vertex.Tangent = FVector4(Tangent.X, Tangent.Y, Tangent.Z, ImportedTangent.W);
		}
		else
		{
			FVector Tangent = FFbxTransformUtils::OrthogonalizeTangentToNormal(Triangle.FallbackTangent, Vertex.Normal);
			Vertex.Tangent = FVector4(Tangent.X, Tangent.Y, Tangent.Z, 1.0f);
		}

		for (int32 WeightIndex = 0; WeightIndex < static_cast<int32>(Weights.size()) && WeightIndex < 4; ++WeightIndex)
		{
			Vertex.BoneIndices[WeightIndex] = Weights[WeightIndex].BoneIndex;
			Vertex.BoneWeights[WeightIndex] = Weights[WeightIndex].Weight;
		}
		NormalizeWeights(Vertex.BoneWeights, 4);
		return Vertex;
	}
}

bool FFbxSkinWeightImporter::ImportSkin(FbxScene* Scene, FFbxImportContext& Context, FString* OutMessage)
{
	(void)Scene;
	Context.SkeletalVertices.clear();
	Context.SkeletalIndices.clear();
	Context.SkeletalSections.clear();
	Context.SkeletalMeshRanges.clear();
	Context.TangentSums.clear();
	Context.BitangentSums.clear();
	Context.MorphSourcesByLOD.clear();
	Context.MorphSourcesByLOD.resize(1);

	TArray<FbxNode*> BaseRenderMeshNodes;
	int32            BaseLODIndex = std::numeric_limits<int32>::max();
	for (FbxNode* MeshNode : Context.MeshNodes)
	{
		if (!MeshNode || FFbxSceneQuery::IsCollisionProxyNode(MeshNode))
		{
			continue;
		}

		FbxMesh* CandidateMesh = MeshNode->GetMesh();
		if (CandidateMesh && !FFbxSceneQuery::MeshHasSkin(CandidateMesh))
		{
			FbxNode* ParentBoneNode = nullptr;
			int32 ParentBoneIndex = -1;
			if (!FFbxSceneQuery::FindNearestParentBoneIndex(MeshNode, Context.BoneNodeToIndex, ParentBoneNode, ParentBoneIndex))
			{
				continue;
			}
		}

		const int32 LODIndex = FFbxSceneQuery::GetMeshLODIndex(MeshNode);
		if (LODIndex < BaseLODIndex)
		{
			BaseLODIndex = LODIndex;
			BaseRenderMeshNodes.clear();
		}
		if (LODIndex == BaseLODIndex)
		{
			BaseRenderMeshNodes.push_back(MeshNode);
		}
	}

	for (FbxNode* Node : Context.AllNodes)
	{
		FbxMesh* Mesh = Node ? Node->GetMesh() : nullptr;
		if (!Mesh)
		{
			continue;
		}

		if (FFbxSceneQuery::IsCollisionProxyNode(Node) || !FFbxSceneQuery::ContainsNode(BaseRenderMeshNodes, Node))
		{
			if (FFbxSceneQuery::IsCollisionProxyNode(Node))
			{
				Context.AddWarningOnce(ESkeletalImportWarningType::CollisionProxySkippedFromRenderLOD, "Collision proxy mesh was skipped from skeletal render geometry.");
			}
			continue;
		}

		const int32 DeformerCount = Mesh->GetDeformerCount(FbxDeformer::eSkin);
		FbxSkin* Skin = DeformerCount > 0 ? static_cast<FbxSkin*>(Mesh->GetDeformer(0, FbxDeformer::eSkin)) : nullptr;
		const int32 ClusterCount = Skin ? Skin->GetClusterCount() : 0;
		const bool bHasSkin = Skin && ClusterCount > 0;
		const int32 RigidBoneIndex = bHasSkin ? -1 : FFbxSkeletonImporter::FindNearestParentBoneIndex(Node, Context.BoneNodeToIndex);

		TArray<TArray<FWeightData>> TempWeights(Mesh->GetControlPointsCount());
		FbxAMatrix NodeGeometryTransform = FFbxTransformUtils::GetGeometryTransform(Node);
		FMatrix MeshBindGlobal = FFbxTransformUtils::ToEngineMatrix(Node->EvaluateGlobalTransform() * NodeGeometryTransform);
		bool bHasClusterMeshBindGlobal = false;

		for (int32 ClusterIndex = 0; ClusterIndex < ClusterCount; ++ClusterIndex)
		{
			FbxCluster* Cluster = Skin->GetCluster(ClusterIndex);
			if (!Cluster || !Cluster->GetLink())
			{
				continue;
			}

			auto BoneIt = Context.BoneNodeToIndex.find(Cluster->GetLink());
			if (BoneIt == Context.BoneNodeToIndex.end())
			{
				continue;
			}

			FbxAMatrix LinkBindMatrix;
			Cluster->GetTransformLinkMatrix(LinkBindMatrix);
			const int32 BoneIndex = BoneIt->second;
			Context.Bones[BoneIndex].InverseBindPoseMatrix = FFbxTransformUtils::ToEngineMatrix(LinkBindMatrix).GetInverse();

			if (!bHasClusterMeshBindGlobal)
			{
				FbxAMatrix MeshBindMatrix;
				Cluster->GetTransformMatrix(MeshBindMatrix);
				MeshBindGlobal = FFbxTransformUtils::ToEngineMatrix(MeshBindMatrix);
				bHasClusterMeshBindGlobal = true;
			}

			int32* ControlPointIndices = Cluster->GetControlPointIndices();
			double* ControlPointWeights = Cluster->GetControlPointWeights();
			const int32 NumIndices = Cluster->GetControlPointIndicesCount();
			if (!ControlPointIndices || !ControlPointWeights || NumIndices <= 0)
			{
				continue;
			}

			for (int32 ControlPointWeightIndex = 0; ControlPointWeightIndex < NumIndices; ++ControlPointWeightIndex)
			{
				const int32 ControlPointIndex = ControlPointIndices[ControlPointWeightIndex];
				if (!FFbxSceneQuery::IsValidControlPointIndex(Mesh, ControlPointIndex))
				{
					continue;
				}

				const float Weight = static_cast<float>(ControlPointWeights[ControlPointWeightIndex]);
				if (Weight <= 0.0f)
				{
					continue;
				}

				TempWeights[ControlPointIndex].push_back({ BoneIndex, Weight });
			}
		}

		for (TArray<FWeightData>& Weights : TempWeights)
		{
			if (Weights.empty() && RigidBoneIndex >= 0)
			{
				Weights.push_back({ RigidBoneIndex, 1.0f });
			}

			if (Weights.empty() && !Context.Bones.empty())
			{
				Weights.push_back({ 0, 1.0f });
				Context.AddWarningOnce(ESkeletalImportWarningType::MissingSkinWeight, "One or more vertices had no skin weights and were assigned to root bone.");
			}

			std::sort(Weights.begin(), Weights.end(), [](const FWeightData& A, const FWeightData& B)
			{
				return A.Weight > B.Weight;
			});

			if (Weights.size() > 4)
			{
				Weights.resize(4);
				Context.AddWarningOnce(ESkeletalImportWarningType::MoreThanFourInfluences, "One or more vertices had more than four bone influences; strongest four were kept.");
			}
		}

		FFbxSectionBuilder SectionBuilds;
		FFbxSkeletalVertexDeduplicator Deduplicator;
		const uint32 VertexStart = static_cast<uint32>(Context.SkeletalVertices.size());
		const uint32 FirstIndex = static_cast<uint32>(Context.SkeletalIndices.size());
		const size_t MorphSourceStartIndex = Context.MorphSourcesByLOD.empty() ? 0 : Context.MorphSourcesByLOD[0].size();
		int32 PolygonVertexIndex = 0;

		for (int32 PolygonIndex = 0; PolygonIndex < Mesh->GetPolygonCount(); ++PolygonIndex)
		{
			const int32 PolygonSize = Mesh->GetPolygonSize(PolygonIndex);
			if (PolygonSize != 3)
			{
				PolygonVertexIndex += PolygonSize;
				continue;
			}

			FFbxTriangleSample Triangle;
			if (!FFbxGeometryReader::ReadTriangleSample(Mesh, PolygonIndex, FFbxMeshImportSpace::IdentitySpace(), Triangle))
			{
				PolygonVertexIndex += PolygonSize;
				continue;
			}

			const int32 MaterialIndex = ResolveGlobalMaterialIndex(Node, Mesh, PolygonIndex, Context);
			FFbxImportedSectionBuild* Section = SectionBuilds.FindOrAddSection(MaterialIndex);
			if (!Section)
			{
				PolygonVertexIndex += PolygonSize;
				continue;
			}
			uint32 TriIndices[3] = {};
			bool bValidTriangle = true;
			TArray<FFbxImportedMorphSourceVertex> PendingMorphSources;

			for (int32 CornerIndex = 0; CornerIndex < 3; ++CornerIndex)
			{
				const int32 ControlPointIndex = Triangle.ControlPointIndices[CornerIndex];
				const int32 CurrentPolygonVertexIndex = PolygonVertexIndex + CornerIndex;
				if (!FFbxSceneQuery::IsValidControlPointIndex(Mesh, ControlPointIndex))
				{
					bValidTriangle = false;
					break;
				}

				const TArray<FWeightData>& Weights = TempWeights[ControlPointIndex];
				FVertexPNCTBW Vertex = MakeSkeletalVertex(Mesh, PolygonIndex, CornerIndex, CurrentPolygonVertexIndex, Triangle, Weights);
				bool bAddedNewVertex = false;
				const uint32 VertexIndex = Deduplicator.FindOrAdd(Vertex, Mesh, ControlPointIndex, MaterialIndex, Context.SkeletalVertices, bAddedNewVertex);
				TriIndices[CornerIndex] = VertexIndex;

				if (!Context.MorphSourcesByLOD.empty())
				{
					FFbxImportedMorphSourceVertex MorphSource;
					MorphSource.MeshNode = Node;
					MorphSource.Mesh = Mesh;
					MorphSource.SourceMeshNodeName = Node && Node->GetName() ? FString(Node->GetName()) : FString();
					MorphSource.ControlPointIndex = ControlPointIndex;
					MorphSource.PolygonIndex = PolygonIndex;
					MorphSource.CornerIndex = CornerIndex;
					MorphSource.PolygonVertexIndex = CurrentPolygonVertexIndex;
					MorphSource.VertexIndex = VertexIndex;
					MorphSource.MeshToReference = FMatrix::Identity;
					MorphSource.NormalToReference = FMatrix::Identity;
					MorphSource.BaseNormalInReference = Vertex.Normal;
					MorphSource.BaseTangentInReference = Vertex.Tangent;
					PendingMorphSources.push_back(MorphSource);
				}
			}

			if (bValidTriangle)
			{
				Section->Indices.push_back(TriIndices[0]);
				Section->Indices.push_back(TriIndices[1]);
				Section->Indices.push_back(TriIndices[2]);
				if (!Context.MorphSourcesByLOD.empty())
				{
					Context.MorphSourcesByLOD[0].insert(Context.MorphSourcesByLOD[0].end(), PendingMorphSources.begin(), PendingMorphSources.end());
				}
			}

			PolygonVertexIndex += PolygonSize;
		}

		if (!Context.MorphSourcesByLOD.empty())
		{
			TArray<FFbxImportedMorphSourceVertex>& MorphSources = Context.MorphSourcesByLOD[0];
			for (size_t MorphIndex = MorphSourceStartIndex; MorphIndex < MorphSources.size(); ++MorphIndex)
			{
				FFbxImportedMorphSourceVertex& Source = MorphSources[MorphIndex];
				if (Source.VertexIndex < Context.SkeletalVertices.size())
				{
					const FVertexPNCTBW& FinalVertex = Context.SkeletalVertices[Source.VertexIndex];
					Source.BaseNormalInReference = FinalVertex.Normal;
					Source.BaseTangentInReference = FinalVertex.Tangent;
				}
			}
		}

		if (!SectionBuilds.IsEmpty())
		{
			bool bNeedsNoneMaterialSlot = false;
			const TArray<FString> MaterialSlotNames = CollectImportedMaterialSlotNames(Context.Materials);
			SectionBuilds.AppendFinalSkeletalSections(MaterialSlotNames, Context.SkeletalIndices, Context.SkeletalSections, &bNeedsNoneMaterialSlot);
			if (bNeedsNoneMaterialSlot)
			{
				Context.AddWarningOnce(ESkeletalImportWarningType::UnsupportedMaterialProperty, "One or more skeletal polygons had no valid FBX material and were assigned to None slot.");
			}
		}

		FSkeletalMeshRange MeshRange;
		MeshRange.VertexStart = VertexStart;
		MeshRange.VertexEnd = static_cast<uint32>(Context.SkeletalVertices.size());
		MeshRange.FirstIndex = FirstIndex;
		MeshRange.IndexCount = static_cast<uint32>(Context.SkeletalIndices.size()) - FirstIndex;
		MeshRange.MeshBindGlobal = MeshBindGlobal;
		if (MeshRange.VertexStart < MeshRange.VertexEnd && MeshRange.IndexCount > 0)
		{
			Context.SkeletalMeshRanges.push_back(MeshRange);
		}
	}

	const bool bImportedAnyGeometry = !Context.SkeletalVertices.empty() && !Context.SkeletalIndices.empty();
	if (!bImportedAnyGeometry && OutMessage)
	{
		*OutMessage = "FBX skeletal import failed: no skinned geometry imported.";
	}
	return bImportedAnyGeometry;
}
