#include "Mesh/Fbx/FbxStaticMeshImporter.h"

#include "Mesh/Fbx/FbxCollisionImporter.h"
#include "Mesh/Fbx/FbxGeometryReader.h"
#include "Mesh/Fbx/FbxMaterialImporter.h"
#include "Mesh/Fbx/FbxMetadataImporter.h"
#include "Mesh/Fbx/FbxSceneHierarchyImporter.h"
#include "Mesh/Fbx/FbxSceneQuery.h"
#include "Mesh/Fbx/FbxSectionBuilder.h"
#include "Mesh/Fbx/FbxTransformUtils.h"
#include "Mesh/Fbx/FbxVertexDeduplicator.h"
#include "Mesh/MeshImportOptions.h"
#include "Materials/MaterialManager.h"

#include <algorithm>
#include <limits>
#include <map>
#include <utility>

namespace
{
	static int32 ResolveGlobalMaterialIndex(FbxNode* Node, FbxMesh* Mesh, int32 PolygonIndex, const FFbxImportContext& Context)
	{
		FbxSurfaceMaterial* Material = FFbxMaterialSlotResolver::ResolvePolygonMaterial(Node, Mesh, PolygonIndex);
		auto It = Context.MaterialToSlotIndex.find(Material);
		return (It != Context.MaterialToSlotIndex.end()) ? It->second : -1;
	}

	static TArray<FString> CollectStaticMaterialSlotNames(const TArray<FStaticMaterial>& Materials)
	{
		TArray<FString> SlotNames;
		SlotNames.reserve(Materials.size());
		for (const FStaticMaterial& Material : Materials)
		{
			SlotNames.push_back(Material.MaterialSlotName);
		}
		return SlotNames;
	}

	static FNormalVertex MakeStaticVertex(
		FbxMesh* Mesh,
		int32 PolygonIndex,
		int32 CornerIndex,
		int32 PolygonVertexIndex,
		const FFbxMeshImportSpace& ImportSpace,
		const FFbxTriangleSample& Triangle
		)
	{
		FNormalVertex Vertex;
		const int32 ControlPointIndex = Triangle.ControlPointIndices[CornerIndex];
		Vertex.pos = Triangle.Positions[CornerIndex];

		FVector Normal;
		if (FFbxGeometryReader::TryReadNormal(Mesh, PolygonIndex, CornerIndex, Normal))
		{
			Vertex.normal = FFbxTransformUtils::TransformNormalByMatrix(Normal, ImportSpace.NormalTransform);
		}
		else
		{
			Vertex.normal = Triangle.FallbackNormal;
		}
		if (Vertex.normal.IsNearlyZero())
		{
			Vertex.normal = FVector::UpVector;
		}

		Vertex.NumUVs = static_cast<uint8>((std::min)(FFbxGeometryReader::GetUVSetCount(Mesh), MAX_STATIC_MESH_UV_CHANNELS));
		if (Vertex.NumUVs == 0)
		{
			Vertex.NumUVs = 1;
		}
		Vertex.tex = FFbxGeometryReader::ReadUVByChannel(Mesh, PolygonIndex, CornerIndex, 0);
		for (int32 UVIndex = 1; UVIndex < static_cast<int32>(Vertex.NumUVs) && UVIndex < MAX_STATIC_MESH_UV_CHANNELS; ++UVIndex)
		{
			Vertex.ExtraUV[UVIndex - 1] = FFbxGeometryReader::ReadUVByChannel(Mesh, PolygonIndex, CornerIndex, UVIndex);
		}

		Vertex.color = FFbxGeometryReader::ReadVertexColor(Mesh, ControlPointIndex, PolygonVertexIndex);

		FVector4 ImportedTangent;
		if (FFbxGeometryReader::TryReadTangent(Mesh, ControlPointIndex, PolygonVertexIndex, ImportedTangent))
		{
			const FVector Tangent(ImportedTangent.X, ImportedTangent.Y, ImportedTangent.Z);
			const FVector ReferenceTangent = FFbxTransformUtils::TransformTangentByMatrix(Tangent, ImportSpace.VertexTransform, Vertex.normal);
			Vertex.tangent = FVector4(ReferenceTangent.X, ReferenceTangent.Y, ReferenceTangent.Z, ImportedTangent.W);
		}
		else
		{
			FVector FallbackTangent = FFbxTransformUtils::OrthogonalizeTangentToNormal(Triangle.FallbackTangent, Vertex.normal);
			Vertex.tangent = FVector4(FallbackTangent.X, FallbackTangent.Y, FallbackTangent.Z, 1.0f);
		}

		return Vertex;
	}

	static bool ImportLODFromNodes(
		const TArray<FbxNode*>& Nodes,
		int32 SourceLODIndex,
		const FFbxImportContext& Context,
		const FImportOptions& Options,
		const TArray<FStaticMaterial>& Materials,
		FStaticMeshLOD& OutLOD,
		bool& bInOutNeedsNoneSlot
		)
	{
		OutLOD = FStaticMeshLOD();
		OutLOD.SourceLODIndex = SourceLODIndex;
		OutLOD.SourceLODName = "LOD" + std::to_string(SourceLODIndex);

		FFbxSectionBuilder Sections;
		FFbxStaticVertexDeduplicator Deduplicator;

		for (FbxNode* Node : Nodes)
		{
			FbxMesh* Mesh = Node ? Node->GetMesh() : nullptr;
			if (!Mesh)
			{
				continue;
			}

			float ScreenSize = 1.0f;
			float DistanceThreshold = 0.0f;
			if (FFbxSceneQuery::TryGetLODSettings(Node, ScreenSize, DistanceThreshold))
			{
				OutLOD.ScreenSize = ScreenSize;
				OutLOD.DistanceThreshold = DistanceThreshold;
			}

			FFbxMeshImportSpace ImportSpace = FFbxMeshImportSpace::FromStaticMeshNode(Node);
			if (Options.StaticFbxSkinnedMeshPolicy == EStaticFbxSkinnedMeshPolicy::ImportBindPoseAsStatic)
			{
				const int32 SkinCount = Mesh->GetDeformerCount(FbxDeformer::eSkin);
				FbxSkin* Skin = SkinCount > 0 ? static_cast<FbxSkin*>(Mesh->GetDeformer(0, FbxDeformer::eSkin)) : nullptr;
				if (Skin && Skin->GetClusterCount() > 0)
				{
					FbxAMatrix MeshBindMatrix;
					Skin->GetCluster(0)->GetTransformMatrix(MeshBindMatrix);
					ImportSpace.VertexTransform = FFbxTransformUtils::ToEngineMatrix(MeshBindMatrix);
					ImportSpace.NormalTransform = ImportSpace.VertexTransform.GetInverse().GetTransposed();
				}
			}
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
				if (!FFbxGeometryReader::ReadTriangleSample(Mesh, PolygonIndex, ImportSpace, Triangle))
				{
					PolygonVertexIndex += PolygonSize;
					continue;
				}

				const int32 MaterialIndex = ResolveGlobalMaterialIndex(Node, Mesh, PolygonIndex, Context);
				FFbxImportedSectionBuild* Section = Sections.FindOrAddSection(MaterialIndex);
				if (!Section)
				{
					PolygonVertexIndex += PolygonSize;
					continue;
				}

				uint32 PendingIndices[3] = {};
				bool bValidTriangle = true;
				for (int32 CornerIndex = 0; CornerIndex < 3; ++CornerIndex)
				{
					const int32 ControlPointIndex = Triangle.ControlPointIndices[CornerIndex];
					if (!FFbxSceneQuery::IsValidControlPointIndex(Mesh, ControlPointIndex))
					{
						bValidTriangle = false;
						break;
					}

					FNormalVertex Vertex = MakeStaticVertex(Mesh, PolygonIndex, CornerIndex, PolygonVertexIndex + CornerIndex, ImportSpace, Triangle);
					bool bAddedNewVertex = false;
					PendingIndices[CornerIndex] = Deduplicator.FindOrAdd(Vertex, Mesh, ControlPointIndex, MaterialIndex, OutLOD.Vertices, bAddedNewVertex);
				}

				if (bValidTriangle)
				{
					Section->Indices.push_back(PendingIndices[0]);
					Section->Indices.push_back(PendingIndices[1]);
					Section->Indices.push_back(PendingIndices[2]);
				}

				PolygonVertexIndex += PolygonSize;
			}
		}

		if (Sections.IsEmpty())
		{
			return false;
		}

		const TArray<FString> MaterialSlotNames = CollectStaticMaterialSlotNames(Materials);
		Sections.BuildFinalStaticSections(MaterialSlotNames, OutLOD.Indices, OutLOD.Sections, &bInOutNeedsNoneSlot);
		OutLOD.CacheBounds();
		return !OutLOD.Vertices.empty() && !OutLOD.Indices.empty();
	}

	static void AddDefaultNoneMaterialIfNeeded(TArray<FStaticMaterial>& Materials, FStaticMesh& Mesh, bool bNeedsNoneSlot)
	{
		if (!bNeedsNoneSlot)
		{
			return;
		}

		int32 NoneMaterialIndex = -1;
		for (int32 MaterialIndex = 0; MaterialIndex < static_cast<int32>(Materials.size()); ++MaterialIndex)
		{
			if (Materials[MaterialIndex].MaterialSlotName == "None")
			{
				NoneMaterialIndex = MaterialIndex;
				break;
			}
		}

		if (NoneMaterialIndex < 0)
		{
			FStaticMaterial DefaultMaterial;
			DefaultMaterial.MaterialSlotName = "None";
			DefaultMaterial.MaterialInterface = FMaterialManager::Get().GetOrCreateMaterial("None");
			Materials.push_back(DefaultMaterial);
			NoneMaterialIndex = static_cast<int32>(Materials.size()) - 1;
		}

		for (FStaticMeshSection& Section : Mesh.Sections)
		{
			if (Section.MaterialSlotName == "None")
			{
				Section.MaterialIndex = NoneMaterialIndex;
			}
		}
		for (FStaticMeshLOD& LOD : Mesh.LODModels)
		{
			for (FStaticMeshSection& Section : LOD.Sections)
			{
				if (Section.MaterialSlotName == "None")
				{
					Section.MaterialIndex = NoneMaterialIndex;
				}
			}
		}
	}

	static FMatrix GetStaticCollisionLocalMatrix(FbxNode* CollisionNode)
	{
		if (!CollisionNode)
		{
			return FMatrix::Identity;
		}
		return FFbxTransformUtils::ToEngineMatrix(CollisionNode->EvaluateGlobalTransform() * FFbxTransformUtils::GetGeometryTransform(CollisionNode));
	}
}

bool FFbxStaticMeshImporter::Import(FbxScene* Scene, const FString& SourcePath, const FImportOptions* Options, FFbxImportContext& Context, FFbxStaticMeshImportResult& OutResult, FString* OutMessage)
{
	OutResult = FFbxStaticMeshImportResult();
	Context.SourcePath = SourcePath;
	Context.AllNodes.clear();
	Context.MeshNodes.clear();

	FbxNode* RootNode = Scene ? Scene->GetRootNode() : nullptr;
	if (!RootNode)
	{
		if (OutMessage) *OutMessage = "FBX static mesh import failed: root node not found.";
		return false;
	}

	FFbxSceneQuery::CollectAllNodes(RootNode, Context.AllNodes);
	FFbxSceneQuery::CollectMeshNodes(RootNode, Context.MeshNodes);
	FFbxSceneHierarchyImporter::CollectSceneNodes(Scene, OutResult.Mesh.SceneNodes);
	FFbxMetadataImporter::CollectSceneNodeMetadata(Scene, OutResult.Mesh.NodeMetadata);

	FFbxMaterialImporter::CollectMaterials(Scene, Context);
	FFbxMaterialImporter::BuildStaticMaterials(Context, OutResult.Materials);

	std::map<int32, TArray<FbxNode*>> RenderNodesByLOD;
	const FImportOptions EffectiveOptions = Options ? *Options : FImportOptions::Default();
	for (FbxNode* MeshNode : Context.MeshNodes)
	{
		if (!MeshNode)
		{
			continue;
		}

		if (FFbxSceneQuery::IsCollisionProxyNode(MeshNode))
		{
			FImportedCollisionShape Shape;
			if (FFbxCollisionImporter::ImportCollisionShape(MeshNode, GetStaticCollisionLocalMatrix(MeshNode), -1, FString(), Shape))
			{
				OutResult.Mesh.CollisionShapes.push_back(std::move(Shape));
			}
			continue;
		}

		FbxMesh* Mesh = MeshNode->GetMesh();
		const bool bHasSkin = Mesh && FFbxSceneQuery::MeshHasSkin(Mesh);
		if (bHasSkin && EffectiveOptions.StaticFbxSkinnedMeshPolicy == EStaticFbxSkinnedMeshPolicy::Skip)
		{
			continue;
		}

		RenderNodesByLOD[FFbxSceneQuery::GetMeshLODIndex(MeshNode)].push_back(MeshNode);
	}

	bool bNeedsNoneSlot = OutResult.Materials.empty();
	for (const auto& Pair : RenderNodesByLOD)
	{
		FStaticMeshLOD LOD;
		if (!ImportLODFromNodes(Pair.second, Pair.first, Context, EffectiveOptions, OutResult.Materials, LOD, bNeedsNoneSlot))
		{
			continue;
		}

		if (OutResult.Mesh.LODModels.empty())
		{
			OutResult.Mesh.Vertices = LOD.Vertices;
			OutResult.Mesh.Indices = LOD.Indices;
			OutResult.Mesh.Sections = LOD.Sections;
		}

		OutResult.Mesh.LODModels.push_back(std::move(LOD));
	}

	AddDefaultNoneMaterialIfNeeded(OutResult.Materials, OutResult.Mesh, bNeedsNoneSlot);
	OutResult.Mesh.PathFileName = SourcePath;
	OutResult.Mesh.CacheBounds();
	OutResult.SourceMaterials = Context.Materials;

	const bool bImportedAnyGeometry = !OutResult.Mesh.Vertices.empty() && !OutResult.Mesh.Indices.empty();
	if (!bImportedAnyGeometry && OutMessage)
	{
		*OutMessage = "FBX static mesh import failed: no geometry imported.";
	}
	return bImportedAnyGeometry;
}
