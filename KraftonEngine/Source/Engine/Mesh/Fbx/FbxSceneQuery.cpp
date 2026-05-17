#include "Mesh/Fbx/FbxSceneQuery.h"

#include <algorithm>
#include <cctype>
#include <cstring>

namespace
{
	static bool StartsWith(const FString& Value, const char* Prefix)
	{
		return Prefix && Value.rfind(Prefix, 0) == 0;
	}

	static FString ToUpperAscii(FString Value)
	{
		for (char& C : Value)
		{
			C = static_cast<char>(std::toupper(static_cast<unsigned char>(C)));
		}
		return Value;
	}

	static FString StripPrefix(const FString& Value, const char* Prefix)
	{
		if (Prefix && Value.rfind(Prefix, 0) == 0)
		{
			return Value.substr(std::strlen(Prefix));
		}
		return FString();
	}

	static bool IsTransformOnlyNode(FbxNode* Node)
	{
		if (!Node)
		{
			return false;
		}

		FbxNodeAttribute* Attribute = Node->GetNodeAttribute();
		if (!Attribute)
		{
			return true;
		}

		const FbxNodeAttribute::EType Type = Attribute->GetAttributeType();
		return Type == FbxNodeAttribute::eNull || Type == FbxNodeAttribute::eMarker;
	}
}

void FFbxSceneQuery::CollectAllNodes(FbxNode* RootNode, TArray<FbxNode*>& OutNodes)
{
	if (!RootNode)
	{
		return;
	}

	OutNodes.push_back(RootNode);
	for (int32 ChildIndex = 0; ChildIndex < RootNode->GetChildCount(); ++ChildIndex)
	{
		CollectAllNodes(RootNode->GetChild(ChildIndex), OutNodes);
	}
}

void FFbxSceneQuery::CollectMeshNodes(FbxNode* RootNode, TArray<FbxNode*>& OutMeshNodes)
{
	if (!RootNode)
	{
		return;
	}

	FbxNodeAttribute* Attribute = RootNode->GetNodeAttribute();
	if (Attribute && Attribute->GetAttributeType() == FbxNodeAttribute::eMesh)
	{
		OutMeshNodes.push_back(RootNode);
	}

	for (int32 ChildIndex = 0; ChildIndex < RootNode->GetChildCount(); ++ChildIndex)
	{
		CollectMeshNodes(RootNode->GetChild(ChildIndex), OutMeshNodes);
	}
}

bool FFbxSceneQuery::IsSkeletonNode(FbxNode* Node)
{
	FbxNodeAttribute* Attr = Node ? Node->GetNodeAttribute() : nullptr;
	return Attr && Attr->GetAttributeType() == FbxNodeAttribute::eSkeleton;
}

bool FFbxSceneQuery::MeshHasSkin(FbxMesh* Mesh)
{
	if (!Mesh)
	{
		return false;
	}

	const int32 SkinCount = Mesh->GetDeformerCount(FbxDeformer::eSkin);
	for (int32 SkinIndex = 0; SkinIndex < SkinCount; ++SkinIndex)
	{
		FbxSkin* Skin = static_cast<FbxSkin*>(Mesh->GetDeformer(SkinIndex, FbxDeformer::eSkin));
		if (Skin && Skin->GetClusterCount() > 0)
		{
			return true;
		}
	}
	return false;
}

bool FFbxSceneQuery::SceneHasSkinDeformer(FbxScene* Scene)
{
	if (!Scene || !Scene->GetRootNode())
	{
		return false;
	}

	TArray<FbxNode*> MeshNodes;
	CollectMeshNodes(Scene->GetRootNode(), MeshNodes);
	for (FbxNode* Node : MeshNodes)
	{
		if (MeshHasSkin(Node ? Node->GetMesh() : nullptr))
		{
			return true;
		}
	}
	return false;
}

bool FFbxSceneQuery::IsValidControlPointIndex(const FbxMesh* Mesh, int32 ControlPointIndex)
{
	return Mesh && ControlPointIndex >= 0 && ControlPointIndex < Mesh->GetControlPointsCount();
}

bool FFbxSceneQuery::ContainsNode(const TArray<FbxNode*>& Nodes, const FbxNode* Node)
{
	return std::find(Nodes.begin(), Nodes.end(), Node) != Nodes.end();
}

void FFbxSceneQuery::AddUniqueNode(TArray<FbxNode*>& Nodes, FbxNode* Node)
{
	if (Node && !ContainsNode(Nodes, Node))
	{
		Nodes.push_back(Node);
	}
}

bool FFbxSceneQuery::IsSceneRootNode(FbxNode* Node)
{
	return Node && Node->GetParent() == nullptr;
}

FString FFbxSceneQuery::ReadStringProperty(FbxNode* Node, const char* PropertyName)
{
	if (!Node || !PropertyName)
	{
		return FString();
	}

	FbxProperty Property = Node->FindProperty(PropertyName);
	if (!Property.IsValid())
	{
		return FString();
	}

	if (Property.GetPropertyDataType().GetType() == eFbxString)
	{
		const FbxString Value = Property.Get<FbxString>();
		return Value.Buffer() ? FString(Value.Buffer()) : FString();
	}

	return FString();
}

int32 FFbxSceneQuery::ParseLODIndexFromName(const FString& Name)
{
	const char* Patterns[] = { "_LOD", "_lod", "LOD", "lod" };
	for (const char* Pattern : Patterns)
	{
		const size_t Pos = Name.rfind(Pattern);
		if (Pos == FString::npos)
		{
			continue;
		}

		size_t DigitPos = Pos + std::strlen(Pattern);
		if (DigitPos < Name.size() && Name[DigitPos] == '_')
		{
			++DigitPos;
		}

		if (DigitPos >= Name.size() || !std::isdigit(static_cast<unsigned char>(Name[DigitPos])))
		{
			continue;
		}

		int32 LODIndex = 0;
		while (DigitPos < Name.size() && std::isdigit(static_cast<unsigned char>(Name[DigitPos])))
		{
			LODIndex = LODIndex * 10 + (Name[DigitPos] - '0');
			++DigitPos;
		}
		return LODIndex;
	}
	return 0;
}

int32 FFbxSceneQuery::GetMeshLODIndex(FbxNode* MeshNode)
{
	if (!MeshNode)
	{
		return 0;
	}

	FbxNode* Parent = MeshNode->GetParent();
	if (Parent && Parent->GetNodeAttribute() && Parent->GetNodeAttribute()->GetAttributeType() == FbxNodeAttribute::eLODGroup)
	{
		for (int32 LODIndex = 0; LODIndex < Parent->GetChildCount(); ++LODIndex)
		{
			if (Parent->GetChild(LODIndex) == MeshNode)
			{
				return LODIndex;
			}
		}
	}

	return ParseLODIndexFromName(MeshNode->GetName());
}

bool FFbxSceneQuery::TryGetFloatProperty(FbxNode* Node, const char* PropertyName, float& OutValue)
{
	if (!Node || !PropertyName)
	{
		return false;
	}

	FbxProperty Property = Node->FindProperty(PropertyName);
	if (!Property.IsValid())
	{
		return false;
	}

	switch (Property.GetPropertyDataType().GetType())
	{
	case eFbxFloat:
		OutValue = static_cast<float>(Property.Get<FbxFloat>());
		return true;
	case eFbxDouble:
		OutValue = static_cast<float>(Property.Get<FbxDouble>());
		return true;
	case eFbxInt:
		OutValue = static_cast<float>(Property.Get<FbxInt>());
		return true;
	case eFbxUInt:
		OutValue = static_cast<float>(Property.Get<FbxUInt>());
		return true;
	case eFbxLongLong:
		OutValue = static_cast<float>(Property.Get<FbxLongLong>());
		return true;
	default:
		return false;
	}
}

bool FFbxSceneQuery::TryGetLODSettings(FbxNode* MeshNode, float& OutScreenSize, float& OutDistanceThreshold)
{
	OutScreenSize        = 1.0f;
	OutDistanceThreshold = 0.0f;
	if (!MeshNode)
	{
		return false;
	}

	bool  bFound = false;
	float Value  = 0.0f;
	if (TryGetFloatProperty(MeshNode, "ScreenSize", Value) || TryGetFloatProperty(MeshNode, "LODScreenSize", Value) || TryGetFloatProperty(
		MeshNode,
		"LodScreenSize",
		Value
	))
	{
		OutScreenSize = Value;
		bFound        = true;
	}

	if (TryGetFloatProperty(MeshNode, "DistanceThreshold", Value) || TryGetFloatProperty(MeshNode, "LODDistance", Value) || TryGetFloatProperty(
		MeshNode,
		"LodDistance",
		Value
	) || TryGetFloatProperty(MeshNode, "LODThreshold", Value))
	{
		OutDistanceThreshold = Value;
		bFound               = true;
	}

	FbxNode* Parent = MeshNode->GetParent();
	if (Parent && Parent->GetNodeAttribute() && Parent->GetNodeAttribute()->GetAttributeType() == FbxNodeAttribute::eLODGroup)
	{
		const FString IndexSuffix = std::to_string(GetMeshLODIndex(MeshNode));
		if (TryGetFloatProperty(Parent, ("LOD" + IndexSuffix + "_ScreenSize").c_str(), Value) || TryGetFloatProperty(
			Parent,
			("LOD" + IndexSuffix + "ScreenSize").c_str(),
			Value
		))
		{
			OutScreenSize = Value;
			bFound        = true;
		}
		if (TryGetFloatProperty(Parent, ("LOD" + IndexSuffix + "_Distance").c_str(), Value) || TryGetFloatProperty(
			Parent,
			("LOD" + IndexSuffix + "Distance").c_str(),
			Value
		) || TryGetFloatProperty(Parent, ("LOD" + IndexSuffix + "_Threshold").c_str(), Value))
		{
			OutDistanceThreshold = Value;
			bFound               = true;
		}
	}

	return bFound;
}

bool FFbxSceneQuery::IsCollisionProxyName(const FString& Name)
{
	const FString UpperName = ToUpperAscii(Name);
	return StartsWith(UpperName, "UCX_") || StartsWith(UpperName, "UBX_") || StartsWith(UpperName, "USP_") || StartsWith(UpperName, "UCP_") || StartsWith(
		UpperName,
		"MCDCX_"
	);
}

bool FFbxSceneQuery::IsCollisionProxyNode(FbxNode* Node)
{
	if (!Node)
	{
		return false;
	}

	if (IsCollisionProxyName(Node->GetName()))
	{
		return true;
	}

	return ReadStringProperty(Node, "ImportKind") == "Collision";
}

bool FFbxSceneQuery::IsSocketName(const FString& Name)
{
	return StartsWith(Name, "SOCKET_") || StartsWith(Name, "Socket_") || StartsWith(Name, "socket_");
}

bool FFbxSceneQuery::IsSocketNode(FbxNode* Node)
{
	if (!Node || IsSkeletonNode(Node))
	{
		return false;
	}

	if (FbxNodeAttribute* Attribute = Node->GetNodeAttribute())
	{
		if (Attribute->GetAttributeType() == FbxNodeAttribute::eMesh)
		{
			return false;
		}
	}

	if (!IsTransformOnlyNode(Node))
	{
		return false;
	}

	const FString ImportKind = ReadStringProperty(Node, "ImportKind");
	return ImportKind == "Socket" || IsSocketName(Node->GetName());
}

FString FFbxSceneQuery::GetSocketName(FbxNode* Node)
{
	if (!Node)
	{
		return FString();
	}

	const FString ExplicitName = ReadStringProperty(Node, "SocketName");
	if (!ExplicitName.empty())
	{
		return ExplicitName;
	}

	const FString NodeName = Node->GetName();
	FString       Stripped = StripPrefix(NodeName, "SOCKET_");
	if (!Stripped.empty()) return Stripped;
	Stripped = StripPrefix(NodeName, "Socket_");
	if (!Stripped.empty()) return Stripped;
	Stripped = StripPrefix(NodeName, "socket_");
	if (!Stripped.empty()) return Stripped;
	return NodeName;
}

void FFbxSceneQuery::CollectSocketNodes(FbxNode* Node, TArray<FbxNode*>& OutSocketNodes)
{
	if (!Node)
	{
		return;
	}

	if (IsSocketNode(Node))
	{
		OutSocketNodes.push_back(Node);
	}

	for (int32 ChildIndex = 0; ChildIndex < Node->GetChildCount(); ++ChildIndex)
	{
		CollectSocketNodes(Node->GetChild(ChildIndex), OutSocketNodes);
	}
}

bool FFbxSceneQuery::FindNearestParentBoneIndex(FbxNode* Node, const TMap<FbxNode*, int32>& BoneNodeToIndex, FbxNode*& OutBoneNode, int32& OutBoneIndex)
{
	FbxNode* Current = Node ? Node->GetParent() : nullptr;
	while (Current && !IsSceneRootNode(Current))
	{
		auto BoneIt = BoneNodeToIndex.find(Current);
		if (BoneIt != BoneNodeToIndex.end())
		{
			OutBoneNode  = Current;
			OutBoneIndex = BoneIt->second;
			return true;
		}
		Current = Current->GetParent();
	}

	OutBoneNode  = nullptr;
	OutBoneIndex = -1;
	return false;
}
