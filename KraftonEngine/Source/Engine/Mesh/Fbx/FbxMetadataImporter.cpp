#include "Mesh/Fbx/FbxMetadataImporter.h"

#include <algorithm>
#include <cctype>
#include <utility>
#include <string>

namespace
{
    static bool StartsWith(const FString& Value, const char* Prefix)
    {
        return Prefix && Value.rfind(Prefix, 0) == 0;
    }

    static FString GetNodePath(FbxNode* Node)
    {
        if (!Node)
        {
            return FString();
        }

        TArray<FString> Parts;
        FbxNode*        Current = Node;
        while (Current)
        {
            Parts.push_back(Current->GetName() ? FString(Current->GetName()) : FString());
            Current = Current->GetParent();
        }

        FString Path;
        for (auto It = Parts.rbegin(); It != Parts.rend(); ++It)
        {
            if (It->empty())
            {
                continue;
            }

            if (!Path.empty())
            {
                Path += "/";
            }
            Path += *It;
        }
        return Path;
    }

    static bool IsInternalImporterProperty(const FString& Name)
    {
        return Name == "ImportKind" || Name == "SocketName";
    }

    static bool IsMetadataProperty(const FbxProperty& Property)
    {
        if (!Property.IsValid())
        {
            return false;
        }

        const char*   RawName = Property.GetNameAsCStr();
        const FString Name    = RawName ? FString(RawName) : FString();
        if (Name.empty() || IsInternalImporterProperty(Name))
        {
            return false;
        }

        if (Property.GetFlag(FbxPropertyFlags::eUserDefined))
        {
            return true;
        }

        return StartsWith(Name, "Meta_") || StartsWith(Name, "Gameplay_") || StartsWith(Name, "FbxMeta_");
    }

    static FString ToStringValue(const FbxProperty& Property)
    {
        const FbxDataType DataType = Property.GetPropertyDataType();
        const EFbxType    Type     = DataType.GetType();

        switch (Type)
        {
        case eFbxString:
        {
            const FbxString Value = Property.Get<FbxString>();
            return Value.Buffer() ? FString(Value.Buffer()) : FString();
        }
        case eFbxBool:
            return Property.Get<FbxBool>() ? "true" : "false";
        case eFbxInt:
            return std::to_string(static_cast<int32>(Property.Get<FbxInt>()));
        case eFbxUInt:
            return std::to_string(static_cast<uint32>(Property.Get<FbxUInt>()));
        case eFbxLongLong:
            return std::to_string(static_cast<int64>(Property.Get<FbxLongLong>()));
        case eFbxULongLong:
            return std::to_string(static_cast<uint64>(Property.Get<FbxULongLong>()));
        case eFbxFloat:
            return std::to_string(static_cast<float>(Property.Get<FbxFloat>()));
        case eFbxDouble:
            return std::to_string(static_cast<double>(Property.Get<FbxDouble>()));
        default:
            return FString();
        }
    }

    static bool ConvertPropertyToMetadataValue(FbxProperty Property, FFbxImportedMetadataValue& OutValue)
    {
        if (!IsMetadataProperty(Property))
        {
            return false;
        }

        const char* RawName = Property.GetNameAsCStr();
        OutValue.Key        = RawName ? FString(RawName) : FString();
        if (OutValue.Key.empty())
        {
            return false;
        }

        const FbxDataType DataType = Property.GetPropertyDataType();
        const EFbxType    Type     = DataType.GetType();

        switch (Type)
        {
        case eFbxString:
        {
            OutValue.Type         = EFbxImportedMetadataValueType::String;
            const FbxString Value = Property.Get<FbxString>();
            OutValue.StringValue  = Value.Buffer() ? FString(Value.Buffer()) : FString();
            break;
        }
        case eFbxBool:
            OutValue.Type = EFbxImportedMetadataValueType::Bool;
            OutValue.BoolValue   = Property.Get<FbxBool>();
            OutValue.StringValue = OutValue.BoolValue ? "true" : "false";
            break;
        case eFbxInt:
            OutValue.Type = EFbxImportedMetadataValueType::Int;
            OutValue.IntValue    = static_cast<int32>(Property.Get<FbxInt>());
            OutValue.StringValue = std::to_string(OutValue.IntValue);
            break;
        case eFbxUInt:
            OutValue.Type = EFbxImportedMetadataValueType::Int;
            OutValue.IntValue    = static_cast<int32>(Property.Get<FbxUInt>());
            OutValue.StringValue = std::to_string(OutValue.IntValue);
            break;
        case eFbxLongLong:
            OutValue.Type = EFbxImportedMetadataValueType::Int;
            OutValue.IntValue    = static_cast<int32>(Property.Get<FbxLongLong>());
            OutValue.StringValue = std::to_string(OutValue.IntValue);
            break;
        case eFbxFloat:
            OutValue.Type = EFbxImportedMetadataValueType::Float;
            OutValue.FloatValue  = static_cast<float>(Property.Get<FbxFloat>());
            OutValue.StringValue = std::to_string(OutValue.FloatValue);
            break;
        case eFbxDouble:
            OutValue.Type = EFbxImportedMetadataValueType::Float;
            OutValue.FloatValue  = static_cast<float>(Property.Get<FbxDouble>());
            OutValue.StringValue = std::to_string(OutValue.FloatValue);
            break;
        case eFbxDouble3:
        {
            OutValue.Type          = EFbxImportedMetadataValueType::Vector3;
            const FbxDouble3 Value = Property.Get<FbxDouble3>();
            OutValue.VectorValue   = FVector(static_cast<float>(Value[0]), static_cast<float>(Value[1]), static_cast<float>(Value[2]));
            OutValue.StringValue   = std::to_string(OutValue.VectorValue.X) + "," + std::to_string(OutValue.VectorValue.Y) + "," + std::to_string(
                OutValue.VectorValue.Z
            );
            break;
        }
        case eFbxDouble4:
        {
            OutValue.Type          = EFbxImportedMetadataValueType::Color;
            const FbxDouble4 Value = Property.Get<FbxDouble4>();
            OutValue.ColorValue    = FVector4(
                static_cast<float>(Value[0]),
                static_cast<float>(Value[1]),
                static_cast<float>(Value[2]),
                static_cast<float>(Value[3])
            );
            OutValue.StringValue = std::to_string(OutValue.ColorValue.X) + "," + std::to_string(OutValue.ColorValue.Y) + "," + std::to_string(
                OutValue.ColorValue.Z
            ) + "," + std::to_string(OutValue.ColorValue.W);
            break;
        }
        default:
            OutValue.Type = EFbxImportedMetadataValueType::String;
            OutValue.StringValue = ToStringValue(Property);
            if (OutValue.StringValue.empty())
            {
                return false;
            }
            break;
        }

        return true;
    }

    static void CollectNodeMetadataRecursive(FbxNode* Node, TArray<FFbxImportedNodeMetadata>& OutMetadata)
    {
        if (!Node)
        {
            return;
        }

        FFbxMetadataImporter::CollectObjectMetadata(Node, Node->GetName() ? FString(Node->GetName()) : FString(), GetNodePath(Node), OutMetadata);

        for (int32 ChildIndex = 0; ChildIndex < Node->GetChildCount(); ++ChildIndex)
        {
            CollectNodeMetadataRecursive(Node->GetChild(ChildIndex), OutMetadata);
        }
    }
}

void FFbxMetadataImporter::CollectSceneNodeMetadata(FbxScene* Scene, TArray<FFbxImportedNodeMetadata>& OutMetadata)
{
    OutMetadata.clear();
    if (!Scene || !Scene->GetRootNode())
    {
        return;
    }

    CollectNodeMetadataRecursive(Scene->GetRootNode(), OutMetadata);
}

void FFbxMetadataImporter::CollectNodeMetadata(FbxNode* Node, TArray<FFbxImportedNodeMetadata>& OutMetadata)
{
    CollectNodeMetadataRecursive(Node, OutMetadata);
}

void FFbxMetadataImporter::CollectObjectMetadata(
    FbxObject*                        Object,
    const FString&                    SourceName,
    const FString&                    NodePath,
    TArray<FFbxImportedNodeMetadata>& OutMetadata
    )
{
    TArray<FFbxImportedMetadataValue> Values = ExtractMetadataValues(Object);
    if (Values.empty())
    {
        return;
    }

    FFbxImportedNodeMetadata Metadata;
    Metadata.SourceNodeName = SourceName;
    Metadata.NodePath       = NodePath;
    Metadata.Values         = std::move(Values);
    OutMetadata.push_back(std::move(Metadata));
}

TArray<FFbxImportedMetadataValue> FFbxMetadataImporter::ExtractMetadataValues(FbxObject* Object)
{
    TArray<FFbxImportedMetadataValue> Values;
    if (!Object)
    {
        return Values;
    }

    for (FbxProperty Property = Object->GetFirstProperty(); Property.IsValid(); Property = Object->GetNextProperty(Property))
    {
        FFbxImportedMetadataValue Value;
        if (ConvertPropertyToMetadataValue(Property, Value))
        {
            Values.push_back(Value);
        }
    }

    return Values;
}
