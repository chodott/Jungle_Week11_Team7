#pragma once

#include "Core/CoreTypes.h"
#include "Math/Matrix.h"
#include "Mesh/MeshCollisionAsset.h"

#include <fbxsdk.h>

class FFbxCollisionImporter
{
public:
    static bool ImportCollisionShape(
        FbxNode*                 MeshNode,
        const FMatrix&           LocalMatrix,
        int32                    ParentBoneIndex,
        const FString&           ParentBoneName,
        FImportedCollisionShape& OutShape
        );

private:
    static EImportedCollisionType DetectCollisionType(const FString& NodeName);
};
