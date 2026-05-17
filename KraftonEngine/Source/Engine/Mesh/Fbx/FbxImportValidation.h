#pragma once

#include "Mesh/SkeletalMeshAsset.h"

class FFbxImportValidation
{
public:
	static float ValidateBindPoseSkinningError(const FSkeletalMesh& Mesh);
};
