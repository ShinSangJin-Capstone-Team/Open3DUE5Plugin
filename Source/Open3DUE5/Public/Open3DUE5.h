// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Containers/Array.h"
#include "Math/MathFwd.h"
#include "HAL/Platform.h"
#include "Modules/ModuleManager.h"

class OPEN3DUE5_API FOpen3DUE5Module : public IModuleInterface
{
public:

	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

	static void CleanUpRawData(TArray<FVector> InPoints, float VoxelSize, TArray<FVector>& OutPoints);

	static void VoxelizedArrFromPoints(TArray<FVector3f> InPoints, double VoxelSize, TArray<FIntVector>& OutVoxelizedArr, FIntVector& CalcedRealSize);

	void CleanUpSensorHPS();
	void InitSensor();
	void GetSensorOneFrame(TArray<FVector>& Points);

private:
	TArray<void*> DLLHandles = {};

	TArray<FString> DLLPaths =
	{
#if PLATFORM_WINDOWS
		"Binaries/ThirdParty/Open3D/bin/Open3D.dll"//,
		//"Binaries/ThirdParty/HPS3D/bin/HPS3D160_SDK.dll"
#endif
	};

	/** Handle to the dll we will load */
	//void* Open3DHandle;
};
