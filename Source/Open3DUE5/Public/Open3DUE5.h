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

	static void VoxelizedArrFromPoints(TArray<FVector3f> InPoints, double VoxelSize, TArray<FIntVector>& OutVoxelizedArr, FIntVector& CalcedRealSize);

	void InitSensor();
	void GetSensorOneFrame(TArray<FVector>& Points);

private:
	TArray<void*> DLLHandles = {};

	TArray<FString> DLLPaths =
	{
		"Binaries/ThirdParty/Open3D/bin/Open3D.dll",
		"Binaries/ThirdParty/CS20/bin/opencv/opencv_core440.dll",
		"Binaries/ThirdParty/CS20/bin/opencv/opencv_imgproc440.dll",
		"Binaries/ThirdParty/CS20/bin/opencv/opencv_imgcodecs440.dll",
		"Binaries/ThirdParty/CS20/bin/opencv/opencv_videoio440.dll",
		"Binaries/ThirdParty/CS20/bin/opencv/opencv_highgui440.dll",
		"Binaries/ThirdParty/CS20/bin/csreconstruction2.0.dll",
		"Binaries/ThirdParty/CS20/bin/pthreadVC2.dll",
		"Binaries/ThirdParty/CS20/bin/SonixCamera.dll",
		"Binaries/ThirdParty/CS20/bin/SynexensSDK.dll"
	};

	/** Handle to the dll we will load */
	//void* Open3DHandle;
};
