// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Containers/Array.h"
#include "Math/MathFwd.h"
#include "HAL/Platform.h"
#include "Modules/ModuleManager.h"

namespace sy3
{
	class context;
	class pipeline;
	enum sy3_error;
};

class OPEN3DUE5_API FOpen3DUE5Module : public IModuleInterface
{
public:

	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

	static void VoxelizedArrFromPoints(TArray<FVector3f> InPoints, double VoxelSize, TArray<FIntVector>& OutVoxelizedArr, FIntVector& CalcedRealSize);

	void InitSensor();
	void GetSensorOneFrame(TArray<FVector>& Points);

	void InitSensorCS20();
	void GetSensorOneFrameCS20(TArray<FVector>& Points);

private:
	TArray<void*> DLLHandles = {};

	TArray<FString> DLLPaths =
	{
		"Binaries/ThirdParty/Open3D/bin/Open3D.dll",
		"Binaries/ThirdParty/CS20/bin/csreconstruction.dll",
		"Binaries/ThirdParty/CS20/bin/libsynexens3.dll",
		"Binaries/ThirdParty/CS20/bin/opencv_calib3d440.dll",
		"Binaries/ThirdParty/CS20/bin/opencv_core440.dll",
		"Binaries/ThirdParty/CS20/bin/opencv_features2d440.dll",
		"Binaries/ThirdParty/CS20/bin/opencv_flann440.dll",
		"Binaries/ThirdParty/CS20/bin/opencv_highgui440.dll",
		"Binaries/ThirdParty/CS20/bin/opencv_imgcodecs440.dll",
		"Binaries/ThirdParty/CS20/bin/opencv_imgproc440.dll",
		"Binaries/ThirdParty/CS20/bin/opencv_videoio440.dll",
		"Binaries/ThirdParty/CS20/bin/pthreadVC2.dll",
		"Binaries/ThirdParty/CS20/bin/SonixCamera.dll",
		"Binaries/ThirdParty/HPS3D/bin/HPS3D160_SDK.dll"
	};

	/** Handle to the dll we will load */
	//void* Open3DHandle;

	sy3::context* ctx = nullptr;
	sy3::pipeline* pline = nullptr;
	sy3::sy3_error e;

	int handle;
};