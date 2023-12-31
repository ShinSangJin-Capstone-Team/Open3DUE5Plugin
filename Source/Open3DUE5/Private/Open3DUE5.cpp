// Copyright Epic Games, Inc. All Rights Reserved.

#include "Open3DUE5.h"
#include "Core.h"
#include "HAL/Platform.h"
#include "Containers/Array.h"
#include "Math/MathFwd.h"

/**/
#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#include "Windows/WindowsHWrapper.h"
#endif
//*/
#include <iostream>
#include <vector>
#include <memory>
#include <tchar.h>

/**/
#if PLATFORM_WINDOWS
#include "Windows/HideWindowsPlatformTypes.h"
#endif
//*/

THIRD_PARTY_INCLUDES_START
#pragma push_macro("check")   // store 'check' macro current definition
#undef check  // undef to avoid conflicts
#include "open3d/Open3D.h"
#include "libsynexens3/libsynexens3.h"
#include "opencv.hpp"
#include "imgproc/imgproc.hpp"
#include "HPS3DUser_IF.h"
#include "HPS3DUser_IF.c"
#pragma pop_macro("check")  // restore definition
THIRD_PARTY_INCLUDES_END

#include "Modules/ModuleManager.h"
#include "Interfaces/IPluginManager.h"

#define LOCTEXT_NAMESPACE "FOpen3DUE5Module"

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif
#define no_init_all_deprecated


#define RAW_WIDTH 1280
#define RAW_HEIGHT 960

#if 1
#define DEPTH_WIDTH 640
#define DEPTH_HEIGHT 480
#else
#define DEPTH_WIDTH 320
#define DEPTH_HEIGHT 240
#endif

#define RGB_WIDTH 1920
#define RGB_HEIGHT 1080

#define RGBD_WINDOW_NAME "RGBD"
#define RGB_WINDOW_NAME "RGB"
#define TOF_WINDOW_NAME "TOF"

typedef std::basic_string<TCHAR> tstring;
//HPS3D start

TCHAR* StringToTCHAR(std::string& s) 
{ 
	size_t cn;
	//tstring tstr;
	const char* all = s.c_str();
	int len = 1 + strlen(all);
	wchar_t* t = new wchar_t[len]; 
	if (NULL == t) throw std::bad_alloc();
	//mbstowcs(t, all, len);
	mbstowcs_s(&cn, t, len, all, len - 1);
	return (TCHAR*)t;
}


#pragma region CS20
namespace CS20Things
{
	sy3::context* ctx;
	sy3::pipeline* pline;
	sy3::sy3_error e;
}

void print_device_info(sy3::device* dev)
{
	sy3::sy3_error e;
	printf("\nUsing device 0, an %s\n", sy3::sy3_get_device_info(dev, sy3::SY3_CAMERA_INFO_NAME, e));
	printf("    Serial number: %s\n", sy3::sy3_get_device_info(dev, sy3::SY3_CAMERA_INFO_SERIAL_NUMBER, e));
	printf("    Firmware version: %s\n\n", sy3::sy3_get_device_info(dev, sy3::SY3_CAMERA_INFO_FIRMWARE_VERSION, e));
	UE_LOG(LogTemp, Warning, TEXT("\nUsing device 0, an %s\n"), sy3::sy3_get_device_info(dev, sy3::SY3_CAMERA_INFO_NAME, e));
	UE_LOG(LogTemp, Warning, TEXT("    Serial number: %s\n"), sy3::sy3_get_device_info(dev, sy3::SY3_CAMERA_INFO_SERIAL_NUMBER, e));
	UE_LOG(LogTemp, Warning, TEXT("    Firmware version: %s\n\n"), sy3::sy3_get_device_info(dev, sy3::SY3_CAMERA_INFO_FIRMWARE_VERSION, e));
}

void print_support_format(sy3::device* dev, sy3::sy3_error& e)
{

	/**
	std::vector<sy3::sy3_stream> support_stream = dev->get_support_stream(e);
	for (int i = 0; i < support_stream.size(); i++)
	{
		UE_LOG(LogTemp, Warning, TEXT("support stream : % s \n"), sy3_stream_to_string(support_stream[i]));
		//printf("support stream:%s \n", sy3_stream_to_string(support_stream[i]));
		std::vector<sy3::sy3_format> support_format = dev->get_support_format(support_stream[i], e);
		for (int j = 0; j < support_format.size(); j++)
		{
			UE_LOG(LogTemp, Warning, TEXT("\t\t support format:%d x %d \n"), support_format[j].width, support_format[j].height);
			printf("\t\t support format:%d x %d \n", support_format[j].width, support_format[j].height);
		}
	}
	//*/
}

void FOpen3DUE5Module::InitSensorCS20()
{
	UE_LOG(LogTemp, Warning, TEXT("version: % s"), *FString(sy3::sy3_get_version(CS20Things::e)));

	printf("version:%s \n", sy3::sy3_get_version(CS20Things::e));
	CS20Things::ctx = sy3::sy3_create_context(CS20Things::e);
	sy3::device* dev = CS20Things::ctx->query_device(CS20Things::e);

	if (CS20Things::e != sy3::sy3_error::SUCCESS) {
		UE_LOG(LogTemp, Warning, TEXT("error: %d %s \n"), CS20Things::e, *FString(sy3::sy3_error_to_string(CS20Things::e)));
		printf("error:%d  %s \n", CS20Things::e, sy3::sy3_error_to_string(CS20Things::e));
		return;
	}

	print_support_format(dev, CS20Things::e);
	print_device_info(dev);

	CS20Things::pline = sy3::sy3_create_pipeline(CS20Things::ctx, CS20Things::e);

	sy3::config* cfg = sy3_create_config(CS20Things::ctx, CS20Things::e);
	cfg->enable_stream(sy3::sy3_stream::SY3_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, CS20Things::e);
	cfg->enable_stream(sy3::sy3_stream::SY3_STREAM_RGB, RGB_WIDTH, RGB_HEIGHT, CS20Things::e);

	CS20Things::pline->start(cfg, CS20Things::e);
}

void FOpen3DUE5Module::GetSensorOneFrameCS20(TArray<FVector>& Points)
{
	if (!CS20Things::pline)
	{
		if (!CS20Things::ctx)
		{
			CS20Things::ctx = sy3::sy3_create_context(CS20Things::e);
			sy3::device* dev = CS20Things::ctx->query_device(CS20Things::e);

			if (CS20Things::e != sy3::sy3_error::SUCCESS) {
				UE_LOG(LogTemp, Warning, TEXT("error: %d %s \n"), CS20Things::e, *FString(sy3::sy3_error_to_string(CS20Things::e)));
				printf("error:%d  %s \n", CS20Things::e, sy3::sy3_error_to_string(CS20Things::e));
				Points.Empty();
				Points.Add(FVector(-1.0, -1.0, -1.0));
				return;
			}

			print_support_format(dev, CS20Things::e);
			print_device_info(dev);
		}

		CS20Things::pline = sy3::sy3_create_pipeline(CS20Things::ctx, CS20Things::e);

		sy3::config* cfg = sy3_create_config(CS20Things::ctx, CS20Things::e);
		cfg->enable_stream(sy3::sy3_stream::SY3_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, CS20Things::e);
		cfg->enable_stream(sy3::sy3_stream::SY3_STREAM_RGB, RGB_WIDTH, RGB_HEIGHT, CS20Things::e);

		CS20Things::pline->start(cfg, CS20Things::e);
	}
	Points.Empty();

	sy3::frameset* frameset = CS20Things::pline->wait_for_frames(SY3_DEFAULT_TIMEOUT, CS20Things::e);
	sy3::depth_frame* depth_frame = frameset->get_depth_frame();
	//sy3::rgb_frame* rgb_frame = frameset->get_rgb_frame();

	if (depth_frame)// && rgb_frame)
	{
		/*sy3::sy3_intrinsics intrinsics_tof = depth_frame->get_profile()->get_intrinsics();
		printf("depth intrinsics: %d x %d  %f %f\n", intrinsics_tof.width, intrinsics_tof.height, intrinsics_tof.fx, intrinsics_tof.fy);
		sy3::sy3_intrinsics intrinsics_rgb = rgb_frame->get_profile()->get_intrinsics();
		printf("rgb intrinsics: %d x %d  %f %f\n", intrinsics_rgb.width, intrinsics_rgb.height, intrinsics_rgb.fx, intrinsics_rgb.fy);

		show_depth_frame(depth_frame, "depth");*/

		if (depth_frame->get_width() == 640) {

			int align_width = 640;
			int align_height = 480;
			sy3::frameset* set = frameset;//pline->get_process_engin(e)->align_to_rgb(depth_frame, rgb_frame, align_width, align_height, e);
			//show_depth_frame(set->get_depth_frame(), "algin_depth");

			int height = set->get_depth_frame()->get_height();
			int width = set->get_depth_frame()->get_width();
			UE_LOG(LogTemp, Warning, TEXT("Height: %d, Width: %d"), height, width);
			sy3::sy3_error e1;
			sy3::points* points = CS20Things::pline->get_process_engin(e1)->comptute_points(depth_frame, true, e1);
			float* data = points->get_points();
			std::cout << points->get_length() << std::endl;
			/**/
			for (auto i = 0; i < height; i++)
			{
				for (auto j = 0; j < width; j++)
				{
					auto depth = data[(3 * width) * i + 3 * j + 2];
					if (200 < depth && depth < 3000)
					{
						Points.Add(FVector(
							data[(3 * width) * i + 3 * j + 0],
							data[(3 * width) * i + 3 * j + 1],
							data[(3 * width) * i + 3 * j + 2]
						));
					}
					//printf("%4d ", (int)data[i * (3 * width) + j * (3) + 2]);
				}
			}
			delete points;
		}
	}

	delete frameset;
}

#pragma endregion

#pragma region HPS
namespace HPSThings
{
	int handle = -1;
	HPS3D_StatusTypeDef ret = HPS3D_RET_OK;
	HPS3D_DeviceSettings_t settings;

	static HPS3D_MeasureData_t g_measureData;
}

static void EventCallBackFunc(int handle, int eventType, uint8_t* data, int dataLen, void* userPara)
{
	switch ((HPS3D_EventType_t)eventType)
	{
		//Measurement data notification event
	case HPS3D_SIMPLE_ROI_EVEN:
	case HPS3D_FULL_ROI_EVEN:
	case HPS3D_FULL_DEPTH_EVEN:
	case HPS3D_SIMPLE_DEPTH_EVEN:
		HPS3D_ConvertToMeasureData(data, &HPSThings::g_measureData, (HPS3D_EventType_t)eventType);
		//PrintResultData((HPS3D_EventType_t)eventType, g_measureData);
		break;
	case HPS3D_SYS_EXCEPTION_EVEN: /*System anomaly notification event*/
		printf("SYS ERR :%s\n", data);
		break;
	case HPS3D_DISCONNECT_EVEN: /*Connection abnormal disconnection notification event*/
		printf("Device disconnected!\n");
		HPS3D_CloseDevice(handle);
		HPS3D_MeasureDataFree(&HPSThings::g_measureData);
		break;
	case HPS3D_NULL_EVEN:  //Empty event notification
	default:
		break;
	}
}

static std::string toString(const Eigen::MatrixXd& mat) {
	std::stringstream ss;
	ss << mat;
	return ss.str();
}

FVector pinPointToFittingPlane(int x, int y, float tolerance, HPS3D_PerPointCloudData_t** data)
{
	float originalDepth = data[y][x].z;
	auto originalPoint = Eigen::Vector3d(data[y][x].x, data[y][x].y, data[y][x].z);

	std::vector<Eigen::Vector3d> PointsToFormPlane = {};
	std::vector<Eigen::Vector3d> PointsToFormPlaneCentroid = {};

	Eigen::Vector3d PointMean = { 0, 0, 0 };
	for (auto i = -2; i < 3; ++i)
	{
		for (auto j = -2; j < 3; ++j)
		{
			if (0 <= (i + x) && (i + x) < HPSThings::settings.max_resolution_X && 0 <= (j + y) && (j + y) < HPSThings::settings.max_resolution_Y)
			{
				auto Point = Eigen::Vector3d(data[j + y][i + x].x, data[j + y][i + x].y, data[j + y][i + x].z);
				if (FMath::Abs((Point - originalPoint).norm()) < tolerance)
				{
					PointsToFormPlane.push_back(Point);
					PointMean += Point;
				}
			}
		}
	}

	PointMean = PointMean / PointsToFormPlane.size();

	for (auto& APoint : PointsToFormPlane)
	{
		Eigen::Vector3d NormedPoint = APoint - PointMean;
		PointsToFormPlaneCentroid.push_back(NormedPoint);
	}
	
	auto Matrix = Eigen::Matrix3Xd(3, PointsToFormPlaneCentroid.size());

	for (auto i = 0; i < PointsToFormPlaneCentroid.size(); ++i)
	{
		Matrix.col(i) = PointsToFormPlaneCentroid[i];
	}

	//UE_LOG(LogTemp, Warning, TEXT("%s"), *FString(toString(Matrix).c_str()));

	Eigen::JacobiSVD<Eigen::Matrix3Xd> SVD;

	SVD.compute(Matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);

	Eigen::Vector3d PlaneNormal = SVD.matrixU().rightCols<1>();
	PlaneNormal.normalize();

	Eigen::Vector3d v = originalPoint - PointMean;
	auto dist = v.dot(PlaneNormal);
	Eigen::Vector3d newPoint = originalPoint - dist * PlaneNormal;
	return FVector(newPoint.x(), newPoint.y(), newPoint.z());
}

void pinAllDataToFittingPlanes(HPS3D_PerPointCloudData_t** dataBuffer)
{
	auto** tempBuffer = new HPS3D_PerPointCloudData_t * [HPSThings::settings.max_resolution_Y] {};
	for (auto i = 0; i < HPSThings::settings.max_resolution_Y; ++i)
	{
		tempBuffer[i] = new HPS3D_PerPointCloudData_t[HPSThings::settings.max_resolution_X]{};
	}

	for (auto i = 0; i < HPSThings::settings.max_resolution_Y; i++)
	{
		for (auto j = 0; j < HPSThings::settings.max_resolution_X; j++)
		{
			auto& originalPoint = dataBuffer[i][j];
			auto originalDepth = originalPoint.z;
			auto newLoc = pinPointToFittingPlane(j, i, 150, dataBuffer);
			tempBuffer[i][j].z = newLoc.Z;
			tempBuffer[i][j].x = newLoc.X;
			tempBuffer[i][j].y = newLoc.Y;
		}
	}

	for (auto i = 0; i < HPSThings::settings.max_resolution_Y; i++)
	{
		for (auto j = 0; j < HPSThings::settings.max_resolution_X; j++)
		{
			dataBuffer[i][j] = tempBuffer[i][j];
		}
	}

	for (auto i = 0; i < HPSThings::settings.max_resolution_Y; ++i)
	{
		delete[] tempBuffer[i];
	}

	delete[] tempBuffer;
}

void FOpen3DUE5Module::CleanUpSensorHPS()
{
	HPS3D_UnregisterEventCallback();
	if (HPSThings::handle >= 0)
	{
		HPS3D_StopCapture(HPSThings::handle);
		HPS3D_CloseDevice(HPSThings::handle);
	}
	HPS3D_MeasureDataFree(&HPSThings::g_measureData);
}

void FOpen3DUE5Module::InitSensor()
{
	UE_LOG(LogTemp, Warning, TEXT("HPS3D160 C/C++ Demo (Visual Statudio 2017)\n\n"));
	UE_LOG(LogTemp, Warning, TEXT("SDK Ver:%s\n"), HPS3D_GetSDKVersion());

	//Memroy allocation
	HPS3D_MeasureDataInit(&HPSThings::g_measureData);

	HPSThings::handle = -1;

	//HPSThings::ret = HPS3D_USBConnectDevice((char*)_T("\\\\.\\COM23"), &HPSThings::handle);

	
	for (int i = 1; i < 10; ++i)
	{
		//if you have problem with connection this may be the problem
		auto portNameString = std::string("COM") + std::to_string(i);

		HPSThings::ret = HPS3D_USBConnectDevice((char*)StringToTCHAR(portNameString), &HPSThings::handle);

		if (HPSThings::ret != HPS3D_RET_OK) continue;
		else break;
	}
	if (HPSThings::ret != HPS3D_RET_OK)
	{
		for (int i = 10; i < 127; ++i)
		{
			//if you have problem with connection this may be the problem
			auto portNameString = std::string("\\\\.\\COM") + std::to_string(i);
			HPSThings::ret = HPS3D_USBConnectDevice((char*)StringToTCHAR(portNameString), &HPSThings::handle);

			if (HPSThings::ret != HPS3D_RET_OK) continue;
			else break;
		}
	}

	if (HPSThings::ret != HPS3D_RET_OK)
	{
		//printf("Device connection failed here,Err:%d\n", ret);
		UE_LOG(LogTemp, Warning, TEXT("Device connection failed here,Err:%d\n"), HPSThings::ret);
		return;
	}
	//Register event callback function for receiving continuous data packets and handling exceptions
	HPS3D_RegisterEventCallback(EventCallBackFunc, NULL);

	printf("Device version:%s\n", HPS3D_GetDeviceVersion(HPSThings::handle));
	printf("Device serial number:%s\n\n", HPS3D_GetSerialNumber(HPSThings::handle));
	UE_LOG(LogTemp, Warning, TEXT("Device handle: %d\n"), HPSThings::handle);

	HPSThings::ret = HPS3D_ExportSettings(HPSThings::handle, &HPSThings::settings);
	if (HPSThings::ret != HPS3D_RET_OK)
	{
		//printf("Failed to export device parameters,Err:%d\n", ret);
		return;
	}
}

void FOpen3DUE5Module::GetSensorOneFrame(TArray<FVector>& Points)
{
	//printf("HPS3D160 C/C++ Demo (Visual Statudio 2019)\n\n");
	//printf("SDK Ver:%s\n", HPS3D_GetSDKVersion());
	bool isContinuous = false;

	Points.Empty();

	//printf("Select measurement mode: Single measurement(1) Continuous measurement(2) Exit(Other)\n");
	char se = '1';
	//getchar(); //Filter Carriage Return
	if (se == '1')
	{
		//Single measurement mode
		HPS3D_EventType_t type = HPS3D_NULL_EVEN;
		//printf("%d\n", handle);
		HPSThings::ret = HPS3D_SingleCapture(HPSThings::handle, &type, &HPSThings::g_measureData);
		if (HPSThings::ret != HPS3D_RET_OK)
		{
			//printf("Single measurement failed,Err:%d\n", ret);
			return;
		}
		//PrintResultData(type, g_measureData);

		auto** dataBuffer = new HPS3D_PerPointCloudData_t * [HPSThings::settings.max_resolution_Y] {};
		for (auto i = 0; i < HPSThings::settings.max_resolution_Y; ++i)
		{
			dataBuffer[i] = new HPS3D_PerPointCloudData_t[HPSThings::settings.max_resolution_X]{};
		}

		for (auto i = 0; i < HPSThings::settings.max_resolution_Y; i++)
		{
			for (auto j = 0; j < HPSThings::settings.max_resolution_X; j++)
			{
				dataBuffer[i][j] = HPSThings::g_measureData.full_depth_data.point_cloud_data.point_data[HPSThings::settings.max_resolution_X * i + j];
			}
		}

		// flatten depths
		pinAllDataToFittingPlanes(dataBuffer);
		pinAllDataToFittingPlanes(dataBuffer);
		pinAllDataToFittingPlanes(dataBuffer);
		pinAllDataToFittingPlanes(dataBuffer);
		pinAllDataToFittingPlanes(dataBuffer);

		for (auto i = 0; i < HPSThings::settings.max_resolution_Y; i++)
		{
			for (auto j = 0; j < HPSThings::settings.max_resolution_X; j++)
			{
				auto distance = dataBuffer[i][j].z;
				if (200 < distance && distance < 2000)
				{
					Points.Add(FVector(
						dataBuffer[i][j].x,
						dataBuffer[i][j].y,
						dataBuffer[i][j].z
					));
				}
				//printf("%1d", (int)data.full_depth_data.point_cloud_data.point_data[i*160 + j].z/1000);
			}
		}

		for (auto i = 0; i < HPSThings::settings.max_resolution_Y; ++i)
		{
			delete[] dataBuffer[i];
		}

		delete[] dataBuffer;
		//UE_LOG(LogTemp, Warning, TEXT("%2f "), g_measureData.full_depth_data.point_cloud_data.point_data[400].z);
		//UE_LOG(LogTemp, Warning, TEXT("%2f "), g_measureData.full_depth_data.point_cloud_data.point_data[500].z);
	}
	else
	{
		isContinuous = false;
		return;
	}
}
#pragma endregion

void FOpen3DUE5Module::StartupModule()
{
	// This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module

	// Get the base directory of this plugin
	FString BaseDir = IPluginManager::Get().FindPlugin("Open3DUE5")->GetBaseDir();

	// Add on the relative location of the third party dll and load it
	//FString LibraryPath;
#if PLATFORM_WINDOWS
	DLLHandles.Empty();
	for (auto Path : DLLPaths)
	{
		auto LibraryPath = FPaths::Combine(*BaseDir, Path);
		auto DLLHandle = !LibraryPath.IsEmpty() ? FPlatformProcess::GetDllHandle(*LibraryPath) : nullptr;
		DLLHandles.Add(DLLHandle);
	}
	//LibraryPath = FPaths::Combine(*BaseDir, TEXT("Binaries/ThirdParty/Open3D/bin/Open3D.dll"));
#endif // PLATFORM_WINDOWS

	//Open3DHandle = !LibraryPath.IsEmpty() ? FPlatformProcess::GetDllHandle(*LibraryPath) : nullptr;
}

void FOpen3DUE5Module::ShutdownModule()
{
	// This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
	// we call this function before unloading the module.

	// Free the dll handle

	for (auto& Handle : DLLHandles)
	{
		FPlatformProcess::FreeDllHandle(Handle);
		Handle = nullptr;
	}

	DLLHandles.Empty();

	//FPlatformProcess::FreeDllHandle(Open3DHandle);
	//Open3DHandle = nullptr;
}

void FOpen3DUE5Module::CleanUpRawData(TArray<FVector> InPoints, float VoxelSize, TArray<FVector>& OutPoints)
{
	std::vector<Eigen::Vector3d> EigenPoints = {};

	for (auto APoint : InPoints)
	{
		EigenPoints.push_back(Eigen::Vector3d(APoint.X, APoint.Y, APoint.Z));
	}

	auto DownSampled = open3d::geometry::PointCloud(EigenPoints).VoxelDownSample(5.0 * VoxelSize);

	auto StatCleanUp = DownSampled->RemoveStatisticalOutliersJustPtr(8, 0.75);
	auto RadiusCleanUp = StatCleanUp;// ->RemoveRadiusOutliersJustPtr(5, 8.0 * VoxelSize * FMath::Sqrt(3.0));
	
	OutPoints.Empty();
	for (auto APoint : RadiusCleanUp->points_)
	{
		OutPoints.Add(FVector(APoint.x(), APoint.y(), APoint.z()));
	}
	//UE_LOG(LogTemp, Warning, TEXT("Outpoints First Value: %s"), *OutPoints[0].ToString());
}

void FOpen3DUE5Module::VoxelizedArrFromPoints(TArray<FVector3f> InPoints, double VoxelSize, TArray<FIntVector>& OutVoxelizedArr, FIntVector& CalcedRealSize)
{
	std::vector<Eigen::Vector3d> EigenPoints = {};

	for (auto APoint : InPoints)
	{
		EigenPoints.push_back(Eigen::Vector3d(APoint.X, APoint.Y, APoint.Z));
	}

	if (InPoints.Num() > 0)
	{
		open3d::geometry::PointCloud MyPointCloud = open3d::geometry::PointCloud(EigenPoints);

		auto Open3DVoxelizedGrid = open3d::geometry::VoxelGrid::CreateFromPointCloud(MyPointCloud, VoxelSize);

		// warning: unreal overrides new and delete in IMPLEMENT_MODULE, which crashes with the standards operator delete for std::vector, causing runtime UB
		// causing the heap memory to be broken and thus cause an access violation/integer divide by zero error
		// when _Deallocate / delete is called
		//auto VoxelList = new std::vector<open3d::geometry::Voxel>(Open3DVoxelizedGrid->GetVoxels());

		auto RealSizeEigen = Open3DVoxelizedGrid->GetAxisAlignedBoundingBox().GetExtent();
		const FVector RealSizeFloat = FVector(RealSizeEigen.x(), RealSizeEigen.y(), RealSizeEigen.z());
		const FIntVector RealSize(
			FIntVector(
				FMath::CeilToInt(RealSizeFloat.X),
				FMath::CeilToInt(RealSizeFloat.Y),
				FMath::CeilToInt(RealSizeFloat.Z)
			)
		);

		OutVoxelizedArr.RemoveAll([](FIntVector Vector) { return true; });

		// because of the above issue, access voxels_ directly instead of getting it as an std::vector through a copy constructor 
		for (auto AVoxel : Open3DVoxelizedGrid->voxels_)
		{
			int XCoord = AVoxel.second.grid_index_.x();
			int YCoord = AVoxel.second.grid_index_.y();
			int ZCoord = AVoxel.second.grid_index_.z();

			OutVoxelizedArr.Add(FIntVector(XCoord, YCoord, ZCoord));
		}

		CalcedRealSize = RealSize;
	}
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FOpen3DUE5Module, Open3DUE5)
