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

void Render(sy3::depth_frame* piexls_depth, sy3::frame* piexls_rgb, int width, int height)
{
	std::cout << width << std::endl;
	std::cout << height << std::endl;

	cv::Mat gray16(piexls_depth->get_height(), piexls_depth->get_width(), CV_16UC1, piexls_depth->get_data());
	cv::Mat tmp;
	cv::Mat gray8 = cv::Mat::zeros(gray16.size(), CV_8U);
	cv::normalize(gray16, tmp, 0, 255, cv::NORM_MINMAX);
	cv::convertScaleAbs(tmp, gray8);
	cv::Mat yuvImg(piexls_rgb->get_height(), piexls_rgb->get_width(), CV_8UC3, piexls_rgb->get_data());
	//RGBD
	cv::Mat rgbd_img(height, width, CV_8UC3, cv::Scalar(0));// rgb_img;
	for (int row = 0; row < yuvImg.rows; row++) {
		for (int col = 0; col < yuvImg.cols; col++)
		{
			if (gray16.ptr<uint16_t>(row)[col] > 10)
			{
				rgbd_img.ptr<uchar>(row)[col * 3] = yuvImg.ptr<uchar>(row)[col * 3];
				rgbd_img.ptr<uchar>(row)[col * 3 + 1] = yuvImg.ptr<uchar>(row)[col * 3 + 1];
				rgbd_img.ptr<uchar>(row)[col * 3 + 2] = yuvImg.ptr<uchar>(row)[col * 3 + 2];
			}
		}
	}
	cv::namedWindow(RGBD_WINDOW_NAME, cv::WINDOW_NORMAL);
	cv::resizeWindow(RGBD_WINDOW_NAME, RGB_WIDTH, RGB_HEIGHT);
	cv::imshow(RGBD_WINDOW_NAME, rgbd_img);
}


void show_depth_frame(sy3::depth_frame* frame, const char* name)
{
	if (frame)
	{

		cv::Mat gray16(frame->get_height(), frame->get_width(), CV_16UC1, frame->get_data());
		cv::Mat tmp;
		cv::Mat gray8 = cv::Mat::zeros(gray16.size(), CV_8U);
		cv::normalize(gray16, tmp, 0, 255, cv::NORM_MINMAX);
		cv::convertScaleAbs(tmp, gray8);
		cv::namedWindow(name, cv::WINDOW_NORMAL);
		cv::imshow(name, gray8);

	}
}

void show_rgb_rgb_frame(sy3::frame* frame, const char* name)
{
	if (frame)
	{

		cv::Mat yuvImg(frame->get_height(), frame->get_width(), CV_8UC3, frame->get_data());
		//cv::Mat rgbImg(frame->get_height(), frame->get_width(), CV_8UC3);
		//	cv::cvtColor(yuvImg, rgbImg, cv::ColorConversionCodes::COLOR_BGR2BGR);
		cv::namedWindow(name, cv::WINDOW_NORMAL);
		cv::imshow(name, yuvImg);
		const sy3::stream_profile* rgb_profile = frame->get_profile();
		sy3::sy3_intrinsics rgb_inteinics = rgb_profile->get_intrinsics();

	}
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

	std::vector<sy3::sy3_stream> support_stream = dev->get_support_stream(e);
	for (int i = 0; i < support_stream.size(); i++)
	{
		UE_LOG(LogTemp, Warning, TEXT("support stream : % s \n"), sy3_stream_to_string(support_stream[i]));
		printf("support stream:%s \n", sy3_stream_to_string(support_stream[i]));
		std::vector<sy3::sy3_format> support_format = dev->get_support_format(support_stream[i], e);
		for (int j = 0; j < support_format.size(); j++)
		{
			UE_LOG(LogTemp, Warning, TEXT("\t\t support format:%d x %d \n"), support_format[j].width, support_format[j].height);
			printf("\t\t support format:%d x %d \n", support_format[j].width, support_format[j].height);
		}
	}
}

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
		auto LibraryPath = FPaths::Combine(*BaseDir, *Path);
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

void FOpen3DUE5Module::InitSensor()
{
	UE_LOG(LogTemp, Warning, TEXT("version: % s"), sy3::sy3_get_version(e));

	printf("version:%s \n", sy3::sy3_get_version(e));
	ctx = sy3::sy3_create_context(e);
	sy3::device* dev = ctx->query_device(e);

	if (e != sy3::sy3_error::SUCCESS) {
		UE_LOG(LogTemp, Warning, TEXT("error: %d %s \n"), e, sy3::sy3_error_to_string(e));
		printf("error:%d  %s \n", e, sy3::sy3_error_to_string(e));
		return;
	}

	print_support_format(dev, e);
	print_device_info(dev);

	pline = sy3::sy3_create_pipeline(ctx, e);

	sy3::config* cfg = sy3_create_config(ctx, e);
	cfg->enable_stream(sy3::sy3_stream::SY3_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, e);
	cfg->enable_stream(sy3::sy3_stream::SY3_STREAM_RGB, RGB_WIDTH, RGB_HEIGHT, e);

	pline->start(cfg, e);
}

void FOpen3DUE5Module::GetSensorOneFrame(TArray<FVector>& Points)
{
	if (!pline)
	{
		if (!ctx)
		{
			ctx = sy3::sy3_create_context(e);
			sy3::device* dev = ctx->query_device(e);

			if (e != sy3::sy3_error::SUCCESS) {
				UE_LOG(LogTemp, Warning, TEXT("error: %d %s \n"), e, sy3::sy3_error_to_string(e));
				printf("error:%d  %s \n", e, sy3::sy3_error_to_string(e));
				Points.Empty();
				Points.Add(FVector(-1.0, -1.0, -1.0));
				return;
			}

			print_support_format(dev, e);
			print_device_info(dev);
		}

		pline = sy3::sy3_create_pipeline(ctx, e);

		sy3::config* cfg = sy3_create_config(ctx, e);
		cfg->enable_stream(sy3::sy3_stream::SY3_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, e);
		cfg->enable_stream(sy3::sy3_stream::SY3_STREAM_RGB, RGB_WIDTH, RGB_HEIGHT, e);

		pline->start(cfg, e);
	}
	else
	{
		Points.Empty();

		sy3::frameset* frameset = pline->wait_for_frames(SY3_DEFAULT_TIMEOUT, e);
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
				sy3::points* points = pline->get_process_engin(e1)->comptute_points(depth_frame, true, e1);
				float* data = points->get_points();
				std::cout << points->get_length() << std::endl;
				/**/
				for (auto i = 0; i < height; i++)
				{
					for (auto j = 0; j < width; j++)
					{
						if (data[(3 * width) * i + 3 * j + 2] > 0)
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
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FOpen3DUE5Module, Open3DUE5)
