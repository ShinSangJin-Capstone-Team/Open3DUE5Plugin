// Copyright Epic Games, Inc. All Rights Reserved.

#include "Open3DUE5.h"
#include "Core.h"
#include "HAL/Platform.h"
#include "Containers/Array.h"
#include "Math/MathFwd.h"

#include "Modules/ModuleManager.h"
#include "Interfaces/IPluginManager.h"

/**/
#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#include "Windows/WindowsHWrapper.h"
#endif
//*/
#include <vector>
#include <memory>
#include <algorithm>

/**/
#if PLATFORM_WINDOWS
#include "Windows/HideWindowsPlatformTypes.h"
#endif
//*/

THIRD_PARTY_INCLUDES_START
#pragma push_macro("check")   // store 'check' macro current definition
#undef check  // undef to avoid conflicts
#include "open3d/Open3D.h"
#pragma pop_macro("check")  // restore definition
THIRD_PARTY_INCLUDES_END

#define LOCTEXT_NAMESPACE "FOpen3DUE5Module"

void FOpen3DUE5Module::StartupModule()
{
	// This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module

	// Get the base directory of this plugin
	FString BaseDir = IPluginManager::Get().FindPlugin("Open3DUE5")->GetBaseDir();

	// Add on the relative location of the third party dll and load it
	FString LibraryPath;
	/**/
#if PLATFORM_WINDOWS
	LibraryPath = FPaths::Combine(*BaseDir, TEXT("Binaries/ThirdParty/Open3D/bin/Open3D.dll"));
#endif // PLATFORM_WINDOWS

	Open3DHandle = !LibraryPath.IsEmpty() ? FPlatformProcess::GetDllHandle(*LibraryPath) : nullptr;
	//*/
}

void FOpen3DUE5Module::ShutdownModule()
{
	// This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
	// we call this function before unloading the module.

	// Free the dll handle
	FPlatformProcess::FreeDllHandle(Open3DHandle);
	Open3DHandle = nullptr;
}

static Eigen::Vector3d XYZSpherical(Eigen::Vector3d XYZ)
{
	auto x = XYZ[0];
	auto y = XYZ[1];
	auto z = XYZ[2];
	auto r = sqrt(x * x + y * y + z * z);
	auto r_x = acos(y / r);
	auto r_y = atan2(z, x);
	return Eigen::Vector3d(r, r_x, r_y);
}

static Eigen::Matrix3d GetRotationMatrix(double R_X, double R_Y)
{
	auto rot_x = Eigen::Matrix3d {
		{1.0, 0.0, 0.0},
		{0.0, cos(R_X), -sin(R_X)},
		{0.0, sin(R_X), cos(R_X)}
	};
	auto rot_y = Eigen::Matrix3d {
		{cos(R_Y), 0.0, sin(R_Y)},
		{0.0, 1.0, 0.0},
		{-sin(R_Y), 0.0, cos(R_Y)}
	};

	return rot_y * rot_x;
}
//*/

/**/
static Eigen::Matrix4d GetExtrinsic(Eigen::Vector3d XYZ)
{
	auto rvec = XYZSpherical(XYZ);
	auto r = GetRotationMatrix(rvec[1], rvec[2]);
	Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
	trans.topLeftCorner<3, 3>() = r.block<3,3>(0,0);
	auto t = Eigen::Vector3d(0, 0, 2);
	trans.topRightCorner<3, 1>() = t.head<3>();
	return trans;
}
//*/
/**/
static void PreProcess(std::shared_ptr<open3d::geometry::TriangleMesh> model)
{
	auto min_bound = model->GetMinBound();
	auto max_bound = model->GetMaxBound();
	Eigen::Vector3d center = min_bound + 0.5 * (max_bound - min_bound);
	auto scale = 0.5 * ((max_bound - min_bound).norm());
	
	for (auto& x : model->vertices_)
	{
		x -= center;
		x = x / scale;
	}
}
//*/

void FOpen3DUE5Module::VoxelizedArrFromPoints(TArray<FVector3f> InPoints, double VoxelSize, TArray<FIntVector>& OutVoxelizedArr, FIntVector& CalcedRealSize)
{
	std::vector<Eigen::Vector3d> EigenPoints = {};

	for (auto APoint : InPoints)
	{
		EigenPoints.push_back(Eigen::Vector3d(APoint.X, APoint.Y, APoint.Z));
	}

	if (InPoints.Num() > 0)
	{
		auto MeshifiedPointCloud =
			open3d::geometry::TriangleMesh::CreateFromPointCloudWithoutNormals(
				open3d::geometry::PointCloud(EigenPoints),
				100
			);

		MeshifiedPointCloud->ComputeVertexNormals();

		auto MeshScaleFactor = 0.5 * ((MeshifiedPointCloud->GetMaxBound() - MeshifiedPointCloud->GetMinBound()).norm());
		// Eigen::Vector3d MeshCenterOffset = MeshifiedPointCloud->GetMinBound() + 0.5 * (MeshifiedPointCloud->GetMaxBound() - MeshifiedPointCloud->GetMinBound());

		/**
		for (auto& Point : EigenPoints)
		{
			Point += MeshCenterOffset;
		}
		//*/

		auto Open3DVoxelizedGrid = open3d::geometry::VoxelGrid::CreateFromPointCloud(open3d::geometry::PointCloud(EigenPoints), VoxelSize);
		//auto Open3DVoxelizedGrid = open3d::geometry::VoxelGrid::CreateFromTriangleMesh(*MeshifiedPointCloud, VoxelSize);

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
		/**/
		for (auto AVoxel : Open3DVoxelizedGrid->voxels_)
		{
			int XCoord = AVoxel.second.grid_index_.x();
			int YCoord = AVoxel.second.grid_index_.y();
			int ZCoord = AVoxel.second.grid_index_.z();

			OutVoxelizedArr.Add(FIntVector(XCoord, YCoord, ZCoord));
		}
		//*/

		// Carve

		auto w = 400;
		auto h = 400;

		auto CarvingCloud =
			open3d::geometry::VoxelGrid::CreateDense(
				Eigen::Vector3d(-RealSizeEigen.x()/2.0, -RealSizeEigen.y()/2.0, -RealSizeEigen.z()/2.0),
				Eigen::Vector3d(1.0, 0.7, 0.0),
				VoxelSize,
				RealSizeEigen.x(),
				RealSizeEigen.y(),
				RealSizeEigen.z()
			);

		/**/
		auto CameraSphere = open3d::geometry::TriangleMesh::CreateSphere(1.0, 10);
		
		//UE_LOG(LogTemp, Warning, TEXT("Before: %f %f %f"), CameraSphere->vertices_[0].x(), CameraSphere->vertices_[0].y(), CameraSphere->vertices_[0].z());
		PreProcess(CameraSphere);
		//UE_LOG(LogTemp, Warning, TEXT("After: %f %f %f"), CameraSphere->vertices_[0].x(), CameraSphere->vertices_[0].y(), CameraSphere->vertices_[0].z());
		
		/**/
		//UE_LOG(LogTemp, Warning, TEXT("Before: %f %f %f"), MeshifiedPointCloud->vertices_[0].x(), MeshifiedPointCloud->vertices_[0].y(), MeshifiedPointCloud->vertices_[0].z());
		PreProcess(MeshifiedPointCloud);
		//UE_LOG(LogTemp, Warning, TEXT("After: %f %f %f"), MeshifiedPointCloud->vertices_[0].x(), MeshifiedPointCloud->vertices_[0].y(), MeshifiedPointCloud->vertices_[0].z());
		//*/

		auto params = open3d::camera::PinholeCameraParameters();

		/**/
		auto vis = open3d::visualization::Visualizer();
		vis.CreateVisualizerWindow("Open3D", w, h, 0,0, false);
		
		vis.AddGeometry(MeshifiedPointCloud, true);
		vis.GetRenderOption().mesh_show_back_face_ = true;
		
		vis.GetViewControl().ConvertToPinholeCameraParameters(params);

		UE_LOG(LogTemp, Warning, TEXT("CameraSphere Vertices: %d"), CameraSphere->vertices_.size());

		for (auto i = 0; i < (int)CameraSphere->vertices_.size(); ++i)
		{
			auto XYZ = CameraSphere->vertices_[i];
			auto trans = GetExtrinsic(XYZ);

			/**
			UE_LOG(LogTemp, Warning, TEXT("%f %f %f %f"), trans(0,0), trans(0, 1), trans(0, 2), trans(0, 3));
			UE_LOG(LogTemp, Warning, TEXT("%f %f %f %f"), trans(1, 0), trans(1, 1), trans(1, 2), trans(1, 3));
			UE_LOG(LogTemp, Warning, TEXT("%f %f %f %f"), trans(2, 0), trans(2, 1), trans(2, 2), trans(2, 3));
			UE_LOG(LogTemp, Warning, TEXT("%f %f %f %f"), trans(3, 0), trans(3, 1), trans(3, 2), trans(3, 3));
			//*/
			params.extrinsic_ = trans;

			vis.GetViewControl().ConvertFromPinholeCameraParameters(params, true);

			vis.PollEvents();
			vis.UpdateRender();

			auto Depth = vis.CaptureDepthFloatBuffer(false);
			
			//UE_LOG(LogTemp, Warning, TEXT("camera trans before: %f"), params.extrinsic_(2, 3));
			//Eigen::Vector3d tempoffset;
			//tempoffset.head<3>() = params.extrinsic_.topLeftCorner<3, 3>().transpose() * MeshCenterOffset.head<3>();
			params.extrinsic_(2, 3) = params.extrinsic_(2, 3) * MeshScaleFactor;
			//params.extrinsic_.topRightCorner<3, 1>() += 3.0 * (tempoffset / MeshScaleFactor).head<3>();
			//UE_LOG(LogTemp, Warning, TEXT("camera trans after: %f"), params.extrinsic_(2, 3));


			CarvingCloud->CarveDepthMap(*Depth, params, false);
			//CarvingCloud->CarveSilhouette(open3d::geometry::Image(*Depth), params, true);
			UE_LOG(LogTemp, Warning, TEXT("Carve view %03d / %03d"), i + 1, CameraSphere->vertices_.size());
		}
		vis.DestroyVisualizerWindow();
		//*/

		//Eigen::Vector3d tmp = (Open3DVoxelizedGrid->origin_ - CarvingCloud->origin_) / (VoxelSize * MeshScaleFactor);

		for (auto AVoxel : CarvingCloud->voxels_)
		{
			int XCoord = AVoxel.second.grid_index_.x();// -tmp.x();
			int YCoord = AVoxel.second.grid_index_.y();// -15;
			int ZCoord = AVoxel.second.grid_index_.z();// -tmp.z();

			OutVoxelizedArr.Add(FIntVector(XCoord, YCoord, ZCoord));
		}
		//*/

		CalcedRealSize = RealSize;
	}
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FOpen3DUE5Module, Open3DUE5)
