#include <iostream>
#include <tchar.h>
#include "HPS3D160_IF/HPS3DUser_IF.h"

//preallocate memory
static HPS3D_MeasureData_t g_measureData;

static bool PrintResultData(HPS3D_EventType_t type, HPS3D_MeasureData_t data)
{
	int num = 0;
	switch (type)
	{
	case HPS3D_SIMPLE_ROI_EVEN: //Simple ROI data packet without depth data for each point
		printf("*************  HPS3D_SIMPLE_ROI_EVEN  ********************\n");
		num = data.simple_roi_data[0].roi_num;
		for (int i = 0; i < num; i++)
		{
			printf("  ********GroupID:%d  ROIID:%d  *******\n", data.simple_roi_data[i].group_id, data.simple_roi_data[i].roi_id);
			printf("    distance_average:%d\n", data.simple_roi_data[i].distance_average);
			printf("    distance_min    :%d\n", data.simple_roi_data[i].distance_min);
			printf("    saturation_count:%d\n", data.simple_roi_data[i].saturation_count);
			printf("    threshold_state :%d\n", data.simple_roi_data[i].threshold_state);
			printf("    =====================================\n\n");
		}
		break;
	case HPS3D_FULL_ROI_EVEN: //Complete ROI data packet
		printf("*************  HPS3D_FULL_ROI_EVEN  ********************\n");
		num = data.full_roi_data[0].roi_num;
		for (int i = 0; i < num; i++)
		{
			printf("  ********GroupID:%d  ROIID:%d  *******\n", data.full_roi_data[i].group_id, data.full_roi_data[i].roi_id);
			printf("    ROI Left Top    :(%d,%d)\n", data.full_roi_data[i].left_top_x, data.full_roi_data[i].left_top_y);
			printf("    ROI Right Bottom:(%d,%d)\n", data.full_roi_data[i].right_bottom_x, data.full_roi_data[i].right_bottom_y);
			printf("    ROI Pixel Number:%d\n", data.full_roi_data[i].pixel_number);
			printf("    distance_average:%d\n", data.full_roi_data[i].distance_average);
			printf("    distance_min    :%d\n", data.full_roi_data[i].distance_min);
			printf("    saturation_count:%d\n", data.full_roi_data[i].saturation_count);
			printf("    threshold_state :%d\n", data.full_roi_data[i].threshold_state);
			printf("    =====================================\n\n");
		}
		break;
	case HPS3D_SIMPLE_DEPTH_EVEN: //Simple depth data packet without distance for each point and point cloud data
		printf("*************  HPS3D_SIMPLE_DEPTH_EVEN  ********************\n");
		printf(" distance_average:%d\n", data.simple_depth_data.distance_average);
		printf(" distance_min    :%d\n", data.simple_depth_data.distance_min);
		printf(" saturation_count:%d\n", data.simple_depth_data.saturation_count);
		printf("==========================================================\n\n");
		break;
	case HPS3D_FULL_DEPTH_EVEN: //Complete depth map data packet with point cloud data
		printf("*************  HPS3D_FULL_DEPTH_EVEN    ********************\n");
		printf("distance_average:%d\n", data.full_depth_data.distance_average);
		printf("distance_min    :%d\n", data.full_depth_data.distance_min);
		printf("saturation_count:%d\n", data.full_depth_data.saturation_count);

		printf("distance[0]     :%d\n", data.full_depth_data.distance[0]);
		printf("pointCloud[0]   :(%f,%f.%f)\n", data.full_depth_data.point_cloud_data.point_data[0].x,
			data.full_depth_data.point_cloud_data.point_data[0].y, data.full_depth_data.point_cloud_data.point_data[0].z);

		printf("distance[1]     :%d\n", data.full_depth_data.distance[1]);
		printf("pointCloud[1]   :(%f,%f.%f)\n", data.full_depth_data.point_cloud_data.point_data[1].x,
			data.full_depth_data.point_cloud_data.point_data[1].y, data.full_depth_data.point_cloud_data.point_data[1].z);

		printf("==========================================================\n\n");
		break;
	default:
		break;
	}

	return true;
}

static void EventCallBackFunc(int handle, int eventType, uint8_t *data, int dataLen, void *userPara)
{
	switch ((HPS3D_EventType_t)eventType)
	{
		//Measurement data notification event
	case HPS3D_SIMPLE_ROI_EVEN:
	case HPS3D_FULL_ROI_EVEN:
	case HPS3D_FULL_DEPTH_EVEN:
	case HPS3D_SIMPLE_DEPTH_EVEN:
		HPS3D_ConvertToMeasureData(data, &g_measureData, (HPS3D_EventType_t)eventType);
		PrintResultData((HPS3D_EventType_t)eventType, g_measureData);
		break;
	case HPS3D_SYS_EXCEPTION_EVEN: /*System anomaly notification event*/
		printf("SYS ERR :%s\n", data);
		break;
	case HPS3D_DISCONNECT_EVEN: /*Connection abnormal disconnection notification event*/
		printf("Device disconnected!\n");
		HPS3D_CloseDevice(handle);
		HPS3D_MeasureDataFree(&g_measureData);
		break;
	case HPS3D_NULL_EVEN:  //Empty event notification
	default:
		break;
	}
}

static bool PrintDepthData(HPS3D_MeasureData_t data)
{
	int num = 0;
		printf("*************  HPS3D_FULL_DEPTH_EVEN    ********************\n");
		printf("distance_average:%d\n", data.full_depth_data.distance_average);
		printf("distance_min    :%d\n", data.full_depth_data.distance_min);
		printf("saturation_count:%d\n", data.full_depth_data.saturation_count);

		for (int i = 0; i < 60; i++)
		{
			for (int j = 0; j < 160; j++)
			{
				printf("%1d", (int)data.full_depth_data.point_cloud_data.point_data[i*160 + j].z/1000);
			}
			std::cout << std::endl;
		}

		printf("==========================================================\n\n");
	return true;
}

int main()
{
	printf("HPS3D160 C/C++ Demo (Visual Statudio 2017)\n\n");

	printf("SDK Ver:%s\n", HPS3D_GetSDKVersion());

	//Memroy allocation
	HPS3D_MeasureDataInit(&g_measureData);

	int handle = -1;
	HPS3D_StatusTypeDef ret = HPS3D_RET_OK;
	do
	{
		printf("Device Type Selection: HPS3D160-U(1)  HPS3D160-L(2)  Exit(Other)\n");
		char sel = getchar();
		getchar(); //Filter Carriage Return
		if (sel == '1')
		{
			ret = HPS3D_USBConnectDevice((char *)_T("COM6"), &handle);
			//printf("%d\n", handle);
		}
		else if (sel == '2')
		{
			while(1)
			{ 
				ret = HPS3D_EthernetConnectDevice((char *)"192.168.0.10", 12345, &handle);
				if (ret == HPS3D_RET_OK)
					break;
				else if (ret != HPS3D_RET_OK)
				{
					printf("Device connection failed,Err:%d\n", ret);
					//break;
				}
			}
				
		}
		else
		{
			return 0;
		}
		if (ret != HPS3D_RET_OK)
		{
			printf("Device connection failed here,Err:%d\n", ret);
			break;
		}


		//Register event callback function for receiving continuous data packets and handling exceptions
		HPS3D_RegisterEventCallback(EventCallBackFunc, NULL);

		printf("Device version:%s\n", HPS3D_GetDeviceVersion(handle));
		printf("Device serial number:%s\n\n", HPS3D_GetSerialNumber(handle));

		HPS3D_DeviceSettings_t settings;
		ret = HPS3D_ExportSettings(handle, &settings);
		if (ret != HPS3D_RET_OK)
		{
			printf("Failed to export device parameters,Err:%d\n", ret);
			break;
		}
		printf("Resolution:%d X %d\n", settings.max_resolution_X, settings.max_resolution_Y);
		printf("Maximum supported number of ROI groups:%d  Current ROI group：%d\n", settings.max_roi_group_number, settings.cur_group_id);
		printf("Maximum supported number of ROIs:%d\n", settings.max_roi_number);
		printf("Maximum supported number of multi-machine identifiers:%d, Current device multi-machine identifier:%d\n", settings.max_multiCamera_code, settings.cur_multiCamera_code);
		printf("Current device user ID:%d\n", settings.user_id);
		printf("Optical path compensation turned on: %d\n\n", settings.optical_path_calibration);

		bool isContinuous = false;
		do
		{
			printf("Select measurement mode: Single measurement(1) Continuous measurement(2) Exit(Other)\n");
			char sel = getchar();
			getchar(); //Filter Carriage Return
			if (sel == '1')
			{
				//Single measurement mode
				HPS3D_EventType_t type = HPS3D_NULL_EVEN;
				ret = HPS3D_SingleCapture(handle, &type, &g_measureData);
				if (ret != HPS3D_RET_OK)
				{
					printf("Single measurement failed,Err:%d\n", ret);
					break;
				}
				//PrintResultData(type, g_measureData);
				PrintDepthData(g_measureData);
			}
			else if (sel == '2')
			{
				isContinuous = true;
				HPS3D_StartCapture(handle);
				break;
			}
			else
			{
				isContinuous = false;
				break;
			}
		} while (1);

		if (isContinuous)
		{
			while (1)
			{
				Sleep(10);
			}
		}
	} while (0);

	HPS3D_StopCapture(handle);
	HPS3D_CloseDevice(handle);
	HPS3D_MeasureDataFree(&g_measureData);
	system("pause");
}


