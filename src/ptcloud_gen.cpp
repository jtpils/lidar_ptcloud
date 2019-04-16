#include "ros/ros.h"//ros
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud.h"
#include "../include/api.h"//api interface
#include <sstream>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

ros::Publisher ptcloudL_pub, ptcloudR_pub;//Global variable, because the observer callback function needs to be used

//The observer callback function
void *User_Func(HPS3D_HandleTypeDef *handle, AsyncIObserver_t *event)
{
	sensor_msgs::PointCloud measureData;	
	measureData.points.resize(MAX_PIX_NUM);

	ros::Time scan_time = ros::Time::now();
	//populate the PointCloud message
	measureData.header.stamp = scan_time;
	measureData.header.frame_id = "hps";

	if(event->AsyncEvent == ISubject_Event_DataRecvd)
	{
		switch(event->RetPacketType)
		{
			// case SIMPLE_ROI_PACKET:
			// 	printf("distance = %d  event->RetPacketType = %d\n",event->MeasureData.simple_roi_data[0].distance_average,event->RetPacketType);
			// 	break;
			// case FULL_ROI_PACKET:
			// 	break;
			case FULL_DEPTH_PACKET: /*点云数据和深度数据在这里获取*/
				for(int i = 0; i < MAX_PIX_NUM; i++){
					geometry_msgs::Point32 pt;
					pt.x = event->MeasureData.point_cloud_data[0].point_data[i].x;
					pt.y = event->MeasureData.point_cloud_data[0].point_data[i].y;
					pt.z = event->MeasureData.point_cloud_data[0].point_data[i].z;
					measureData.points[i] = pt;
				}
				//48 49
				switch (handle->DeviceName[11])
				{
				case 48:
					ptcloudL_pub.publish(measureData);
					break;
				case 49:
					ptcloudR_pub.publish(measureData);
					break;
				default:
					printf("Not ttyACM*!\n");
					break;
				}
			// case SIMPLE_DEPTH_PACKET:
			// 	printf("distance = %d  event->RetPacketType = %d\n",event->MeasureData.simple_depth_data->distance_average,event->RetPacketType);
			// 	break;
			// case OBSTACLE_PACKET:
			// 	printf("Obstacle ID：%d\n",event->MeasureData.Obstacle_data->Id);
			// 	if(event->MeasureData.Obstacle_data->Id > 20)
			// 	{
			// 		handle->RunMode = RUN_IDLE;
			// 		HPS3D_SetRunMode(handle);
			// 	}
			// 	break;
			// case NULL_PACKET:
			// 	printf("null packet\n");
			// 	//The return packet type is empty
			// 	break;
			default:
				printf("system error!\n");
				break;
		}
	}
}

//printf log callback function
void my_printf(uint8_t *str)
{
	std::cout<< str;
}


int main(int argc, char **argv)
{
	system("sudo chmod 777 /dev/ttyACM*");
	
	ros::init(argc, argv, "ptcloud_gen");//ros init
	ros::NodeHandle n;//Create a node

	uint32_t a = 0;
	uint8_t fileName[10][20];
	uint32_t dev_cnt = 0;
	uint8_t dev[13] = "/dev/ttyACM*";
	RET_StatusTypeDef ret = RET_OK;

	//Lists the optional devices
	dev_cnt = HPS3D_GetDeviceList((uint8_t *)"/dev/",(uint8_t *)"ttyACM",fileName);

	HPS3D_HandleTypeDef handle[dev_cnt];
	AsyncIObserver_t Observer[dev_cnt];

	//Create a topic
	ptcloudL_pub = n.advertise<sensor_msgs::PointCloud>("ptcloudL", 1000);
	ptcloudR_pub = n.advertise<sensor_msgs::PointCloud>("ptcloudR", 1000);

	//set debug enable and install printf log callback function
	HPS3D_SetDebugEnable(false);
	HPS3D_SetDebugFunc(&my_printf);
	HPS3D_SetPointCloudEn(true);


	for (int i = 0; i < dev_cnt; i++){
		const size_t len = strlen((const char *)fileName[i]);
		dev[11] = (uint8_t)fileName[i][len-1];
		handle[i].DeviceName = (uint8_t *) dev;
		printf("%s\n", handle[i].DeviceName);

		//Device Connection
		ret = HPS3D_Connect(&(handle[i]));
		if(ret != RET_OK)
		{
			printf("Device open failed！ret = %d\n",ret);
			return 1;
		}
		
		//Point Data Setting
		HPS3D_SetOpticalEnable(&(handle[i]), true);

		//Device init
		ret = HPS3D_ConfigInit(&(handle[i]));
		if(RET_OK != ret)
		{
			printf("Initialization failed:%d\n", ret);
			return 1;
		}
		printf("Initialization succeed\n");

		//Observer callback function and initialization
		Observer[i].AsyncEvent = ISubject_Event_DataRecvd;
		Observer[i].NotifyEnable = true;
		Observer[i].ObserverID = i;
		Observer[i].RetPacketType = NULL_PACKET;

		handle[i].OutputPacketType = PACKET_FULL;
		HPS3D_SetPacketType(&(handle[i]));

		//Add observer one
		HPS3D_AddObserver(&User_Func, &(handle[i]), &(Observer[i]));	

		if(ret != RET_OK)
		{
			//Remove device and disconnect
			HPS3D_RemoveDevice(&(handle[i]));
			printf("Initialization failed, Remove device\n");
			return 1;
		}

		//Set running mode
		handle[i].SyncMode = ASYNC;
		handle[i].RunMode = RUN_CONTINUOUS;
		HPS3D_SetRunMode(&(handle[i]));
	}

	while(ros::ok())
	{		

	}

	for (int i = 0; i < dev_cnt; i++){
		if(HPS3D_RemoveDevice(&(handle[i])) != RET_OK)
		{
			printf("HPS3D_RemoveDevice faild\n");
		}
		else
		{	
			printf("HPS3D_RemoveDevice succeed\n");
		}
		HPS3D_DisConnect(&(handle[i]));
		HPS3D_RemoveObserver(&(Observer[i]));
	}
	return 0;
}

