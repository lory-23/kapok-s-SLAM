/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"

#include"DynaRadar.h"
#include<ros/ros.h>



 
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"


#define ridarFrame "radar_frame";
using namespace std;

void pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& mmwCloudMsg);
class DataGrabber
{
public:
    DataGrabber(ORB_SLAM3::System* pSLAM,DynaRadar::DynaRadar* pRadar):mpSLAM(pSLAM),mpRadar(pRadar){}
    void pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& mmwCloudMsg);
    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;
    DynaRadar::DynaRadar* mpRadar;
    cv::Mat lastImage;
    cv::Mat currentImage;
};
//argc是从参数数量，argv是参数队列
int main(int argc, char **argv)
{
    string running_path = get_current_dir_name();
    cout<<running_path<<endl;
    ros::init(argc, argv, "Mono_radar");      //初始化节点
    ros::start();

    // cv::namedWindow("fusion", cv::WINDOW_AUTOSIZE);
   
   
    argv[1] = "../../../Vocabulary/ORBvoc.txt";
    argv[2] = "./setting/RealSense_D435.yaml";

    char i =0;
    for(i=0;i<3;i++)
        printf(argv[i]);

	string strSettingsFile = "./setting/DynaRadar.yaml";
	
	cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
   	    cout <<"read matrix from file"<<endl;
       if(!fsSettings.isOpened())
       	{
       		cerr << "Failed to open settings file at: " << strSettingsFile << endl;
      		exit(-1);
   		}

		cv::Mat R_cam_mat = cv::Mat(4,4,CV_64FC1,cv::Scalar::all(0)) ;
		cv::Mat P_cam_mat = cv::Mat(4,4,CV_64FC1,cv::Scalar::all(0)) ;
		cv::Mat extrinsicT_mat = cv::Mat(4,4,CV_64FC1,cv::Scalar::all(0)) ;
       fsSettings["R_cam_mat"]>>R_cam_mat;
       cout<<R_cam_mat;

    DynaRadar::DynaRadar radar(false,false,false);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(&radar,argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);
    DataGrabber dgb(&SLAM,&radar);
    ros::NodeHandle nodeHandler;
   
   
    // ros::Subscriber sub = nodeHandler.subscribe("/usb_cam/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    ros::Subscriber sub = nodeHandler.subscribe("/camera/color/image_raw", 1, &DataGrabber::GrabImage,&dgb);
    


	ros::Subscriber pointCloudSub = nodeHandler.subscribe("/radar_enhanced_pcl", 1 ,&DataGrabber::pointCloudCallback,&dgb );
    
    ros::spin();
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_radar.txt");

    ros::shutdown();

    return 0;
}

void DataGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv::Mat proj_img;
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg,"bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // proj_img =  mpRadar->drawDoppler(cv_ptr->image);
    // proj_img =  mpRadar->drawPoint(cv_ptr->image,mpRadar->getvMoveRadarPoints2d(),cv::Scalar(0,255,0));
    // imshow("fusion" , proj_img);
    lastImage = currentImage;
    currentImage = cv_ptr->image;
    mpRadar->saveImg(cv_ptr->image,cv_ptr->header.stamp.toNSec()/1000);
    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
}


void DataGrabber::pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& mmwCloudMsg)
{
  
   sensor_msgs::PointCloud2 radarCloudMsg2;
    convertPointCloudToPointCloud2(*mmwCloudMsg, radarCloudMsg2);
    radarCloudMsg2.header.stamp = mmwCloudMsg->header.stamp;
    radarCloudMsg2.header.frame_id = ridarFrame;  //坐标frame
    pcl::PCLPointCloud2 radarCloudpcl2;
    pcl_conversions::toPCL(radarCloudMsg2, radarCloudpcl2);
    mpRadar->HandelPointCloud(radarCloudpcl2);
   

}
