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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

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
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM3::System* mpSLAM;
    DynaRadar::DynaRadar* mpRadar;
    cv::Mat lastImage;
    cv::Mat currentImage;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    argv[1] = "../../../Vocabulary/ORBvoc.txt";
    argv[2] = "./setting/RealSense_D435.yaml";

/*     if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    
 */
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
    ORB_SLAM3::System SLAM(&radar,argv[1],argv[2],ORB_SLAM3::System::RGBD,true);

    DataGrabber dgb(&SLAM,&radar);

    ros::NodeHandle nh;

    // message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 100);
    // message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 100);
    // 以下为tum rgbd 数据集的topic
    // message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_color", 100);
    // message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image", 100);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh,"/camera/color/image_raw",1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh,"/camera/depth/image_rect_raw",1);
	ros::Subscriber pointCloudSub = nh.subscribe("/radar_enhanced_pcl", 1 ,&DataGrabber::pointCloudCallback,&dgb );


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&DataGrabber::GrabRGBD,&dgb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_RGBD_radar.txt");

    ros::shutdown();

    return 0;
}

void DataGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    lastImage = currentImage;
    currentImage = cv_ptrRGB->image;
    mpRadar->saveImg(cv_ptrRGB->image,cv_ptrRGB->header.stamp.toNSec()/1000);
    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
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
