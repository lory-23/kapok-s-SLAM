#ifndef __DynaRadar
#define __DynaRadar

#include <Eigen/Core>

#define PCL_NO_PRECOMPILE

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/extract_indices.h>

#include <iostream>
#include <cstdlib>
#include <string>
#include <boost/foreach.hpp>
#include <boost/serialization/string.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include<opencv2/core/eigen.hpp>



#include"Frame.h"
using namespace std;

/***************定义radar点云*******************/
struct Point4D_radar
{
	PCL_ADD_POINT4D;                  // 添加pcl里xyz+padding
	float doppler = 0.0;
	float range = 0;
	float power = 0;
	float alpha = 0;
	float beta = 0;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // 确保定义新类型点云内存与SSE对齐
} EIGEN_ALIGN16;                    // 强制SSE填充以正确对齐内存

POINT_CLOUD_REGISTER_POINT_STRUCT( // 注册点类型宏
    Point4D_radar,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, doppler,Doppler )   //-17.3m/s to  17.3m/s 
    (float, range, Range)
    (float, power, Power)
    (float, alpha, Alpha)
    (float, beta, Beta)
    );   

namespace DynaRadar
{

class DynaRadar
{
private:
    string strSettingsFile ;                   //配置文件路径
//RadarSetingFile
    cv::Mat R_cam_mat = cv::Mat(4,4,CV_64FC1,cv::Scalar::all(0)) ;
    cv::Mat P_cam_mat = cv::Mat(4,4,CV_64FC1,cv::Scalar::all(0)) ;
    cv::Mat extrinsicT_mat = cv::Mat(4,4,CV_64FC1,cv::Scalar::all(0)) ;//外参
    float mSegDistanceThreshold;                 //毫米波点云分割阈值
    
    //3x3 rectifying rotation to make image planes co-planar, R_rect_0X:3x3
    Eigen::Matrix<double,4,4> R_rect_02;
    //3x4 projection matrix after rectification, P_rect_02:3x4						
    Eigen::Matrix<double,3,4>  P_rect_02;
    Eigen::Matrix<double,4,4> extrinsicT;

    
    cv::Mat mMask = cv::Mat::ones(480,640,CV_8UC3);               //动态目标musk
    float mMaskRadius;                                                                           
    
    
    cv::Mat mCurrentImage;
    cv::Mat mCurrentImageGray;
    double mCurrentImageTimestamp=0;
    double mCurrentRadarTimestamp=0;

    pcl::PointCloud<Point4D_radar> ::Ptr mRadarCloud4DPtr;
    pcl::PointCloud<Point4D_radar> ::Ptr mMoveRadarCloud4DPtr;

    vector<cv::Point2f> mvMoveRadarPoint2f;                       //运动分割后雷达点
    vector<float> mvMoveRadarPointsfDepth;                       //运动点深度（距离）
    vector<vector<cv::Point2f> > mvvMoveRadarPoint2f; //运动分割后动态雷达点              已聚类
    vector<vector<float> > mvvMoveRadarPointsfDepth; //运动分割后动态雷达点深度      已聚类
    vector<vector<float> > mvvMoveRadarPointsfDoppler; //运动分割后动态雷达点速度  已聚类

    // vector<float> mvRadarPointfDoppler;                                 //雷达点多普勒速度
    // vector<float> mvRadarPointfDepth;                                     //雷达点2d点深度
    pcl::PointCloud<pcl::PointXYZI>::Ptr mRadarCloudXYZIPtr ;     //雷达投影点云(u,v,depth,dopplar*100)
    pcl::PointCloud<pcl::PointXYZI>::Ptr mMoveCloudXYZIPtr ;        //雷达投影点云(u,v,depth,dopplar*100)
    std::vector<pcl::PointIndices> mMoveClusterIndices;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;                               //PCL可视化窗口
    // pcl::visualization::PCLVisualizer::Ptr viewerMovePtr;                   ////PCL运动点可视化窗口

    std::vector<cv::KeyPoint> mLastKeyPointsUn,mTrackedKeyPointsUn;   //SLAM中当前帧与参考帧关键点（为使用）
    float mDelaunaySegDistanceThreshold;                                  //Delaunay 动态点距离判定阈值
    float mDelaunaySegDistanceMin;                                  //Delaunay 动态点距离判定阈值
    ORB_SLAM3::Frame mCurrentFrame;
    //FLAGS
    bool mbPoint3dViewer;
    bool mbDelaunayViewer;
    bool mbFilterViewer;
public:
    

    DynaRadar(bool bFilterViewer = false,bool bDelaunayViewer = false,bool bPointViewer=false);
    ~DynaRadar();
    void updateSetting();

    void saveImg(cv::Mat img ,const double &timestamp);
    Eigen::Vector3d transformProject(const Eigen::Vector4d& P_lidar);    
    pcl::PointCloud < pcl::PointXYZI > ::Ptr Project(pcl::PointCloud<Point4D_radar> ::Ptr radarCloud4DPtr);

    // cv::Mat drawDoppler(cv::Mat img);
    cv::Mat drawVPoint2f(cv::Mat img,vector<cv::Point2f> vCVpoint,const cv::Scalar& color);
    
    void HandelPointCloud(pcl::PCLPointCloud2 inuptPclCloud2);
    void showPoint3d();
    void DBSCAN_MovePoints3d();
    void segMovePoints();
    void projectClusters2D();
    void showClusters2D();
    vector<cv::Point2f> getvMoveRadarPoint2f();

    
    void filterCurrentFrame(ORB_SLAM3::Frame& currentFrame,ORB_SLAM3::KeyFrame* pReferenceKF);
    
    cv::Mat calculateDelaunay(cv::Mat& img,const std::vector<cv::KeyPoint> &mvKeysUn);
    
    cv::Mat getMask();
    vector<cv::RotatedRect> getMaskBoxs();
    
};






}






#endif