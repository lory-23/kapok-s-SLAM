
#include "DBSCAN.h"
#include "DynaRadar.h"
#include "ORBmatcher.h"
#include "MapPoint.h"
#include <opencv2/optflow.hpp>
#include <numeric>
using namespace std;
using namespace cv;
namespace DynaRadar
{

    void DynaRadar::updateSetting()
    {
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
            cout <<"read matrix from file"<<endl;
        if(!fsSettings.isOpened())
            {
                cerr << "Failed to open settings file at: " << strSettingsFile << endl;
                exit(-1);
            }

            fsSettings["R_cam_mat"]>>R_cam_mat;
            fsSettings["P_cam_mat"]>>P_cam_mat;
            fsSettings["extrinsicT_mat"]>>extrinsicT_mat;
            fsSettings["mSegDistanceThreshold"]>>mSegDistanceThreshold;             //雷达多普勒速度动态点判定阈值

            fsSettings["mDelaunaySegDistanceThreshold"]>>mDelaunaySegDistanceThreshold; //Delaunay 动态点距离判定阈值
            fsSettings["mDelaunaySegDistanceMin"]>>mDelaunaySegDistanceMin;
            fsSettings["mMaskRadius"]>>mMaskRadius;
            cv::cv2eigen(R_cam_mat, R_rect_02);	
            cv::cv2eigen(P_cam_mat, P_rect_02);
            cv::cv2eigen(extrinsicT_mat, extrinsicT);

            cout<<"R:"<<endl<<R_rect_02<<endl;
            cout<<"P:"<<endl<<P_rect_02<<endl;
            cout<<"T:"<<endl<<extrinsicT<<endl;
            cout<<"mSegDistanceThreshold:  "<<mSegDistanceThreshold<<endl;
            cout<<"mMaskRadius:  "<<mMaskRadius<<endl;

            cout<<"mDelaunaySegDistanceThreshold:  "<<mDelaunaySegDistanceThreshold<<endl;      //Delaunay 动态点距离判定阈值
            cout<<"mDelaunaySegDistanceMin:  "<<mDelaunaySegDistanceMin<<endl; 
            
    }

    DynaRadar::~DynaRadar()
    {

    }

    DynaRadar::DynaRadar(bool bFilterViewer,bool bDelaunayViewer,bool bPointViewer):mbFilterViewer(bFilterViewer),mbDelaunayViewer(bDelaunayViewer),mbPoint3dViewer(bPointViewer)
    {
        strSettingsFile = "setting/DynaRadar.yaml";
        mRadarCloudXYZIPtr.reset(new pcl::PointCloud<pcl::PointXYZI>());
        mMoveCloudXYZIPtr.reset(new pcl::PointCloud<pcl::PointXYZI>());
        
        if(mbFilterViewer == true)
        {
            cv::namedWindow("Filter", WINDOW_AUTOSIZE);
        }
        if(mbDelaunayViewer == true)
        {
            cv::namedWindow("Delaunay", WINDOW_AUTOSIZE);
        }
        if(mbPoint3dViewer==true)
        {
            viewerPtr.reset(new pcl::visualization::PCLVisualizer ("cloud"));
            // viewerMovePtr.reset(new pcl::visualization::PCLVisualizer ("movepoints"));
        }
        updateSetting();
    }
    
    void DynaRadar::saveImg(cv::Mat img,const double &timestamp)
    {
        mCurrentImage = img;
        mCurrentImageTimestamp = timestamp;

        cv::Mat CurrentImageGray(mCurrentImage.size.p[0],mCurrentImage.size.p[1],CV_8UC3,Scalar(0,0,0));
        if(mCurrentImage.channels()==3)
        {
                cvtColor(mCurrentImage,CurrentImageGray,cv::COLOR_BGR2GRAY);
        }
        else if(mCurrentImage.channels()==4)
        {
                cvtColor(mCurrentImage,CurrentImageGray,cv::COLOR_BGRA2GRAY);
        }
        mCurrentImageGray = CurrentImageGray;
    }
    
    void DynaRadar::HandelPointCloud(pcl::PCLPointCloud2 inuptPclCloud2)
    {
        mCurrentRadarTimestamp = inuptPclCloud2.header.stamp;
        pcl::PointCloud<Point4D_radar> ::Ptr radarCloud4DPtr(new pcl::PointCloud<Point4D_radar>);
        
        pcl::fromPCLPointCloud2(inuptPclCloud2, *radarCloud4DPtr);
        mRadarCloud4DPtr = radarCloud4DPtr;   //存到类内指针

        segMovePoints();     //分割出mMoveRadarCloud4DPtr
        DBSCAN_MovePoints3d();       //对mMoveRadarCloud4DPtr 密度聚类，  得到 mMoveClusterIndices  
        cout<<"ClusterNum: "<<mMoveClusterIndices.size()<<endl;
        if(mbPoint3dViewer==true)   
            showPoint3d( );   //没修改坐标顺序就显示了，颜色表示不同动态聚类

        // mRadarCloudXYZIPtr =  Project(mRadarCloud4DPtr);    //4d雷达点投影思维的2d点云(u,v,depth,dopplar*100)，用于显示 
        mMoveCloudXYZIPtr = Project(mMoveRadarCloud4DPtr);
        
        projectClusters2D();       //将动态点云转成vv point2f
        showClusters2D();          //在mCurrentImg上画vv point2f

    }

    pcl::PointCloud < pcl::PointXYZI >::Ptr DynaRadar::Project(pcl::PointCloud<Point4D_radar> ::Ptr radarCloud4DPtr)
    {
        int pointcloudsize=0;
        pointcloudsize = radarCloud4DPtr->size();      

        pcl::PointCloud < pcl::PointXYZI > ::Ptr _radarCloudXYZIPtr(new pcl::PointCloud<pcl::PointXYZI>);  //雷达投影点云(u,v,depth,dopplar*100)

        for(int i = 0; i < pointcloudsize; i++)
        {
            pcl::PointXYZI radarPointXYZI ;
            float point_doppler;
            float point_depth;
            Eigen::Vector4d P_lidar(-radarCloud4DPtr->points[i].y,
                                                                                                    -radarCloud4DPtr->points[i].z,
                                                                                radarCloud4DPtr->points[i].x,
                                                                            1);
            Eigen::Vector3d P_uv = transformProject(P_lidar);
            
            //去除不在图像上的投影点,并把点转为cv::Point2f类型
            // if(P_uv[0] >= 0 && P_uv[1] >= 0 && P_uv[0]<640 && P_uv[1]<480)
            {	
                radarPointXYZI.x = P_uv[0]; 
                radarPointXYZI.y =  P_uv[1];
                radarPointXYZI.z = P_lidar[2];;
                radarPointXYZI.intensity =point_doppler*100 ;
                _radarCloudXYZIPtr->points.push_back(radarPointXYZI);
            }
            cout<<"-----------yuanqi-------------"<<P_lidar<<endl;	
            cout<<"----------yuanqi----------"<<P_uv<<endl;                                                
        } 
              	
        return _radarCloudXYZIPtr;
        
    }

    Eigen::Vector3d DynaRadar::transformProject(const Eigen::Vector4d& P_lidar)
    {	Eigen::Vector3d z_P_uv = P_rect_02*R_rect_02*extrinsicT*P_lidar;
        return Eigen::Vector3d( int( z_P_uv[0]/z_P_uv[2] + 0.5 ) , int( z_P_uv[1]/z_P_uv[2] + 0.5 ), 1 );
    }

    // cv::Mat DynaRadar::drawDoppler(cv::Mat img)
    // {
    //     int i ;
    //    int point2f_num = mvRadarPoint2f.size();
    //     cv::Mat out_img = img.clone();
    //     rotate(out_img, out_img, ROTATE_180); //
    //     for(i=0;i<point2f_num;i++)
    //     {
    //         if (mvRadarPointfDoppler[i] > 0)
    //                 circle(out_img,mvRadarPoint2f[i],3,Scalar(255,0,0));
    //             else if (mvRadarPointfDoppler[i] < 0)
    //                 circle(out_img,mvRadarPoint2f[i],3,Scalar(0,0,255));
    //             else
    //                 circle(out_img,mvRadarPoint2f[i],3,Scalar(0,255,0));
    //     }
    //     return out_img;
    // }
    
    cv::Mat DynaRadar::drawVPoint2f(cv::Mat img,vector<cv::Point2f> vCVpoint,const cv::Scalar& color)
    {
        int i ;
        cv::Mat out_img = img.clone();
        // rotate(out_img, out_img, ROTATE_180); //
        for(i=0;i<vCVpoint.size();i++)
            circle(out_img,vCVpoint[i],2,color);
        return out_img;
    }

    cv::Mat drawVPoint2fColorful(cv::Mat img,vector<cv::Point2f> vCVpoint,const vector<float> depth)
    {
        int i ;
        cv::Scalar  color(255,255,255) ;
        cv::Mat out_img = img.clone();
        cv::cvtColor(out_img,out_img,CV_BGR2HSV);

        for(i=0;i<vCVpoint.size();i++)
        {
            color = Scalar(depth[i]*10,255,255);
            circle(out_img,vCVpoint[i],3,color);
        }

        cv::cvtColor(out_img,out_img,CV_HSV2BGR);
        return out_img;
    }

    void DynaRadar::segMovePoints()    //分割运动点将运动点并聚类成目标
    {
        pcl::PointCloud<Point4D_radar> ::Ptr radarCloudpclPtr(new pcl::PointCloud<Point4D_radar>);
        radarCloudpclPtr->points.resize(mRadarCloud4DPtr->size());
        for(int i=0; i<radarCloudpclPtr->size();i++)
        {
            radarCloudpclPtr->points[i].x = mRadarCloud4DPtr->points[i].x;
            radarCloudpclPtr->points[i].y = mRadarCloud4DPtr->points[i].y;
            radarCloudpclPtr->points[i].z = mRadarCloud4DPtr->points[i].doppler;
        }
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);//创建模型系数对象指针用于输出分割后的平面模型系数
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);   //创建内层点云索引指针
        pcl::SACSegmentation<Point4D_radar> seg;   //创建点云分割器
        seg.setOptimizeCoefficients( true);                  //设置点云分割优化参数：true
        seg.setModelType(pcl::SACMODEL_PLANE); //设置模型类型：平面
        seg.setMaxIterations(100);//最大迭代次数为100
        seg.setMethodType(pcl::SAC_RANSAC);         //设置筛选方法类型：RANSAC
        seg.setDistanceThreshold(mSegDistanceThreshold);                      //外点阈值
        seg.setInputCloud(radarCloudpclPtr);           //向分割器中填入点云
        seg.segment(*inliers, *coefficients);                //开始分割（输出为指向内点的指针，分割出的平面模型系数）
        // cout<<"inliersNum: "<<(*inliers).indices.size()<<endl;
	    pcl::PointCloud<Point4D_radar>::Ptr outlierCloud4D(new pcl::PointCloud<Point4D_radar>);
        pcl::ExtractIndices<Point4D_radar> extract;
        extract.setInputCloud (mRadarCloud4DPtr);
        extract.setIndices (inliers);
        extract.setNegative (true);
        extract.filter (*outlierCloud4D);
        mMoveRadarCloud4DPtr = outlierCloud4D;


        
    }

    void DynaRadar::DBSCAN_MovePoints3d()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr MoveCloudXYZPtr (new pcl::PointCloud<pcl::PointXYZ> ()); 
        MoveCloudXYZPtr->points.resize(mMoveRadarCloud4DPtr->size());
        for (size_t i = 0; i < MoveCloudXYZPtr->points.size(); i++) 
        {
            MoveCloudXYZPtr->points[i].x = mMoveRadarCloud4DPtr->points[i].x;    //XYZI（u,v,,z.speed）
            MoveCloudXYZPtr->points[i].y = mMoveRadarCloud4DPtr->points[i].y;
            MoveCloudXYZPtr->points[i].z = mMoveRadarCloud4DPtr->points[i].z;
        }
        
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(MoveCloudXYZPtr);
        std::vector<pcl::PointIndices> cluster_indices;
        DBSCANSimpleCluster<pcl::PointXYZ> ec;
        ec.setCorePointMinPts(1);  //核心点领域内最少点数
        ec.setClusterTolerance(2);  //半径
        ec.setMinClusterSize(2);     //最小聚类点
        ec.setMaxClusterSize(25000);   //最大聚类点
        ec.setSearchMethod(tree);
        ec.setInputCloud(MoveCloudXYZPtr);
        ec.extract(cluster_indices);
        mMoveClusterIndices = cluster_indices;
    }

    void DynaRadar::showPoint3d( )
    {
        viewerPtr->setBackgroundColor(0, 0, 0);
        viewerPtr->removePointCloud("sample");
        viewerPtr->removePointCloud("movePoint");

        pcl::PointCloud<pcl::PointXYZI>::Ptr MoveClusterClouds (new pcl::PointCloud<pcl::PointXYZI> ()); 
        MoveClusterClouds->points.resize(mMoveRadarCloud4DPtr->size());
        for(int i =0; i<mMoveClusterIndices.size();i++)             //每个运动点聚类
        {
            pcl::PointIndices MoveClusterIndice=mMoveClusterIndices[i];
            for(int j=0;j<MoveClusterIndice.indices.size();j++)   //每个聚类里的每个点
            {
                MoveClusterClouds->points[MoveClusterIndice.indices[j]]. x= mMoveRadarCloud4DPtr->points[MoveClusterIndice.indices[j]].x;    //XYZI（u,v,,z.speed）
                MoveClusterClouds->points[MoveClusterIndice.indices[j]].y = mMoveRadarCloud4DPtr->points[MoveClusterIndice.indices[j]].y;
                MoveClusterClouds->points[MoveClusterIndice.indices[j]].z = mMoveRadarCloud4DPtr->points[MoveClusterIndice.indices[j]].z;
                MoveClusterClouds->points[MoveClusterIndice.indices[j]].intensity =  i*10;      
            }
        }        
        
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor(MoveClusterClouds, "intensity");
        viewerPtr->addPointCloud<pcl::PointXYZI>(MoveClusterClouds, fildColor, "movePoint");
        viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6,"movePoint"); // 添加运动点

        viewerPtr->addPointCloud<Point4D_radar>(mRadarCloud4DPtr, "sample",0);
        viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample"); //  添加静态点
        viewerPtr->addCoordinateSystem (1.0);// 显示坐标轴，并设置比例
        viewerPtr->spinOnce(1);
    }

    void DynaRadar::projectClusters2D()
    {
        vector<vector<cv::Point2f> > vvMoveRadarPoint2f; //运动分割后动态雷达点              已聚类
        vector<vector<float> > vvMoveRadarPointsfDepth; //运动分割后动态雷达点深度      已聚类
        vector<vector<float> > vvMoveRadarPointsfDoppler; //运动分割后动态雷达点速度  已聚类
        pcl::PointCloud<pcl::PointXYZI>::Ptr MoveClusterPointsPtr (new pcl::PointCloud<pcl::PointXYZI> ()); 
         
        for(int i =0; i<mMoveClusterIndices.size();i++)             //每个聚类
        {
            pcl::copyPointCloud(*mMoveCloudXYZIPtr, mMoveClusterIndices[i], *MoveClusterPointsPtr); //MoveClusterPointsPtr是每个聚类(u,v,depth,dopplar*100)
            int movepointssize=MoveClusterPointsPtr->size();
            cerr<<"pointNum in cluter"<<i<<": "<<movepointssize<<endl;
            cv::Point2f cv_point;
            float cv_point_depth,cv_point_doppler;
            vector<cv::Point2f> _vMoveRadarPoints2d;
            vector<float> _vMoveRadarPointsfDepth;
            vector<float> _vMoveRadarPointsfDoppler;
            for(int j=0;j<movepointssize;j++)         //聚类中每个点
            {
                cv_point.x = (*MoveClusterPointsPtr)[j].x;
                cv_point.y = (*MoveClusterPointsPtr)[j].y;
                cv_point_depth = (*MoveClusterPointsPtr)[j].z;
                cv_point_doppler = (*MoveClusterPointsPtr)[j].intensity;
                _vMoveRadarPoints2d.push_back(cv_point);
                _vMoveRadarPointsfDepth.push_back(cv_point_depth);
                _vMoveRadarPointsfDoppler.push_back(cv_point_doppler);
            }
            
            vvMoveRadarPoint2f.push_back(_vMoveRadarPoints2d);
            vvMoveRadarPointsfDepth.push_back(_vMoveRadarPointsfDepth);
            vvMoveRadarPointsfDoppler.push_back(_vMoveRadarPointsfDoppler);
        }
        mvvMoveRadarPoint2f = vvMoveRadarPoint2f;
        mvvMoveRadarPointsfDepth = vvMoveRadarPointsfDepth;
        mvvMoveRadarPointsfDoppler = vvMoveRadarPointsfDoppler;
    }

    void DynaRadar::showClusters2D()
    {
        RNG rng(12345);
        int cluterNum = mvvMoveRadarPoint2f.size();
        for(int i=0;i<cluterNum;i++)
        {   
            int r = rng.uniform(0, 255);
            int g = rng.uniform(0, 255);
            int b = rng.uniform(0, 255);
            mCurrentImageGray = drawVPoint2f(mCurrentImageGray,mvvMoveRadarPoint2f[i],cv::Scalar(b, g, r));
           
        }
        
    }

    vector<cv::Point2f> DynaRadar::getvMoveRadarPoint2f()
    {
        return mvMoveRadarPoint2f;
    }

    // static void draw_subdiv( Mat& img, Subdiv2D& subdiv, Scalar delaunay_color )
    // {
    //     vector<Vec6f> triangleList;
    //     subdiv.getTriangleList(triangleList);
    //     vector<Point> pt(3);
    //     for( size_t i = 0; i < triangleList.size(); i++ )
    //     {
    //         Vec6f t = triangleList[i];
    //         pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
    //         pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
    //         pt[2] = Point(cvRound(t[4]), cvRound(t[5]));
    //         line(img, pt[0], pt[1], delaunay_color, 1, LINE_AA, 0);
    //         line(img, pt[1], pt[2], delaunay_color, 1, LINE_AA, 0);
    //         line(img, pt[2], pt[0], delaunay_color, 1, LINE_AA, 0);
    //     }
    // }
    // static void draw_subdiv_point( Mat& img, Point2f fp, Scalar color )
    // {
    //     circle( img, fp, 3, color, FILLED, LINE_8, 0 );
    // }
    // static void locate_point( Mat& img, Subdiv2D& subdiv, Point2f fp, Scalar active_color )
    // {
    //     int e0=0, vertex=0;
    //     subdiv.locate(fp, e0, vertex);
    //     if( e0 > 0 )
    //     {
    //         int e = e0;
    //         do
    //         {
    //             Point2f org, dst;

    //             e = subdiv.getEdge(e, Subdiv2D::NEXT_AROUND_LEFT);
    //         }
    //         while( e != e0 );
    //     }
    //     draw_subdiv_point( img, fp, active_color );
    // }
    
   
    // cv::Mat DynaRadar::calculateDelaunay(cv::Mat& img,const std::vector<cv::KeyPoint> &mvKeysUn)
    // {
        
    //     double t = (double)getTickCount();

    //     int num_KeyPoints = mvKeysUn.size();;
    //     Size img_size = img.size();
    
    //     Rect rect(0, 0, img_size.width, img_size.height);
    //     Subdiv2D subdiv(rect);
    
    //     for( int i =0;i<num_KeyPoints;i++)
    //     {
    //         Point2f fp  = mvKeysUn[i].pt;
    //         locate_point( img, subdiv, fp, (0, 0, 255) );
    //         subdiv.insert(fp);
    //     }

    //     draw_subdiv( img, subdiv, (255,255,255) );

    //     if(mbDelaunayViewer == true)
    //     {   
    //         cout<<"DelaunayPoint : "<<num_KeyPoints<<endl;
    //         cout<<"img_size :    "<<img_size<<endl;
    //         imshow( "Delaunay", img );    
    //         t = ((double)getTickCount() - t)/getTickFrequency();
    //         cout <<num_KeyPoints<<" points. "<< "执行时间(秒): " << t << endl;
    //     }

    //     return img;
    // }

    // float getDistance(cv::Point2f a,cv::Point2f b)
    // {
    //     float distance = powf((a.x-b.x),2)+powf((a.y-b.y),2);
    //     return distance;
    // }

    // #define DerictDel
    void DynaRadar::filterCurrentFrame(ORB_SLAM3::Frame& currentFrame,ORB_SLAM3::KeyFrame* pReferenceKF)   //没啥用了
    {
        mMask  = cv::Mat::ones(480,640,CV_8UC3);
        cv::Mat Mask =cv::Mat::ones(480,640,CV_8U);
        cv::Mat DelCF = mCurrentImage.clone();
        // cv::Mat DelKF(480,640,CV_8UC3,Scalar(0,0,0));
     
        int MoveRadarPointSize = mvMoveRadarPoint2f.size();
        cout<<"MoveRadarPoints num: "<<MoveRadarPointSize<<endl;
        if(mbFilterViewer==true)
            DelCF = drawVPoint2f(DelCF,mvMoveRadarPoint2f,Scalar(255,255,0));    //雷达判定的运动点

        #ifndef DerictDel     //直接删除

            if(MoveRadarPointSize<100&&MoveRadarPointSize>0)
            {
                for(int i=0;i<MoveRadarPointSize;i++)
                {
                    // cout<<"moveRadarPoint "<<i<<"   depth:"<<mvMoveRadarPointsfDepth[i] <<endl;
                    cv::circle(Mask,mvMoveRadarPoint2f[i],150/mvMoveRadarPointsfDepth[i]  ,Scalar(0),-1);   
                }

                std::vector<cv::KeyPoint> _mvKeys;
                cv::Mat _mDescriptors;
                for (size_t i(0); i < currentFrame.mvKeys.size(); ++i)
                {
                    int val = (int)Mask.at<uchar>(currentFrame.mvKeys[i].pt.y,currentFrame.mvKeys[i].pt.x);
                    if (val == 0)
                        {
                            // _mvKeys.push_back(currentFrame.mvKeys[i]);
                            // _mDescriptors.push_back(currentFrame.mDescriptors.row(i));
                        }

                }
                
                // currentFrame.mvKeys = _mvKeys;
                // currentFrame.mDescriptors = _mDescriptors;
                // currentFrame.N = currentFrame.mvKeys.size();
            
            }
            else
                cout<<"NO RadarPoint now !!"<<endl;
            
            
            
            if(mbFilterViewer==true)
            {
                cv::hconcat(DelCF,mMask,DelCF);
                imshow( "Filter", DelCF );
            }









        #else                 //特征点法

            ///////////////////////////////当前帧与参考帧关键点匹配//////////////////////////////////////////

            vector<cv::KeyPoint> vKFpoints = pReferenceKF->mvKeysUn;
            vector<cv::KeyPoint>  vCFpoints = currentFrame.mvKeysUn;
            vector<int> matchedKFpointsId ;
            vector<int> matchedCFpointsId ;
            vector<cv::KeyPoint> vMatchedKFpoints;
            vector<cv::KeyPoint> vMatchedCFpoints;
            vector<cv::Point2f> vMatchedKFpoints2f;
            vector<cv::Point2f> vMatchedCFpoints2f;
            int match_num;
            vector<int> vMatches;  //索引为当前帧point序号，值为参考振point序号        
            
            currentFrame.ComputeBoW();

            ORB_SLAM3::ORBmatcher matcher(0.5,true);
            match_num = matcher.MatchByBoW(pReferenceKF,currentFrame, vMatches);

            // ORB_SLAM3::ORBmatcher matcher(0.8,true);
            // match_num = matcher.MatchByWindow(*pReferenceKF,currentFrame,vMatches,300);
            // for(int i = 0; i<vMatches.size();i++)
            // {
            //     cout<<"vMatches: "<<vMatches[i]<<endl;
            // }
            cout<<"match_num = "<<match_num<<endl;
            
            for( int i =0 , j =0 ;i<vMatches.size();i++)               //筛除未匹配的id对
            {
                if(vMatches[i] != -1)
                {
                    matchedCFpointsId.push_back(i);
                    matchedKFpointsId.push_back(vMatches[i]);
                }
            }
            match_num = matchedCFpointsId.size();
            cout<<"after match_num = "<<match_num<<endl;
            for(int i = 0;i<match_num;i++)                     //筛除未匹配的keypoint
            {
                vMatchedKFpoints2f.push_back( vKFpoints[matchedKFpointsId[i]].pt);
                vMatchedCFpoints2f.push_back( vCFpoints[matchedCFpointsId[i]].pt);
            }

            ////////////////////////////////☝当前帧与参考帧关键点匹配☝///////////////////////////////////////

            cv::Mat DelCF = mCurrentImage.clone();
            cv::Mat DelKF(480,640,CV_8UC3,Scalar(0,0,0));
            // DelKF = calculateDelaunay(DelKF,vMatchedKFpoints);
            // cv::Mat DelCF(480,640,CV_8UC3,Scalar(0,0,0));
            // DelcF = calculateDelaunay(DelcF,vMatchedCFpoints);
            cout<<"MoveRadarPoints num: "<<mvMoveRadarPoint2f.size()<<endl;
            DelCF = drawVPoint2f(DelCF,mvMoveRadarPoint2f,Scalar(255,255,0));    //雷达判定的运动点
            /////////////////////////////👇三角剖分并获取边👇///////////////////////////////
            Rect rect(0, 0, 640, 480);
            Subdiv2D subdivCF(rect);
            // Subdiv2D subdivKF(rect);

            subdivCF.insert(vMatchedCFpoints2f);    //插入的点索引+4就是vertex ID
            // subdivKF.insert(vMatchedKFpoints2f);

            std::vector< Vec4f >  	edgeListCF,edgeListKF;
            subdivCF.getEdgeList(edgeListCF); 
            // subdivKF.getEdgeList(edgeListKF); 
            int edgeNumCF = edgeListCF.size();
            // int edgeNumKF = edgeListKF.size();
        
            /////////////////////////////////////////////////////////////////////////

            vector<cv::Vec2f> vMoveVec2fCF(match_num,{-1,-1});   //CF中每个特征点的运动向量
            for(int i = 0;i<match_num;i++)                    //检查点匹配效果
            {
                cv::circle(DelCF,vMatchedCFpoints2f[i],2,Scalar(255,0,0));
                // cv::circle(DelCF,vMatchedKFpoints2f[i],3,Scalar(255,0,0));  
                // line(DelCF, vMatchedCFpoints2f[i], vMatchedKFpoints2f[i], Scalar(0,255,0), 1, LINE_AA, 0);               
               
                vMoveVec2fCF[i]= vMatchedCFpoints2f[i]-vMatchedKFpoints2f[i];
                line(DelCF, vMatchedCFpoints2f[i],   vMatchedKFpoints2f[i]  , Scalar(0,255,0), 1, LINE_AA, 0);

            }

            if(false)  //点关联判断
            for( int i =0; i<edgeNumCF;i++)
            {
                vector<Point2f> ptCF(2);
                ptCF[0] = Point2f(edgeListCF[i][0], edgeListCF[i][1]);
                ptCF[1] = Point2f(edgeListCF[i][2], edgeListCF[i][3]);
                if(abs(ptCF[0].x)>1900||abs(ptCF[0].y)>1900)       //排除徳劳内三角形的错误边
                    continue;
                if(abs(ptCF[1].x)>1900||abs(ptCF[1].y)>1900)
                    continue;
                int orgIDCF=-1,dstIDCF=-1;
                for(int j =0;j<match_num;j++)                                //徳劳内边的顶点与原关键点匹配，匹配ID存放于orgIDCF，dstIDCF
                {
                    if (vMatchedCFpoints2f[j]==ptCF[0])
                    orgIDCF = j;
                    if (vMatchedCFpoints2f[j]==ptCF[1])
                    dstIDCF = j;
                }
                // if(orgIDCF==-1||dstIDCF==-1)
                //     cout<<"match filed"<<"target:"<<ptCF[0]<<"     findout: "<<vMatchedCFpoints2f[orgIDCF]<<"orgIDCF,dstIDCF"<<orgIDCF<<"  "<<dstIDCF<<endl;

                //计算CF中边的长度与KF中对应边长
                float distanceCF,distanceKF,deffDistance;
                distanceCF = getDistance(vMatchedCFpoints2f[orgIDCF],vMatchedCFpoints2f[dstIDCF]);
                if(distanceCF<50)   //滤除太短的线段
                    continue;
                distanceKF = getDistance(vMatchedKFpoints2f[orgIDCF],vMatchedKFpoints2f[dstIDCF]);
                deffDistance = abs(distanceCF - distanceKF);
                if(deffDistance>10000)   //滤除误匹配点
                    continue;


                if(deffDistance>distanceCF*mDelaunaySegDistanceThreshold&&deffDistance>mDelaunaySegDistanceMin)
                {
                    cout<<"CF"<<vMatchedCFpoints2f[orgIDCF]<<", "<<vMatchedCFpoints2f[dstIDCF]<<"    distanceCF: "<<distanceCF<<  "     NO: "<<i<<"/"<<edgeNumCF <<endl;
                    cout<<"KF"<<vMatchedKFpoints2f[orgIDCF]<<", "<<vMatchedKFpoints2f[dstIDCF]<<"    distanceKF: "<<distanceKF<<"       deffDistance: "<<deffDistance<<endl;
                    line(DelCF, ptCF[0], ptCF[1], Scalar(0,0,255), 3, LINE_AA, 0);
                    line(DelCF, vMatchedKFpoints2f[orgIDCF], vMatchedKFpoints2f[dstIDCF], Scalar(255,0,0), 1, LINE_AA, 0);
                }
                else
                {            
                    // cout<<"CF"<<vMatchedCFpoints2f[orgIDCF]<<", "<<vMatchedCFpoints2f[dstIDCF]<<"    distanceCF: "<<distanceCF<<  "     NO: "<<i<<"/"<<edgeNumCF <<endl;
                    // cout<<"KF"<<vMatchedKFpoints2f[orgIDCF]<<", "<<vMatchedKFpoints2f[dstIDCF]<<"    distanceKF: "<<distanceKF<<endl;
                    line(DelCF, ptCF[0], ptCF[1], Scalar(0,255,0), 1, LINE_AA, 0);
                }
                    
                line(DelKF, vMatchedKFpoints2f[orgIDCF], vMatchedKFpoints2f[dstIDCF], Scalar(0,255,0), 1, LINE_AA, 0);

                cv::Mat MatchCompare;
                cv::hconcat(DelCF,DelKF,MatchCompare);
                imshow( "Filter", MatchCompare );  
                char keyvel;
                if(keyvel = waitKey(5000))
                {
                    if(keyvel=='n')
                        continue;
                    if(keyvel=='q')
                        break;
                }

            }

            else         //运动速度场判断
            {
                vector<cv::Vec2f> vMovePointVec2fCF;

                for(int i=0;i<mvMoveRadarPoint2f.size();i++)        //对每个雷达运动点提取最近边
                {
                    int returnflag =-1;
                    int nearestVtxId=-1,nearestEdgeId=-1;
                    cv::Point2f dstPt,orgPt;
                    int dstPtId,orgPtId;
                    int dstPtIdCF,orgPtIdCF;

                    returnflag = subdivCF.locate(mvMoveRadarPoint2f[i],nearestEdgeId,nearestVtxId);        //对每个雷达运动点提取最近边的起始点
                    if(nearestEdgeId!=0)
                    {
                        dstPtId = subdivCF.edgeDst(nearestEdgeId,&dstPt);
                        orgPtId = subdivCF.edgeOrg(nearestEdgeId,&orgPt);
                        if(dstPtId<4||orgPtId<4) 
                            continue;

                        dstPtIdCF = dstPtId-4;                                  //CF中对应的id
                        orgPtIdCF = orgPtId-4;
                        cv::circle(DelCF,vMatchedCFpoints2f[dstPtIdCF],6,Scalar(0,255,255));
                        cv::circle(DelCF,vMatchedCFpoints2f[orgPtIdCF],6,Scalar(0,255,255));
                        // line(DelCF,dstPt ,   orgPt  , Scalar(0,255,255), 2, LINE_AA, 0);

                        vMovePointVec2fCF.push_back(vMoveVec2fCF[dstPtIdCF]);       //获取CF中始末点的运动向量
                        vMovePointVec2fCF.push_back(vMoveVec2fCF[orgPtIdCF]);
                    }

                }
                cv::Vec2f sumMoveVec,avgMoveVec;
                for(int i=0; i<vMovePointVec2fCF.size();i++)           //计算当前帧中始末点运动向量 均值
                {
                    sumMoveVec+=vMovePointVec2fCF[i];
                }
                avgMoveVec= sumMoveVec/int(vMovePointVec2fCF.size());

                cout<<"avgMoveVec: "<<avgMoveVec<<endl;
                cv::Vec2f distanceVec;
                float gate = 0.1*norm(avgMoveVec,NORM_L2SQR);
                for(int i=0;i<vMoveVec2fCF.size();i++)                      //遍历CF中特征点运动向量  与始末点均值向量 二范数
                {
                    distanceVec = vMoveVec2fCF[i]-avgMoveVec;
                    float distance = norm(distanceVec,NORM_L2SQR);
                    cout<<"distanceVec : "<<distanceVec<<"    avgMoveVec : "<<avgMoveVec;
                    cout<<"     distance : "<<distance<<"    gate : "<<gate<<endl;
                    if(distance<gate)            //标记并删除CF中与始末均值运动向量相近的向量 
                        cv::circle(DelCF,vMatchedCFpoints2f[i],4,Scalar(0,0,255));  
                              
                }
            }
           

            /////////////////
            // vector<Vec6f> triangleListCF,triangleListKF;
            // subdivCF.getTriangleList(triangleListCF);
            // subdivKF.getTriangleList(triangleListKF);
            // vector<Point> ptCF(3),ptKF(3);
            // cout<<"triangleListCF size"<<triangleListCF.size()<<"  triangleListKF size"<<triangleListKF.size()<<endl;
            // for( size_t i = 0; i < triangleListCF.size(); i++ )
            // {
            //     Vec6f t = triangleListCF[i];
            //     ptCF[0] = Point(cvRound(t[0]), cvRound(t[1]));
            //     ptCF[1] = Point(cvRound(t[2]), cvRound(t[3]));
            //     ptCF[2] = Point(cvRound(t[4]), cvRound(t[5]));
            //     line(DelCF, ptCF[0], ptCF[1], Scalar(0,255,0), 1, LINE_AA, 0);
            //     line(DelCF, ptCF[1], ptCF[2], Scalar(0,255,0), 1, LINE_AA, 0);
            //     line(DelCF, ptCF[2], ptCF[0], Scalar(0,255,0), 1, LINE_AA, 0);
            // }

            // for( size_t i = 0; i < triangleListKF.size(); i++ )
            // {
            //     Vec6f t = triangleListKF[i];
            //     ptKF[0] = Point(cvRound(t[0]), cvRound(t[1]));
            //     ptKF[1] = Point(cvRound(t[2]), cvRound(t[3]));
            //     ptKF[2] = Point(cvRound(t[4]), cvRound(t[5]));
            //     line(DelKF, ptKF[0], ptKF[1], Scalar(0,255,0), 1, LINE_AA, 0);
            //     line(DelKF, ptKF[1], ptKF[2], Scalar(0,255,0), 1, LINE_AA, 0);
            //     line(DelKF, ptKF[2], ptKF[0], Scalar(0,255,0), 1, LINE_AA, 0);
            // }
            //////////////////////////////////////////////////////////////
            // cv::Mat MatchCompare;
            // cv::hconcat(DelCF,DelKF,MatchCompare);
            imshow( "Filter", DelCF );
            // imshow( "Filter", MatchCompare );  


        #endif
    }


    cv::Mat DynaRadar::getMask()
    {
        cv::Mat Mask =cv::Mat(mCurrentImageGray.size.p[0],mCurrentImageGray.size.p[1],CV_8U,255);

        int MoveClutersSize = mvvMoveRadarPoint2f.size();
    
        if(MoveClutersSize<50&&MoveClutersSize>0)
            {
                for(int i =0;i<MoveClutersSize;i++)      //每个聚类
                {
                    float sumDepth = accumulate(begin(mvvMoveRadarPointsfDepth[i]),end(mvvMoveRadarPointsfDepth[i]), 0.0);   // 聚类中每个点深度和；
                    float meanDepth = sumDepth / mvvMoveRadarPointsfDepth[i].size();                   // 深度均值
                    mMaskRadius = mMaskRadius/meanDepth;
                    
                    // if(mvvMoveRadarPoint2f[i].size()>1)
                    {
                        vector<Point2f> vDrawpoint2f;
                        for(auto MoveRadarPoint2f:mvvMoveRadarPoint2f[i])
                        {
                            vDrawpoint2f.push_back(MoveRadarPoint2f);
                            Point2f left(MoveRadarPoint2f.x-mMaskRadius,MoveRadarPoint2f.y);
                            Point2f right(MoveRadarPoint2f.x+mMaskRadius,MoveRadarPoint2f.y);
                            Point2f up(MoveRadarPoint2f.x,MoveRadarPoint2f.y-mMaskRadius);
                            Point2f down(MoveRadarPoint2f.x,MoveRadarPoint2f.y+mMaskRadius);
                            vDrawpoint2f.push_back(left);vDrawpoint2f.push_back(right);vDrawpoint2f.push_back(up);vDrawpoint2f.push_back(down);
                        }
                        Point2f vtx[4];
		                RotatedRect box = minAreaRect(vDrawpoint2f);
		                box.points(vtx);
                        for (i = 0; i < 4; i++)
			            line(mCurrentImageGray, vtx[i], vtx[(i + 1) % 4], Scalar(0, 255, 0), 1, LINE_AA);

                        cv::ellipse(Mask, box.center,Size(box.size.width,box.size.height),box.angle,0, 360,Scalar(0),FILLED) ;
                    }
                }
            }
        if(mbFilterViewer==true)
            imshow("Filter",Mask) ;
        
        return Mask;
    }

    vector<RotatedRect> DynaRadar::getMaskBoxs()
    {
        cv::Mat Mask =cv::Mat(mCurrentImageGray.size.p[0],mCurrentImageGray.size.p[1],CV_8U,255);

        int MoveClutersSize = mvvMoveRadarPoint2f.size();
        vector<RotatedRect> boxs;
        if(MoveClutersSize<50&&MoveClutersSize>0)
            {
                for(int i =0;i<MoveClutersSize;i++)      //每个聚类
                {
                    float sumDepth = accumulate(begin(mvvMoveRadarPointsfDepth[i]),end(mvvMoveRadarPointsfDepth[i]), 0.0);   // 聚类中每个点深度和；
                    float meanDepth = sumDepth / mvvMoveRadarPointsfDepth[i].size();                   // 深度均值
                    mMaskRadius = mMaskRadius/meanDepth;
                    
                    // if(mvvMoveRadarPoint2f[i].size()>1)
                    {
                        vector<Point2f> vDrawpoint2f;
                        for(auto MoveRadarPoint2f:mvvMoveRadarPoint2f[i])
                        {
                            vDrawpoint2f.push_back(MoveRadarPoint2f);
                            Point2f left(MoveRadarPoint2f.x-mMaskRadius,MoveRadarPoint2f.y);
                            Point2f right(MoveRadarPoint2f.x+mMaskRadius,MoveRadarPoint2f.y);
                            Point2f up(MoveRadarPoint2f.x,MoveRadarPoint2f.y-mMaskRadius);
                            Point2f down(MoveRadarPoint2f.x,MoveRadarPoint2f.y+mMaskRadius);
                            vDrawpoint2f.push_back(left);vDrawpoint2f.push_back(right);vDrawpoint2f.push_back(up);vDrawpoint2f.push_back(down);
                        }

		                RotatedRect box = minAreaRect(vDrawpoint2f);
		                boxs.push_back(box);
                          
                        std::cout << "box "<< i <<" center"<<box.center<<std::endl; 

                        cv::ellipse(Mask, box.center,Size(box.size.width,box.size.height),box.angle,0, 360,Scalar(0),FILLED) ;
                    }
                }
            }
        if(mbFilterViewer==true)
            imshow("Filter",mCurrentImageGray) ;
            //imshow("Filter",Mask) ;
        
        return boxs;
    }






}//namespace

