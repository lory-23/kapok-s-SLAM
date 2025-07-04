#ifndef DBSCAN_H
#define DBSCAN_H
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include "DynaRadar.h"
#define UN_PROCESSED 0
#define PROCESSING 1
#define PROCESSED 2

inline bool comparePointClusters (const pcl::PointIndices &a, const pcl::PointIndices &b) {
    return (a.indices.size () < b.indices.size ());
}

template <typename PointT>
class DBSCANSimpleCluster {
public:
    typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef typename pcl::search::KdTree<PointT>::Ptr KdTreePtr;
    
    virtual void setInputCloud(PointCloudPtr cloud) {
        input_cloud_ = cloud;
    }

    void setSearchMethod(KdTreePtr tree) {
        search_method_ = tree;
    }

    void extract(std::vector<pcl::PointIndices>& cluster_indices) {
        std::vector<int> nn_indices;
        std::vector<float> nn_distances;
        std::vector<bool> is_noise(input_cloud_->points.size(), false);
        std::vector<int> types(input_cloud_->points.size(), UN_PROCESSED);
        for (int i = 0; i < input_cloud_->points.size(); i++) {
            if (types[i] == PROCESSED) {
                continue;
            }
            int nn_size = radiusSearch(i, eps_, nn_indices, nn_distances);//如果该点非核心点，则认为是噪声点并忽视
            if (nn_size < minPts_) {
                is_noise[i] = true;
                continue;
            }
            //确定是新核心点
            std::vector<int> seed_queue;//当前类目录
            seed_queue.push_back(i);
            types[i] = PROCESSED;
            for (int j = 0; j < nn_size; j++) { //将领域点加入当前目录并标记为已处理
                if (nn_indices[j] != i) {
                    seed_queue.push_back(nn_indices[j]);
                    types[nn_indices[j]] = PROCESSING;
                }
            } // for every point near the chosen core point.
            int sq_idx = 1;
            while (sq_idx < seed_queue.size()) {   
                int cloud_index = seed_queue[sq_idx];        // 取出当前目录中一点
                if (is_noise[cloud_index] || types[cloud_index] == PROCESSED) {    //如果被处理过就下一个
                    // seed_queue.push_back(cloud_index);
                    types[cloud_index] = PROCESSED;
                    sq_idx++;
                    continue; // no need to check neighbors.
                }
                nn_size = radiusSearch(cloud_index, eps_, nn_indices, nn_distances);  //搜索取出的点领域
                if (nn_size >= minPts_) {
                    for (int j = 0; j < nn_size; j++) {
                        if (types[nn_indices[j]] == UN_PROCESSED) {       //如果当前点领域内没处理过，加入目录，待处理
                            
                            seed_queue.push_back(nn_indices[j]);
                            types[nn_indices[j]] = PROCESSING;
                        }
                    }
                }
                types[cloud_index] = PROCESSED;    //标记为处理（这里点一般为新添加到当前目录的点）
                sq_idx++;
            }
            if (seed_queue.size() >= min_pts_per_cluster_ && seed_queue.size () <= max_pts_per_cluster_) { //聚类大小在范围内
                pcl::PointIndices r;   //复制聚类目录到r
                r.indices.resize(seed_queue.size());
                for (int j = 0; j < seed_queue.size(); ++j) {
                    r.indices[j] = seed_queue[j];
                }
                // These two lines should not be needed: (can anyone confirm?) -FF
                std::sort (r.indices.begin (), r.indices.end ());
                r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());

                r.header = input_cloud_->header;
                cluster_indices.push_back (r);   // We could avoid a copy by working directly in the vector
            }
        } // for every point in input cloud
        std::sort (cluster_indices.rbegin (), cluster_indices.rend (), comparePointClusters);
    }

    void setClusterTolerance(double tolerance) {
        eps_ = tolerance; 
    }

    void setMinClusterSize (int min_cluster_size) { 
        min_pts_per_cluster_ = min_cluster_size; 
    }

    void setMaxClusterSize (int max_cluster_size) { 
        max_pts_per_cluster_ = max_cluster_size; 
    }
    
    void setCorePointMinPts(int core_point_min_pts) {
        minPts_ = core_point_min_pts;
    }

protected:
    PointCloudPtr input_cloud_;
    double eps_ {0.0};     //半径
    int minPts_ {1}; // not including the point itself. 领域内最少点数
    int min_pts_per_cluster_ {1};      //聚类最少点
    int max_pts_per_cluster_ {std::numeric_limits<int>::max()};    //聚类最大点数
    KdTreePtr search_method_;      

    virtual int radiusSearch(
        int index, double radius, std::vector<int> &k_indices,
        std::vector<float> &k_sqr_distances) const
    {
        return this->search_method_->radiusSearch(index, radius, k_indices, k_sqr_distances);
    }
}; // class DBSCANCluster

#endif // DBSCAN_H