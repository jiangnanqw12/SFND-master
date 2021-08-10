// PCL lib Functions for processing point clouds

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include "kdtree.h"
#include "render/box.h"
#include "render/render.h"
#include <chrono>
#include <ctime>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <string>
#include <vector>
#define Clustertest 0
template <typename PointT>
class ProcessPointClouds
{
public:
    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> RansacPlane_student(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering_euclideanCluster(
        typename pcl::PointCloud<PointT>::Ptr cloud,
        float clusterTolerance, int minSize, int maxSize, pcl::visualization::PCLVisualizer::Ptr &viewer);
    void euclideanClusterHelper_student(typename pcl::PointCloud<PointT>::Ptr cloud,
                                        std::vector<int> &cluster, std::vector<bool> &flag_process,
                                        int id, typename KdTree_euclidean<PointT>::KdTree_euclidean *tree, float distanceTol);

    void euclideanClusterHelper_solution(int indice, const std::vector<std::vector<float>> points,
                                         std::vector<int> &cluster, std::vector<bool> &processed,
                                         typename KdTree_euclidean<PointT>::KdTree_euclidean *tree, float distanceTol);
    void render2DTree(Node<PointT> *node, pcl::visualization::PCLVisualizer::Ptr &viewer, Box window, int &iteration, uint depth = 0);
    void test(typename pcl::PointCloud<PointT>::Ptr cloud, typename KdTree_euclidean<PointT>::KdTree_euclidean *tree);
    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
};
#endif /* PROCESSPOINTCLOUDS_H_ */