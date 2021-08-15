// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include "kdtree.h"
#include "pcl/ModelCoefficients.h"
#include <unordered_set>
//constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(
        new pcl::PointCloud<PointT>());
    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion(
        new pcl::PointCloud<PointT>());
    //typename pcl::PointCloud<PointT>::Ptr cloudRegion;
    pcl::CropBox<PointT>
        region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud);
    region.filter(*cloudRegion);

    std::vector<int> indices;
    //Set to true if you want to be able to extract the indices of points being removed (default = false)
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};

    for (int point : indices)
        inliers->indices.push_back(point);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::
    SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>);
    for (int index : inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[index]);
    }
    // Extract the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);
    //std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::
    SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    // TODO:: Fill in this function to find inliers for the cloud.

    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    // Segment the Largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset" << std::endl;
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::
    RansacPlane_student(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    auto startTime = std::chrono::steady_clock::now();
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    while (maxIterations--)
    {
        std::unordered_set<int> inliers;
        while (inliers.size() < 3)
        {
            inliers.insert(rand() % (cloud->points.size()));
            //std::cout << cloud->points.size() << std::endl;
        }
        float x1, x2, x3, y1, y2, y3, z1, z2, z3;
        auto itr = inliers.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;
        float a, b, c, d, i, j, k;
        i = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        j = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        k = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        a = i;
        b = j;
        c = k;
        d = -(i * x1 + j * y1 + k * z1);
        for (int index = 0; index < cloud->points.size(); index++)
        {
            if (inliers.count(index) > 0)
            {
                continue;
            }
            float x4 = cloud->points[index].x;
            float y4 = cloud->points[index].y;
            float z4 = cloud->points[index].z;
            float dist = fabs(a * x4 + b * y4 + c * z4 + d) / (sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2)));
            if (dist <= distanceTol)
            {
                inliers.insert(index);
            }
        }
        if (inliers.size() > inliersResult.size())
        {
            inliersResult = inliers;
        }
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    typename pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_obj(new pcl::PointCloud<PointT>);
    for (int index = 0; index < cloud->points.size(); index++)
    {
        if (inliersResult.count(index) > 0)
        {
            cloud_plane->points.push_back(cloud->points[index]);
        }
        else
        {
            cloud_obj->points.push_back(cloud->points[index]);
        }
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_obj, cloud_plane);
    return segResult;
}
template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering_euclideanCluster(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    float clusterTolerance, int minSize, int maxSize, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
#if Clustertest
    printf("euclideanCluster>>()");
#endif
    // TODO: Fill out this function to return list of indices for each cluster

    typename KdTree_euclidean<PointT>::KdTree_euclidean *tree = new KdTree_euclidean<PointT>;
    for (int i = 0; i < cloud->points.size(); i++)
        tree->insert((cloud)->points[i], i);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    //std::vector<std::vector<int>> cluster_indices;
    std::vector<bool> flag_process(cloud->points.size(), false);
    for (int i = 0; i < cloud->points.size(); i++)
    {
        if (flag_process[i])
        {
            continue;
        }
        //typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        std::vector<int> cluster;
        euclideanClusterHelper_student(cloud, cluster, flag_process, i, tree, clusterTolerance);
        //euclideanClusterHelper_solution(i, points, cluster, flag_process, tree, clusterTolerance);
        //cluster_indices.push_back(cluster);

        if ((cluster.size() >= minSize) && (cluster.size() <= maxSize))
        {
            typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
            for (int i = 0; i < cluster.size(); i++)
            {

                cloud_cluster->points.push_back(cloud->points[cluster[i]]);

                //cloud_cluster->push_back((*cloud)[idx]);
            }
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            clusters.push_back(cloud_cluster);
        }
    }

#if Clustertest
    printf("euclideanCluster<<()");
#endif
    //render2DTree(tree->root, viewer, window, it);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
template <typename PointT>
void ProcessPointClouds<PointT>::euclideanClusterHelper_student(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                std::vector<int> &cluster, std::vector<bool> &flag_process,
                                                                int id, typename KdTree_euclidean<PointT>::KdTree_euclidean *tree, float distanceTol)
{
#if Clustertest
    printf("helper>>()");
#endif
    flag_process[id] = true;
    cluster.push_back(id);
    std::vector<int> ids = tree->search(cloud->points[id], distanceTol);
    //std::vector<int> ids = tree->search(cloud, id, distanceTol);
    //tree->test2();
    //std::vector<int> ids;
    for (int i = 0; i < ids.size(); i++)
    {
        if (flag_process[ids[i]] == false)
        {

            euclideanClusterHelper_student(cloud, cluster, flag_process, ids[i], tree, distanceTol);
        }
    }
#if Clustertest
    printf("helper<<()");
#endif
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // 2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for (const auto &idx : it->indices)
            cloud_cluster->push_back((*cloud)[idx]); //*
        //cloud_cluster->push_back((*cloud)[idx]);
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
        //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
template <typename PointT>
void ProcessPointClouds<PointT>::test(typename pcl::PointCloud<PointT>::Ptr cloud, typename KdTree_euclidean<PointT>::KdTree_euclidean *tree)
{
    tree->search();
}
template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}
template <typename PointT>
void render2DTree(Node<PointT> *node, pcl::visualization::PCLVisualizer::Ptr &viewer, Box window, int &iteration, uint depth)
{

    if (node != NULL)
    {
        Box upperWindow = window;
        Box lowerWindow = window;
        // split on x axis
        if (depth % 2 == 0)
        {
            viewer->addLine(pcl::PointXYZ(node->point.x, window.y_min, 0), pcl::PointXYZ(node->point.x, window.y_max, 0), 0, 0, 1, "line" + std::to_string(iteration));
            lowerWindow.x_max = node->point.x;
            upperWindow.x_min = node->point.x;
        }
        // split on y axis
        else
        {
            viewer->addLine(pcl::PointXYZ(window.x_min, node->point.y, 0), pcl::PointXYZ(window.x_max, node->point.y, 0), 1, 0, 0, "line" + std::to_string(iteration));
            lowerWindow.y_max = node->point.y;
            upperWindow.y_min = node->point.y;
        }
        iteration++;

        render2DTree(node->left, viewer, lowerWindow, iteration, depth + 1);
        render2DTree(node->right, viewer, upperWindow, iteration, depth + 1);
    }
}