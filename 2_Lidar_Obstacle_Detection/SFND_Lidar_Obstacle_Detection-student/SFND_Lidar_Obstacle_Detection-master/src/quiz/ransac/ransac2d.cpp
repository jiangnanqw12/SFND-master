/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../processPointClouds.h"
#include "../../render/render.h"
#include <unordered_set>
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Add inliers
    float scatter = 0.6;
    for (int i = -5; i < 5; i++)
    {
        double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = i + scatter * rx;
        point.y = i + scatter * ry;
        point.z = 0;

        cloud->points.push_back(point);
    }
    // Add outliers
    int numOutliers = 10;
    while (numOutliers--)
    {
        double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = 5 * rx;
        point.y = 5 * ry;
        point.z = 0;

        cloud->points.push_back(point);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    return pointProcessor.loadPcd("../../../../../../SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/simpleHighway.pcd");
    //return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem(1.0);
    return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    for (int i = 0; i < maxIterations; i++)
    {
        std::unordered_set<int> inliers;
        while (inliers.size() < 2)
        {
            inliers.insert(rand() % (cloud->points.size()));
        }
        auto itr = inliers.begin();
        //auto point1 = cloud->points[*itr];
        float x1, y1, x2, y2;
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        float a, b, c;
        a = y1 - y2;
        b = x2 - x1;
        c = x1 * y2 - x2 * y1;
        for (int index = 0; index < cloud->points.size(); index++)
        {
            if (inliers.count(index) > 0)
            {
                continue;
            }
            float x3 = cloud->points[index].x;
            float y3 = cloud->points[index].y;
            float distance = fabs(a * x3 + b * y3 + c) / sqrt(a * a + b * b);
            if (distance <= distanceTol)
            {
                inliers.insert(index);
            }
        }
        if (inliers.size() > inliersResult.size())
        {
            inliersResult = inliers;
        }
    }

    // TODO: Fill in this function

    // For max iterations

    // Randomly sample subset and fit line

    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier

    // Return indicies of inliers from fitted line with most inliers

    return inliersResult;
}
std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    while (maxIterations--)
    {
        std::unordered_set<int> inliers;
        while (inliers.size() < 3)
        {
            inliers.insert(rand() % (cloud->points.size()));
            std::cout << cloud->points.size() << std::endl;
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
    return inliersResult;
}
int main()
{

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    // Create data
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

    // TODO: Change the max iteration and distance tolerance arguments for Ransac function
    //std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);
    std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.5);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    for (int index = 0; index < cloud->points.size(); index++)
    {
        pcl::PointXYZ point = cloud->points[index];
        if (inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    // Render 2D point cloud with inliers and outliers
    if (inliers.size())
    {
        renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
        renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
    }
    else
    {
        renderPointCloud(viewer, cloud, "data");
    }

    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
}