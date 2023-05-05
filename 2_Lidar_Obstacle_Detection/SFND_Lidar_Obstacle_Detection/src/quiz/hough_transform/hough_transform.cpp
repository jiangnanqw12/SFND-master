#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/hough_3d.h>

int main(int argc, char **argv) {
  // Load point cloud data from file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>("cloud.pcd", *cloud);

  // Preprocess point cloud data
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(
      new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid.filter(*cloud_downsampled);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud_downsampled);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  ne.setSearchMethod(tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
      new pcl::PointCloud<pcl::Normal>);
  ne.setKSearch(50);
  ne.compute(*cloud_normals);

  // Apply Hough transform for plane detection
  pcl::Hough3DGrouping<pcl::PointXYZ, pcl::PointXYZ, pcl::PointNormal,
                       pcl::PointNormal>
      hough;
  hough.setInputCloud(cloud_downsampled);
  hough.setInputNormals(cloud_normals);
  hough.setUseSingleHypothesis(false);
  hough.setHoughBinSize(0.01f, 2.0f * M_PI / 180.0f, 0.01f);
  hough.setHoughThreshold(5);
  hough.setUseInterpolation(true);
  hough.setUseDistanceWeight(false);
  std::vector<pcl::PointIndices> plane_indices;
  hough.recognize(plane_indices);

  // Extract plane points from point cloud data
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud_downsampled);
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_points(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < plane_indices.size(); i++) {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices(plane_indices[i]));
    extract.setIndices(inliers);
    extract.setNegative(false);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane(
        new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*plane);
    *plane_points += *plane;
  }

  // Save plane points to file
  pcl::io::savePCDFile<pcl::PointXYZ>("plane_points.pcd", *plane_points);

  return 0;
}
