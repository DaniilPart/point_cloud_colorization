#include <iostream>
#include <cmath>
#include <vector>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <Eigen/Dense>

int main(int argc, char** argv)
{
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <input_map.ply> <output_surfels.ply>\n";
    return -1;
  }

  const std::string input_file = argv[1];
  const std::string output_file = argv[2];

  // 1. Load the aggregated RGB point cloud
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(input_file, *cloud) == -1) {
    std::cerr << "Error: Failed to load " << input_file << "\n";
    return -1;
  }
  std::cout << "Loaded map with " << cloud->size() << " points.\n";

  // 2. Prepare the output Surfel cloud
  auto surfel_cloud = std::make_shared<pcl::PointCloud<pcl::PointSurfel>>();
  surfel_cloud->reserve(cloud->size());

  // 3. Initialize KD-Tree for neighborhood search
  pcl::search::KdTree<pcl::PointXYZRGB> tree;
  tree.setInputCloud(cloud);

  // You can tune this. 15 neighbors is usually the sweet spot for a 0.2m voxel grid.
  const int k_neighbors = 15; 
  const float scale_multiplier = 2.0f; // Multiplier to ensure surfels overlap slightly

  std::cout << "Calculating covariance, normals, and radii...\n";

  #pragma omp parallel for // Use your workstation's multi-threading
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    const auto & pt = cloud->points[i];
    std::vector<int> pointIdx(k_neighbors);
    std::vector<float> pointSqrDist(k_neighbors);

    pcl::PointSurfel surfel;
    surfel.x = pt.x;
    surfel.y = pt.y;
    surfel.z = pt.z;
    surfel.rgba = pt.rgba; // Pack RGB into the surfel
    surfel.confidence = 1.0f;

    if (tree.nearestKSearch(pt, k_neighbors, pointIdx, pointSqrDist) > 3) {
      // Compute Covariance Matrix
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cloud, pointIdx, centroid);
      
      Eigen::Matrix3f covariance_matrix;
      pcl::computeCovarianceMatrixNormalized(*cloud, pointIdx, centroid, covariance_matrix);

      // Extract Eigenvalues and Eigenvectors
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance_matrix, Eigen::ComputeEigenvectors);
      Eigen::Vector3f eigenvalues = solver.eigenvalues();
      Eigen::Matrix3f eigenvectors = solver.eigenvectors();

      // Eigenvalues are sorted ascending. 
      // The smallest eigenvalue (index 0) corresponds to the normal vector.
      surfel.normal_x = eigenvectors(0, 0);
      surfel.normal_y = eigenvectors(1, 0);
      surfel.normal_z = eigenvectors(2, 0);

      // The largest eigenvalue (index 2) represents the maximum variance (spread).
      // Standard deviation is the square root of variance.
      float max_std_dev = std::sqrt(std::max(eigenvalues[2], 0.0001f));
      
      // Radius is proportional to the local spread
      surfel.radius = max_std_dev * scale_multiplier;
      
      // Safety cap: don't let isolated points create massive 5-meter spheres
      if (surfel.radius > 1.0f) surfel.radius = 1.0f; 
    } else {
      // Fallback for isolated points
      surfel.normal_x = 0.0f; surfel.normal_y = 0.0f; surfel.normal_z = 1.0f;
      surfel.radius = 0.2f; 
    }

    #pragma omp critical
    {
      surfel_cloud->push_back(surfel);
    }
  }

  // 4. Save the enriched PLY
  pcl::io::savePLYFileBinary(output_file, *surfel_cloud);
  std::cout << "Surfel generation complete. Saved to: " << output_file << "\n";

  return 0;
}