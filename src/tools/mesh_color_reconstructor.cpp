#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include <pcl/conversions.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <input_map.ply> <output_mesh.ply>\n";
    return -1;
  }

  const std::string input_file = argv[1];
  const std::string output_file = argv[2];

  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(input_file, *cloud) == -1) {
    std::cerr << "Error: Failed to load " << input_file << "\n";
    return -1;
  }
  std::cout << "Loaded map with " << cloud->size() << " points.\n";

  auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
  auto normal_tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
  ne.setSearchMethod(normal_tree);
  ne.setInputCloud(cloud);
  ne.setKSearch(20);

  std::cout << "Estimating normals...\n";
  ne.compute(*normals);

  auto cloud_with_normals = std::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
  pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

  // 4. Run Greedy Projection Triangulation (Local Patches)
  pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
  auto gp3_tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZRGBNormal>>();
  pcl::PolygonMesh mesh;
  constexpr double kPi = 3.14159265358979323846;

  // Set the maximum distance between connected points (maximum edge length)
  // Because your map is 0.2m sparse, setting this to 0.4m or 0.5m prevents 
  // the algorithm from connecting distant branches or ground patches.
  gp3.setSearchRadius(0.5); 

  // Set typical thresholds for local patch generation
  gp3.setMu(2.5); // Multiplier for the nearest neighbor distance
  gp3.setMaximumNearestNeighbors(100);
  gp3.setMaximumSurfaceAngle(kPi / 4.0); // 45 degrees
  gp3.setMinimumAngle(kPi / 18.0);       // 10 degrees
  gp3.setMaximumAngle(2.0 * kPi / 3.0);  // 120 degrees
  gp3.setNormalConsistency(false);

  std::cout << "Running Greedy Projection Triangulation...\n";
  gp3.setInputCloud(cloud_with_normals);
  gp3.setSearchMethod(gp3_tree);
  gp3.reconstruct(mesh);

  auto mesh_vertices = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::fromPCLPointCloud2(mesh.cloud, *mesh_vertices);

  pcl::KdTreeFLANN<pcl::PointXYZRGB> color_tree;
  color_tree.setInputCloud(cloud);

  const int k_neighbors = 4;
  const double idw_power = 2.0;

  std::cout << "Interpolating colors for " << mesh_vertices->size() << " mesh vertices...\n";
  for (auto & vertex : mesh_vertices->points) {
    std::vector<int> point_idx_nkn_search(k_neighbors);
    std::vector<float> point_nkn_squared_distance(k_neighbors);

    if (color_tree.nearestKSearch(
        vertex, k_neighbors, point_idx_nkn_search, point_nkn_squared_distance) > 0)
    {
      double r = 0.0;
      double g = 0.0;
      double b = 0.0;
      double weight_sum = 0.0;

      for (std::size_t i = 0; i < point_idx_nkn_search.size(); ++i) {
        double dist = std::sqrt(point_nkn_squared_distance[i]);
        if (dist < 1e-6) {
          dist = 1e-6;
        }

        const double weight = 1.0 / std::pow(dist, idw_power);
        const auto & neighbor = cloud->points[point_idx_nkn_search[i]];
        r += neighbor.r * weight;
        g += neighbor.g * weight;
        b += neighbor.b * weight;
        weight_sum += weight;
      }

      vertex.r = static_cast<std::uint8_t>(r / weight_sum);
      vertex.g = static_cast<std::uint8_t>(g / weight_sum);
      vertex.b = static_cast<std::uint8_t>(b / weight_sum);
    }
  }

  pcl::toPCLPointCloud2(*mesh_vertices, mesh.cloud);

  pcl::io::savePLYFileBinary(output_file, mesh);
  std::cout << "Reconstruction complete. Saved to: " << output_file << "\n";

  return 0;
}