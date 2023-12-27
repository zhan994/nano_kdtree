/**
 * \file test_nano_dbscan_2d.cpp
 * \author Zhihao Zhan (zhanzhihao_dt@163.com)
 * \brief test2d nano dbscan
 * \version 0.1
 * \date 2023-12-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <iostream>
#include <memory>
#include <random>
#include "nano_dbscan.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char **argv)
{
  if (argc != 3)
  {
    std::cout << "usage: ./test3d eps[float] min_pts[int]" << std::endl;
  }
  double    epsilon = std::stod(argv[1]);
  const int min_pts = std::stoi(argv[2]);

  // define data
  const int                                             dim  = 2;
  std::shared_ptr<std::vector<std::array<double, dim>>> data = std::make_shared<std::vector<std::array<double, dim>>>();

  pcl::PointCloud<pcl::PointXYZRGB> cloud_pts;
  pcl::io::loadPCDFile("../pcd/segment.pcd", cloud_pts);
  size_t N = cloud_pts.points.size();
  data->reserve(N);
  size_t n = 0;
  while (n < N)
  {
    data->push_back({{cloud_pts.points[n].x, cloud_pts.points[n].y}});
    n++;
  }

  auto begin = std::chrono::high_resolution_clock::now();
  // step: 1. build kdtree based on nanoflann
  auto adapt = KdtreeAdaptor<std::array<double, dim>>(*data);
  using my_kd_tree_t =
      nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Adaptor<double, decltype(adapt)>, decltype(adapt), dim /* dim */
                                          >;
  my_kd_tree_t index(dim /*dim*/, adapt, {10 /* max leaf */});
  index.buildIndex();

  // step: 2. nano_dbscan
  std::vector<std::vector<size_t>> clusters;
  // note: L2_Adaptor -> radius should be squared.
  NanoDBSCAN<my_kd_tree_t, std::array<double, dim>>(index, data, epsilon * epsilon, min_pts,
                                                    nanoflann::SearchParameters(0), clusters);
  auto end = std::chrono::high_resolution_clock::now();

  // step: 3. RESULTS
  pcl::PointCloud<pcl::PointXYZRGB> cluster_cloud;
  for (const auto &cluster : clusters)
  {
    // 为当前聚类分配一个随机颜色
    uint8_t r = static_cast<uint8_t>(rand() % 256);
    uint8_t g = static_cast<uint8_t>(rand() % 256);
    uint8_t b = static_cast<uint8_t>(rand() % 256);

    for (const auto &id : cluster)
    {
      pcl::PointXYZRGB point;
      point.x = data->at(id)[0];
      point.y = data->at(id)[1];
      point.z = 0;

      point.r = r;
      point.g = g;
      point.b = b;
      cluster_cloud.points.push_back(point);
    }
  }

  std::cout << "total cost: " << time_inc(end, begin) << ", clusters:" << clusters.size() << std::endl;
  cluster_cloud.width  = cluster_cloud.points.size();
  cluster_cloud.height = 1;
  pcl::io::savePCDFile("dbscan_out_2d.pcd", cluster_cloud);

  return 0;
}