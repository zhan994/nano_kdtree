/**
 * Copyright (c) 2022 Jose Carlos Garcia (jcarlos3094@gmail.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Author Jose Carlos Garcia
 *
 */

#include <iostream>
#include <memory>
#include <random>
#include "nano_dbscan.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main()
{
  // define data
  const int                                             dim  = 3;
  std::shared_ptr<std::vector<std::array<double, dim>>> data = std::make_shared<std::vector<std::array<double, dim>>>();

  // generate artificial data
  std::random_device                     rd;
  std::mt19937                           gen(rd());
  std::uniform_real_distribution<double> distr(-1, 1);
  size_t                                 N = 500;
  size_t                                 n = 0;

  pcl::PointCloud<pcl::PointXYZRGB> cloud_pts;
  pcl::io::loadPCDFile("/home/zhan/segment.pcd", cloud_pts);
  N = cloud_pts.points.size();

  data->reserve(N);
  while (n < N)
  {
    data->push_back({{cloud_pts.points[n].x, cloud_pts.points[n].y, cloud_pts.points[n].z}});
    n++;
  }

  auto begin = std::chrono::high_resolution_clock::now();
  // build kdtree:
  auto adapt = Adaptor<std::array<double, dim>>(*data);

  using my_kd_tree_t =
      nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Adaptor<double, decltype(adapt)>, decltype(adapt), dim /* dim */
                                          >;

  my_kd_tree_t index(dim /*dim*/, adapt, {10 /* max leaf */});

  index.buildIndex();

  std::vector<std::vector<size_t>> clusters;
  double                           epsilon = 1;
  epsilon *= epsilon;
  const int min_pts = 15;
  NanoDBSCAN<my_kd_tree_t, std::array<double, dim>>(index, data, epsilon, min_pts, nanoflann::SearchParameters(0),
                                                    clusters);
  auto end = std::chrono::high_resolution_clock::now();

  // RESULTS
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
      point.z = data->at(id)[2];

      point.r = r;
      point.g = g;
      point.b = b;
      cluster_cloud.points.push_back(point);
    }
  }

  std::cout << "total cost: " << time_inc(end, begin) << ", clusters:" << clusters.size() << std::endl;
  cluster_cloud.width  = cluster_cloud.points.size();
  cluster_cloud.height = 1;
  pcl::io::savePCDFile("dbscan_out.pcd", cluster_cloud);

  return 0;
}