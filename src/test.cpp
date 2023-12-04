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



template <typename T>
struct KdtreePositionAdaptor {
  KdtreePositionAdaptor(const std::vector<T> &position) : position_(position)
  {}
  const std::vector<T> &position_;

  inline size_t kdtree_get_point_count() const
  {
    return position_.size();
  }
  inline double kdtree_get_pt(const size_t idx, const size_t dim = 0) const
  {
    if (dim == 0)
    {
      return position_[idx].at(0);
    } else if (dim == 1)
    {
      return position_[idx].at(1);
    } else
    {
      return position_[idx].at(2);
    }
  }

  template <class BBOX>
  bool kdtree_get_bbox(BBOX & /*bb*/) const
  {
    return false;
  }
};

int main()
{
  // define data
  const int                                             dim  = 3;
  std::shared_ptr<std::vector<std::array<double, dim>>> data = std::make_shared<std::vector<std::array<double, dim>>>();

  // generate artificial data
  std::random_device                     rd;
  std::mt19937                           gen(rd());
  std::uniform_real_distribution<double> distr(-1, 1);
  size_t                                 N = 2000;
  size_t                                 n = 0;

  data->reserve(N);
  while (n < N)
  {
    data->push_back({{distr(gen), distr(gen), distr(gen)}});
    n++;
  }

  auto        begin = std::chrono::high_resolution_clock::now();

  // build kdtree:
  auto adapt = KdtreePositionAdaptor<std::array<double, dim>>(*data);

  using my_kd_tree_t =
      nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Adaptor<double, decltype(adapt)>, decltype(adapt), dim /* dim */
                                          >;

  my_kd_tree_t index(dim /*dim*/, adapt, {10 /* max leaf */});

  index.buildIndex();

  std::vector<std::vector<size_t>> clusters;
  double                           epsilon = 0.2;
  epsilon *= epsilon;
  const int min_pts = 3;
  NanoDBSCAN<my_kd_tree_t, std::array<double, dim>>(index, data, epsilon, min_pts, nanoflann::SearchParameters(10),
                                                    clusters);
  auto        end = std::chrono::high_resolution_clock::now();

  // RESULTS
  // for (const auto &cluster : clusters)
  // {
  //   std::cout << "Cluster: " << std::endl;
  //   for (const auto &id : cluster)
  //   {
  //     std::cout << "- Data " << id << " : (" << data->at(id)[0] << ", " << data->at(id)[1] << ", " << data->at(id)[2]
  //               << ")" << std::endl;
  //   }
  //   std::cout << std::endl;
  // }

  std::cout << "total cost: " << time_inc(end, begin) << std::endl;

  return 0;
}