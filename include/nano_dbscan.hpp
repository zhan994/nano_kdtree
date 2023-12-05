/**
 * \file nano_dbscan.hpp
 * \author Zhihao Zhan (zhanzhihao_dt@163.com)
 * \brief dbscan based on nanoflann
 * \version 0.1
 * \date 2023-12-04
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef NANO_DBSCAN_H
#define NANO_DBSCAN_H

#include "nanoflann.hpp"

#include <chrono>

/**
 * \brief time cost in [ms]
 * 
 * \param t_end 
 * \param t_begin 
 * \return double 
 */
double time_inc(std::chrono::high_resolution_clock::time_point &t_end,
                std::chrono::high_resolution_clock::time_point &t_begin)
{
  return std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_begin).count() * 1000;
}

/**
 * \brief KdtreeAdaptor template struct
 * 
 * \tparam T element type
 */
template <typename T>
struct KdtreeAdaptor {
  KdtreeAdaptor(const std::vector<T> &position) : position_(position)
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

/**
 * \brief DBSCAN based on nanoflann
 *
 * \tparam KdtreeType type of kdtree
 * \tparam DataType type of data
 * \param tree built kdtree
 * \param data points data
 * \param epsilon range
 * \param min_pts minimum points
 * \param params flann search params
 * \param clusters dbscan result
 */
template <typename KdtreeType, typename DataType>
void NanoDBSCAN(const KdtreeType &                            tree,
                const std::shared_ptr<std::vector<DataType>> &data,
                double                                        epsilon,
                size_t                                        min_pts,
                const nanoflann::SearchParameters &           params,
                std::vector<std::vector<size_t>> &            clusters)
{
  std::vector<bool> visited;
  visited.resize(data->size(), false);
  std::vector<nanoflann::ResultItem<uint, double>> neighbor_pts;
  std::vector<nanoflann::ResultItem<uint, double>> neighbor_sub_pts;

  std::vector<size_t> cluster_outliers;
  for (size_t i = 0; i < data->size(); ++i)
  {
    // check if the point is visited
    if (visited[i])
      continue;

    // radius search around the unvisited point
    visited[i] = true;
    auto begin = std::chrono::high_resolution_clock::now();
    tree.radiusSearch(data->at(i).data(), epsilon, neighbor_pts, params);
    auto end = std::chrono::high_resolution_clock::now();
    // std::cout << "radius search cost: " << time_inc(end, begin) << std::endl;

    // check if the point is noise
    if (neighbor_pts.size() < static_cast<size_t>(min_pts))
    {
      cluster_outliers.emplace_back(i);
      continue;
    }

    // expand the clusters
    std::vector<size_t> cluster = std::vector<size_t>({i});
    while (!neighbor_pts.empty())
    {
      const unsigned long nb_idx = neighbor_pts.back().first;
      neighbor_pts.pop_back();
      if (visited[nb_idx])
        continue;

      visited[nb_idx] = true;
      begin           = std::chrono::high_resolution_clock::now();
      tree.radiusSearch(data->at(nb_idx).data(), epsilon, neighbor_sub_pts, params);
      end = std::chrono::high_resolution_clock::now();
      // std::cout << "radius search cost: " << time_inc(end, begin) << std::endl;

      if (neighbor_sub_pts.size() >= static_cast<size_t>(min_pts))
      {
        std::copy(neighbor_sub_pts.begin(), neighbor_sub_pts.end(), std::back_inserter(neighbor_pts));
      }
      cluster.emplace_back(nb_idx);
    }
    clusters.emplace_back(std::move(cluster));
  }
  clusters.emplace_back(std::move(cluster_outliers));
}

#endif // NANO_DBSCAN_H