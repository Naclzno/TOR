#ifndef INCLUDE_TOR_HH
#define INCLUDE_TOR_HH

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <methods/preprocessing.hh>


class TOR {
public:
  TOR() = default;
  ~TOR() = default;

  void SetSearchRadius(double search_radius) {
    search_radius_ = search_radius;
  }

  // void SetMinNeighbors(int min_neighbors) {
  //   min_neighbors_ = min_neighbors;
  // }

  double GetSearchRadius() { return search_radius_; }
  // int GetMinNeighbors() { return min_neighbors_; }

  template <typename T>
  std::vector<std::vector<int>> Filter(typename pcl::PointCloud<T>::Ptr &input_cloud) {
  
    using KdTreePtr = typename pcl::KdTreeFLANN<T>::Ptr;
    KdTreePtr kd_tree_(new pcl::KdTreeFLANN<T>());
    kd_tree_->setInputCloud(input_cloud);

    std::vector<std::vector<int>> b3_neighbors_count;

    for (typename pcl::PointCloud<T>::iterator it = input_cloud->begin();
         it != input_cloud->end(); ++it) {
      
      if (it->b != 3) continue;
      
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      
      int neighbors = kd_tree_->radiusSearch(*it, search_radius_, pointIdxRadiusSearch,
                                             pointRadiusSquaredDistance);
      
      int count_b1 = 0;
      int count_b2 = 0;
      
      for (const auto& idx : pointIdxRadiusSearch) {
        if (input_cloud->points[idx].b == 1) ++count_b1;
        if (input_cloud->points[idx].b == 2) ++count_b2;
      }

      b3_neighbors_count.push_back({count_b1, count_b2});
    }

    return b3_neighbors_count;
  }
    
    

private:
  double search_radius_; 
  // int min_neighbors_; 
};

#endif // INCLUDE_TOR_HH
