#ifndef INCLUDE_TOR_HH
#define INCLUDE_TOR_HH

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <methods/preprocessing.hh>


class TOR {
public:
  TOR() = default;
  ~TOR() = default;

  void SetSearchRadius(double search_radius) {
    search_radius_ = search_radius;
  }

  void SetMinNeighbors(int min_neighbors) {
    min_neighbors_ = min_neighbors;
  }

  double GetSearchRadius() { return search_radius_; }
  int GetMinNeighbors() { return min_neighbors_; }

  template <typename T>
  void Filter(typename pcl::PointCloud<T>::Ptr &input_cloud,
              typename pcl::PointCloud<T> &filtered_cloud,
              typename pcl::PointCloud<T> &noise_cloud) {
  
  using KdTreePtr = typename pcl::KdTreeFLANN<T>::Ptr;
  filtered_cloud.clear();
  noise_cloud.clear();
  KdTreePtr kd_tree_(new pcl::KdTreeFLANN<T>());
  kd_tree_->setInputCloud(input_cloud);

  for (typename pcl::PointCloud<T>::iterator it = input_cloud->begin();
       it != input_cloud->end(); ++it) {
    
    if (it->b != 3) continue;
    
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    int neighbors = kd_tree_->radiusSearch(*it, search_radius_, pointIdxRadiusSearch,
                                           pointRadiusSquaredDistance);
    
    int count_time_1 = 0;
    int count_time_2 = 0;

    for (const auto& idx : pointIdxRadiusSearch) {
      if (input_cloud->points[idx].b == 1) ++count_time_1;
      if (input_cloud->points[idx].b == 2) ++count_time_2;
    }

    if (count_time_1 >= min_neighbors_ && count_time_2 >= min_neighbors_) {
      filtered_cloud.push_back(*it);
    } else {
      noise_cloud.push_back(*it);
    }
  }
}
    
private:
  double search_radius_; 
  int min_neighbors_; 
};

#endif // INCLUDE_TOR_HH