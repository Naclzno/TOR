#ifndef INCLUDE_DROR_HH
#define INCLUDE_DROR_HH

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <methods/preprocessing.hh> 

class DROR {
public:
  DROR() = default;
  ~DROR() = default;

  void SetRadiusMultiplier(double radius_multiplier) {
    radius_multiplier_ = radius_multiplier;
  }
  void SetAzimuthAngle(double azimuth_angle) { azimuth_angle_ = azimuth_angle; }
  // void SetMinNeighbors(int min_neighbors) { min_neighbors_ = min_neighbors; }
  void SetMinSearchRadius(double min_search_radius) {
    min_search_radius_ = min_search_radius;
  }

  double GetRadiusMultiplier() { return radius_multiplier_; }
  double GetAzimuthAngle() { return azimuth_angle_; }
  // int GetMinNeighbors() { return min_neighbors_; }
  double GetMinSearchRadius() { return min_search_radius_; }

  template <typename T>
  std::vector<int> Filter(typename pcl::PointCloud<T>::Ptr &input_cloud) {
    using KdTreePtr = typename pcl::KdTreeFLANN<T>::Ptr;
    std::vector<int> b3_neighbors_count; 

    KdTreePtr kd_tree_(new pcl::KdTreeFLANN<T>());
    kd_tree_->setInputCloud(input_cloud);

    for (typename pcl::PointCloud<T>::iterator it = input_cloud->begin();
         it != input_cloud->end(); ++it) {
      if (it->b != 3) continue; 

      float x_i = it->x;
      float y_i = it->y;
      float z_i = it->z;
      float range_i = sqrt(pow(x_i, 2) + pow(y_i, 2) + pow(z_i, 2));

      float search_radius_dynamic =
          radius_multiplier_ * sin(azimuth_angle_ * M_PI / 180) * range_i * 2;

      if (search_radius_dynamic < min_search_radius_) {
        search_radius_dynamic = min_search_radius_;
      }

      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;

      int neighbors =
          kd_tree_->radiusSearch(*it, search_radius_dynamic, pointIdxRadiusSearch,
                                 pointRadiusSquaredDistance);

      int b3_count = 0;
      for (const auto& idx : pointIdxRadiusSearch) {
        if (input_cloud->points[idx].b == 3) {
          ++b3_count;
        }
      }

      b3_neighbors_count.push_back(b3_count);
    }

    return b3_neighbors_count; 
  }

private:
  double radius_multiplier_; 
  double azimuth_angle_; 
  // int min_neighbors_; 
  double min_search_radius_; 
};

#endif // INCLUDE_DROR_HH