#include <gflags/gflags.h>
#include <pcl/io/pcd_io.h>
#include <filters/dror-e.hh>
#include <filters/tor-e.hh>
#include <methods/ewm.hh>
#include <methods/timer.hh>
#include <cstdio>
#include <vector>


DEFINE_string(f, "", "origin_cloud");
DEFINE_string(ep, "./ep.pcd", "ep path");
DEFINE_string(en, "./en.pcd", "en path");
DEFINE_double(m, 3, "radius_multiplier");
DEFINE_double(a, 0.2, "azimuth_angle"); //0.2°
DEFINE_double(r, 0.04, "min_search_radius");
DEFINE_double(dt, 10.0, "distance threshold for preprocessing");
DEFINE_int32(it, 10, "intensity threshold for preprocessing");
DEFINE_double(r2, 0.12, "search_radius");
DEFINE_double(st, 0.5, "score threshold for ewm");


int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  PointCloudT::Ptr origin(new PointCloudT);
  PointCloudT::Ptr environment_features(new PointCloudT);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr en(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ep(new pcl::PointCloud<pcl::PointXYZRGB>);
    
  if (!loadPointCloudFromTxt(FLAGS_f, origin)) {
    return -1;
  }

  preprocessPointCloud(origin, environment_features, FLAGS_dt, FLAGS_it);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr converted(new pcl::PointCloud<pcl::PointXYZRGB>);

  int count_b3 = 0;
  
  for (int i = 0; i < origin->points.size(); ++i) {
    pcl::PointXYZRGB pcl_point;
    convertToPclPointXYZRGB(origin->points[i], pcl_point);
    converted->points.push_back(pcl_point);
  }

  Timer timer;
  DROR dror_filter;
  TOR tor_filter;
  dror_filter.SetRadiusMultiplier(FLAGS_m);
  dror_filter.SetAzimuthAngle(FLAGS_a);
  dror_filter.SetMinSearchRadius(FLAGS_r);
  tor_filter.SetSearchRadius(FLAGS_r2);

  std::vector<int> dror_results = dror_filter.Filter<pcl::PointXYZRGB>(converted);

  std::vector<std::vector<int>> tor_results = tor_filter.Filter<pcl::PointXYZRGB>(converted);

  std::vector<std::vector<double>> combined_matrix;

  for (size_t i = 0; i < dror_results.size(); ++i) {
    std::vector<double> row(3);
    row[0] = static_cast<double>(dror_results[i]);
    row[1] = static_cast<double>(tor_results[i][0]);
    row[2] = static_cast<double>(tor_results[i][1]);
    combined_matrix.push_back(row);
  }

  std::vector<int> highScoreIndices = calculateScores(combined_matrix, FLAGS_st);

  int b3_index = 0;
  for (size_t i = 0; i < converted->points.size(); ++i) {
    if (converted->points[i].b == 3) {
      if (std::find(highScoreIndices.begin(), highScoreIndices.end(), b3_index) != highScoreIndices.end()) {
        en->points.push_back(converted->points[i]); 
      } else {
        ep->points.push_back(converted->points[i]);
      }
      b3_index++;
    }
  }
  
  printf("%.2f ms\n", timer.Get() * 1e-6);

  savePointCloudToTxt(ep, FLAGS_ep);
  // savePointCloudToTxt(en, FLAGS_en);
  savePointCloudToTxt(en, environment_features, FLAGS_en);
  
  return 0;
}