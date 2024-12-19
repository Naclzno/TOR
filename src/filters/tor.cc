#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <gflags/gflags.h>
#include <methods/timer.hh>
#include <cstdio>
#include <filters/tor.hh> 


DEFINE_string(f, "", "origin_cloud_txt");
DEFINE_string(ep, "", "ep path");
DEFINE_string(en, "", "en path");
DEFINE_int32(n, 3, "min_neighbours");
DEFINE_double(r, 0.12, "min_search_radius");
DEFINE_double(dt, 10.0, "distance threshold for preprocessing");
DEFINE_int32(it, 10, "intensity threshold for preprocessing");

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
  
 
  for (int i = 0; i < origin->points.size(); ++i) {
    pcl::PointXYZRGB pcl_point;
    convertToPclPointXYZRGB(origin->points[i], pcl_point);
    converted->points.push_back(pcl_point);
  }
  
 
  Timer timer;
  TOR outrem;
  outrem.SetMinNeighbors(FLAGS_n);
  outrem.SetSearchRadius(FLAGS_r);
  outrem.Filter(converted, *en, *ep);
  printf("%.2f ms\n", timer.Get() * 1e-6);

  savePointCloudToTxt(ep, FLAGS_ep);
  savePointCloudToTxt(en, environment_features, FLAGS_en);
  
  return 0;
}