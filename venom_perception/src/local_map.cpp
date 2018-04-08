#include <ros/ros.h> // ros
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h> // pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <string> // std
#include <fstream>
#include <vector>
#include <math.h>
#include <highgui.h> // opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense> // eigen
#include <Eigen/Geometry> 


int read_record(std::string cloud_src,
                pcl::PointCloud<pcl::PointXYZRGB>& cloud,
		std::string pose_src,
                Eigen::Vector3f& translation,
		Eigen::Affine3f& transform) {
  ROS_INFO_STREAM("Loading " << cloud_src << "...");
  pcl::PCLPointCloud2 pcl_pc2;
  if (pcl::io::loadPCDFile (cloud_src, pcl_pc2) == -1) {
    ROS_ERROR("Load failed");
    return 1;
  }
  pcl::fromPCLPointCloud2(pcl_pc2, cloud);

  ROS_INFO_STREAM("Loading " << pose_src << "...");
  std::ifstream file (pose_src);
  if (file.is_open()) {
    std::string line;
    for (int i=0; i<7; i++) {
      getline(file,line);
    }

    // Retrieve positional information
    getline(file,line);
    translation[0] = stod(line.substr(8,line.size()));
    getline(file,line);
    translation[1] = stod(line.substr(8,line.size()));
    getline(file,line);
    translation[2] = stod(line.substr(8,line.size()));

    // Rotate transform by quaternion
    transform = Eigen::Affine3f::Identity();
    getline(file,line);getline(file,line);
    float qx = stod(line.substr(8,line.size()));
    getline(file,line);
    float qy = stod(line.substr(8,line.size()));
    getline(file,line);
    float qz = stod(line.substr(8,line.size()));
    getline(file,line);
    float qw = stod(line.substr(8,line.size()));
    Eigen::Quaternion<float> q(qw,qx,qy,qz);
    
    transform.rotate(q);
    transform = transform.inverse();

    file.close();
  } else {
    ROS_ERROR_STREAM("Unable to open " << pose_src << ".txt" );
    return 1;
  }
  return 0;
}

int sp = 200, sa = 400; // map resolution
double max_dist = 8.0;

inline int idx(double x, double m, int s) {
  return fmod(x+m,m) / m * s;
}

cv::Mat map(sp,sa,CV_8UC3, cv::Vec3b(155,155,155)); // gray scale image format
void build_map( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ) {
  for (unsigned int i = 0; i < cloud->size(); i++) {
    pcl::PointXYZRGB& p = cloud->points[i];
    double polar = std::atan2(p.z, std::sqrt(p.y*p.y+p.x*p.x));
    double azimuth = std::atan2(p.y, p.x);
    int ip = idx(polar, M_PI, sp);
    int ia = idx(azimuth, 2.0 * M_PI, sa);
    uchar dist = 256 * (std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z)/max_dist);
    map.at<cv::Vec3b>(ip,ia) = cv::Vec3b(dist, 0, 0);
  }
  cv::imwrite( "map.png", map);
}

int main(int argc, char** argv) {

  std::string cloud_prefix = "samples/cloud_", cloud_postfix = ".pcd";
  std::string pose_prefix = "samples/pose_", pose_postfix = ".txt";

  std::vector<std::string> clouds, poses;
  for (int i = 1; i < argc; i++) {
    clouds.push_back(cloud_prefix+std::string(argv[i])+cloud_postfix);
    poses.push_back(pose_prefix+std::string(argv[i])+pose_postfix);
  }

  pcl::visualization::PCLVisualizer viewer("Matrix transform");

  for (int i = 0; i < clouds.size(); i++) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Vector3f translation;
    Eigen::Affine3f transform;
    read_record(clouds[i],*cloud, poses[i], translation, transform);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
    transform.translation() << translation;
    pcl::transformPointCloud (*cloud, *transformed, transform.inverse());

    build_map(transformed);

    viewer.addPointCloud(transformed, "transformed"+std::to_string(i));
  }
  while (!viewer.wasStopped()) {
    viewer.spinOnce();
  }


  return 0;
}
