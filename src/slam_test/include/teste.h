#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/LaserScan.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include "../include/librealsense2/h/rs_types.h"
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core/types.hpp>
#include <random>
#include <cmath>
#include <boost/date_time/gregorian/gregorian_types.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/thread/thread.hpp>
 
 //MUDAR DE BOOST PARA STD
double dx = 0.018;
double dy = 0.0299;
double dz = 0.0205;
float tole = 0.5;

bool init;
typedef struct {
    double fx, fy;
    double cx, cy;
} camera_parameters;

typedef struct {
    double avg;
    std::vector<cv::Point> pontos;
} zone;

camera_parameters camera_params;

rs2_intrinsics intriseco_depth;
rs2_intrinsics intriseco_color;
sensor_msgs::CameraInfo cam_info;
int ts = 2;
pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
ros::Publisher pub_cloud_depth, pub_cloud_lidar, pub_noisy_depth, pub_noisy_scan;
std::string frame_color;
std::string frame_depth;
std::vector<cv::KeyPoint> keypoints_old, keypoints_new;

cv::Mat img_old, img_new;
cv::Mat descriptors_old, descriptors_new;

cv::Mat depth_map;

std::ofstream MyFile;

//standard deviations
const double sigma_lidar1 = 0.001/3;    //0.001; //em metros (120mm-499mm) mudou de 0.01 para 0.001
const double sigma_lidar2 = 0.005/3;    //0.035; //em percentagem (499mm-3500mm)
const double sigma_vision = 0.10/3;    //0.05; //em percentagem aumentado de 2 para 5 para ser maior que o lidar

double recalc_z, recalc_x;

// simulate noise
  //  boost::mt19937 rng;
  //  rng.seed(static_cast<unsigned int>(time(0)));
  //  boost::normal_distribution<> nd_d(0, 0.1); // for rgbd camera
  //  boost::normal_distribution<> nd_l(0, 0.01); // for lidar
  //  boost::variate_generator<boost::mt19937 &, boost::normal_distribution<>> var_nor_d(rng, nd_d);
   // boost::variate_generator<boost::mt19937 &, boost::normal_distribution<>> var_nor_l(rng, nd_l);
float normal_d;
float normal_l;

class RNG_class {
    private:
        std::mt19937 rng;
        std::normal_distribution<> dist;
        double mean;
        double stdv;
    public:
        RNG_class(int seed, double meanNormal, double stdvNormal)  {
            rng = std::mt19937(seed);
            mean = meanNormal;
            stdv = stdvNormal;
        }
        double get(float d) {
            if(d>0 && d==d){
                double new_var = d*(stdv);
                dist = std::normal_distribution<>(mean, new_var);
                return dist(rng);
            }else {
                return 0;
            }
            
        }
};