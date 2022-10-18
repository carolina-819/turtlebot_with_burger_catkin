#include "../include/teste.h"
#include "../include/librealsense2/rsutil.h"
#include "../include/librealsense2/h/rs_types.h"
#include "../include/librealsense2/h/rs_frame.h"
#include <iostream>
#include <fstream>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <opencv2/features2d.hpp>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <chrono>
using namespace std::chrono;
using namespace cv;

typedef u_int16_t Pixel;

RNG_class var_nor_l1((time(0)), 0, sigma_lidar1); 
RNG_class var_nor_l2((time(0)), 0, sigma_lidar2); 
RNG_class var_nor_d((time(0)), 0, sigma_vision);



struct Operator {
        void operator ()(u_int16_t &pixel, const int * position) const{
          // std::cout << "valor do pixel " << var_nor_d.get() << std::endl;
           u_int16_t old_val = pixel;
           float dist = (float) old_val;
           int noise = std::round(var_nor_d.get(dist));
           pixel = old_val + noise;
        }
    };
void check_distance(std::vector<zone> &zonas, Point ponto, float distancia);

std::vector<double> sensor_fusion(double x_old, double y_old, double x_meas, double y_meas, double relation, double range, double teta){
    // range e teta são os valores que vem do lidar
    // old sao os valores da camara, meas sao os valores do lidar que corrigem
    float old_x_variance, old_y_variance, meas_x_variance, meas_y_variance;
    // camara, o x é a distancia, que vai ser o z, que tem de incerteza os 2% certos
    old_x_variance = std::pow(sigma_vision, 2);
    old_y_variance = std::pow(sigma_vision*(relation), 2);

    // lidar, a variancia está associada ao range
    // TODO receber o range e o teta. o range serve p saber qual sigma usar
    if (range * 1000  <= 499){ //mudar para precisao
        meas_x_variance = std::pow(sigma_lidar1 * std::cos(teta) * range * 1000, 2);
        meas_y_variance = std::pow(sigma_lidar1 * std::sin(teta) * range * 1000, 2);
    }else{
        meas_x_variance = std::pow(sigma_lidar2 * std::cos(teta) * range * 1000, 2);
        meas_y_variance = std::pow(sigma_lidar2 * std::sin(teta) * range * 1000, 2);
    }
    
    // CALCULATE X and y
    
    double x = (old_x_variance*x_old + x_meas*meas_x_variance)/(old_x_variance+meas_x_variance);
    double y = (old_y_variance*y_old + y_meas*meas_y_variance)/(old_y_variance+meas_y_variance);
    return std::vector<double> {x, y};
}

void cb_pcl(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PCLPointCloud2 pcl_toda;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_toda(new pcl::PointCloud<pcl::PointXYZ>);
    if(msg){
        pcl_conversions::toPCL(*msg, pcl_toda);
        pcl::fromPCLPointCloud2(pcl_toda, *pc_toda);

       // std::cout << " tamanho recebido " << pc_toda->points.size() << std::endl;
   
    }
   
}
void get_depth_camera_info()
{
    boost::shared_ptr<sensor_msgs::CameraInfo const> sharedCameraInfo;
    do
    {
        sharedCameraInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/d435/depth/camera_info", ros::Duration(5));
        ROS_WARN_STREAM("do");
        if (sharedCameraInfo != NULL)
        {
            cam_info = *sharedCameraInfo;
            intriseco_depth.ppx = cam_info.K[2];
            intriseco_depth.ppy = cam_info.K[5];
            intriseco_depth.fy = cam_info.K[4];
            intriseco_depth.fx = cam_info.K[0];
            intriseco_depth.height = cam_info.height;
            intriseco_depth.width = cam_info.width;
            frame_depth = cam_info.header.frame_id;
            if (cam_info.distortion_model == "plumb_bob") {
                intriseco_depth.model =  RS2_DISTORTION_BROWN_CONRADY;
            }else if  (cam_info.distortion_model == "equidistant") {
                intriseco_depth.model = RS2_DISTORTION_KANNALA_BRANDT4;
            }
            
            ROS_WARN_STREAM("Width = " << cam_info.width << " Height = " << cam_info.height);
        }
        else
        {
            ROS_ERROR("Couldn't get left camera info! Trying again...");
            ros::Duration(1.0).sleep();
        }
    } while (sharedCameraInfo == NULL);
    ROS_WARN_STREAM("done"); // esta preso no do
}

void get_color_camera_info()
{
    boost::shared_ptr<sensor_msgs::CameraInfo const> sharedCameraInfo;
    do
    {
        sharedCameraInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/d435/color/camera_info", ros::Duration(5));
        ROS_WARN_STREAM("do");
        if (sharedCameraInfo != NULL)
        {
            cam_info = *sharedCameraInfo;
            intriseco_color.ppx = cam_info.K[2];
            intriseco_color.ppy = cam_info.K[5];
            intriseco_color.fy = cam_info.K[4];
            intriseco_color.fx = cam_info.K[0];
            intriseco_color.height = cam_info.height;
            intriseco_color.width = cam_info.width;
            frame_color = cam_info.header.frame_id;
            if (cam_info.distortion_model == "plumb_bob") {
                intriseco_color.model =  RS2_DISTORTION_BROWN_CONRADY;
            }else if  (cam_info.distortion_model == "equidistant") {
                intriseco_color.model = RS2_DISTORTION_KANNALA_BRANDT4;
            }
            
        }
        else
        {
            ROS_ERROR("Couldn't get image camera info! Trying again...");
            ros::Duration(1.0).sleep();
        }
    } while (sharedCameraInfo == NULL);
    ROS_WARN_STREAM("done"); 
}

void cb_align(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::LaserScanConstPtr& lidar_msg) 
{
    auto start = high_resolution_clock::now();
 //   std::cout << "publish point " << std::endl;
    pcl::console::TicToc tt;
    tt.tic();
    rs2_intrinsics intriseco = intriseco_color;
    cv_bridge::CvImagePtr depth_image_ptr;
    
    try
    {
        depth_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
   
    Mat img = depth_image_ptr->image;
    float w = intriseco.width;
    float h =  intriseco.height;

    Mat graymat(h, w, CV_8UC1);

    double minVal, maxVal;
   // minMaxLoc(img, &minVal, &maxVal);
   
    
    Mat x_coord(intriseco.height, intriseco.width, CV_32F);
    Mat y_coord(intriseco.height, intriseco.width, CV_32F);
    Mat z_coord(intriseco.height, intriseco.width, CV_32F);
    
    Mat noisy_image = depth_image_ptr->image.clone();
    sensor_msgs::ImagePtr noisy_img_msg;

    sensor_msgs::LaserScan noisy_scan;
    noisy_scan.angle_min = lidar_msg->angle_min;
    noisy_scan.angle_max = lidar_msg->angle_max;
    noisy_scan.angle_increment = lidar_msg->angle_increment;
    noisy_scan.time_increment = lidar_msg->time_increment;
    noisy_scan.scan_time = lidar_msg->scan_time;
    noisy_scan.range_min = lidar_msg->range_min;
    noisy_scan.range_max = lidar_msg->range_max;
    noisy_scan.intensities = lidar_msg->intensities; 
    noisy_scan.header = lidar_msg->header;

    for(int n = 0; n < lidar_msg->ranges.size(); n++){
        float noise = 0;
        if(lidar_msg->ranges.at(n) <= 0.499 ){
            noise = static_cast<float>(var_nor_l1.get(1));
        }else{
            noise = static_cast<float>(var_nor_l2.get(lidar_msg->ranges.at(n)));
        }
        float new_val = lidar_msg->ranges.at(n) + noise;
        if(new_val < 0) {
            new_val=0;
        }
        noisy_scan.ranges.push_back(new_val);
    }
    // publishes noise scan
    noisy_scan.header.stamp = ros::Time::now();
    pub_noisy_scan.publish(noisy_scan);


// iget 3d coordinates
    noisy_image.forEach<Pixel>(Operator());
   // std::cout << "esta alterado " << std::endl;
  /*  for(u_int16_t i = 0; i < h; i++){ //goes through rows
        for (u_int16_t j = 0; j < w; j++){ //goes through columns
        //    float pixel[2] = {i, j};
            
         //   float point[3];
          //  graymat.at<unsigned char>(i, j) = uchar ((img.at<u_int16_t>(i, j) * 255)/maxVal);

          //  x_coord.at<float>(i, j) = ((j - intriseco.ppx)/(intriseco.fx)) * img.at<u_int16_t>(i, j) * 0.001;
          //  y_coord.at<float>(i, j) = ((i - intriseco.ppy)/(intriseco.fy)) * img.at<u_int16_t>(i, j) * 0.001;
          //  z_coord.at<float>(i, j) = img.at<u_int16_t>(i, j) * 0.001;
            
         //   noisy_image.at<u_int16_t>(i, j) = img.at<u_int16_t>(i, j) + static_cast<float>(var_nor_d());
        }
    }*/
    
    // publish noisy image
    std_msgs::Header header; // empty header
    header.stamp = ros::Time::now(); // time

    noisy_img_msg = cv_bridge::CvImage(header, msg->encoding, noisy_image).toImageMsg();
    pub_noisy_depth.publish(noisy_img_msg);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    std::cout << duration.count() << std::endl;
  //  Mat im_with_kp;
  // cv::drawKeypoints( graymat, keypoints_new, im_with_kp, Scalar(255,0,0), DrawMatchesFlags::DEFAULT );
    
    //cv::imshow("image grey", im_with_kp);
    //cv::waitKey(1);

    
 /*   pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pointcloud->header.frame_id = "d435_color_optical_frame";
    
    int size_pcl = 0;
    for (int i = 0; i < keypoints_new.size(); i++){
        // TODO FILTRAR KEYPOINTS AQUI COM A CENA DO LIDAR
        pcl::PointXYZ ponto;
        ponto.z = z_coord.at<float>(keypoints_new[i].pt.y, keypoints_new[i].pt.x);
        ponto.y = y_coord.at<float>(keypoints_new[i].pt.y, keypoints_new[i].pt.x);
        ponto.x = x_coord.at<float>(keypoints_new[i].pt.y, keypoints_new[i].pt.x);
    }
// se ponto.z > 2m podemos descartar esse ponto
// SENSOR FUSION
        //filtra pontos
         // passa para a frame do lidar, valores da camara
         double x_cam = ponto.z - dx;
         double y_cam = dy - ponto.x;
         double bearing_cam = std::atan2(y_cam, x_cam); // bearing aproximado
        double bearing_lidar = 0;
        int pos = bearing_cam/(lidar_msg->angle_increment);
        double distance = -1;
        int aux = -1;
        // otimizar para ter o range cuja diferença é menor
        if((abs(lidar_msg->ranges[pos] - ( std::sqrt(std::pow(x_cam, 2) + std::pow(y_cam, 2)) )) < tole ) && (abs(lidar_msg->ranges[pos] > 0)) ) {
            aux = pos;
        }
        else if((abs(lidar_msg->ranges[pos + 1] - ( std::sqrt(std::pow(x_cam, 2) + std::pow(y_cam, 2)) )) < tole ) && (abs(lidar_msg->ranges[pos + 1] > 0)) ) {
           aux = pos + 1;
        }else if((abs(lidar_msg->ranges[pos -1] - ( std::sqrt(std::pow(x_cam, 2) + std::pow(y_cam, 2)) )) < tole ) && (abs(lidar_msg->ranges[pos - 1] > 0)) ) {
            aux = pos - 1;
        }
        if(aux > -1) {
            distance = lidar_msg->ranges[aux];
            bearing_lidar = aux * lidar_msg->angle_increment;
        }
        
        if(distance > 0 || (lidar_msg->ranges[aux] != lidar_msg->ranges[aux])){
           // std::cout << "encontrou correspondencia" << i << std::endl;
            pcl::PointXYZ p;
            if(!lidar_msg->ranges[aux]){
                std::cout << "e um menino (a distancia e infinita)" << lidar_msg->ranges[aux] << std::endl;
                pointcloud->points.push_back(ponto); //nao da para fazer fusao
                
            }else{
                 // passa de volta para as coordenadas da frame de visao, desta vez com a informaçao do lidar, que a partida é mais reliable??
                // fusao de sensores
                // x_cam e y_cam: coordenadas calculadas com os valores que vem da camara, na frame do lidar
                // x_lidar e y_lidar: coordenadas com os valores que vem do lidar, na frame do lidar
                double x_lidar = (distance * std::cos(bearing_lidar));
                double y_lidar = (distance * std::sin(bearing_lidar));
                std::vector<double> new_val;
                new_val = sensor_fusion(x_cam, y_cam, x_lidar, y_lidar, (ponto.x/ponto.z), distance, bearing_lidar);
                p.y = ponto.y;
                p.z = new_val[0] + dx; 
                p.x = dy - new_val[1];
                pointcloud->points.push_back(p);
                std::cout << "x novo " << new_val[0] << " y novo " << new_val[1] << " x_cam " << x_cam << " y_cam " << y_cam << " x_lidar " << x_lidar << " y_lidar " << y_lidar << std::endl;
            }
            size_pcl++;
        }
    }
    pointcloud->width = size_pcl;
    pointcloud->height = 1;
    ros::Time time_st = ros::Time::now ();
    pointcloud->header.stamp = time_st.toNSec()/1e3;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*pointcloud.get(), cloud_msg);
    pub_cloud_depth.publish(cloud_msg);
    */

}



void cb_rgb(const sensor_msgs::Image &msg){ //detectar objetos, meter bounding boxes nos objetos

    
    cv_bridge::CvImagePtr depth_image_ptr = cv_bridge::toCvCopy(msg);
   
    img_new = depth_image_ptr->image.clone();
    Mat greyMat;
    cv::cvtColor(img_new, greyMat, COLOR_BGR2GRAY);
  
    // Initiate ORB detector
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();

    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );


    detector->detect ( img_new ,keypoints_new );

    descriptor->compute ( img_new, keypoints_new, descriptors_new);

   if(init){
    init = false;
   }else if(keypoints_old.size() > 0 && keypoints_new.size() > 0){

    std::vector<DMatch> matches;

    matcher->match ( descriptors_old, descriptors_new, matches );

    // find the keypoints and descriptors with ORB
    // TODO filtrar pontos pelo quao distinguishable sao
    //tendo a covariancia dos dois sensores no ponto especifico, calculo o que e que seria o ponto real tendo em conta essas duas covariancia (formulas do filtro de kalman, estado e observaçao xy)
    detector->detect(img_new, keypoints_new);
    Mat img_match;
    drawMatches ( img_old, keypoints_old, img_new, keypoints_new, matches, img_match );
    resize(img_match, img_match, cv::Size(img_match.cols/2, img_match.rows/2));
  //  imshow ( "所有匹配点对", img_match );
  //  waitKey(1);
   }
    //TODO GET DESCRIPTORS
  //  Mat im_with_keypoints;
  //  drawKeypoints( img_new, keypoints_keypoints, im_with_keypoints, Scalar(255,0,0), DrawMatchesFlags::DEFAULT );
    
    // do matching if not init
    
    // update new and old image
    img_old = img_new.clone();
    keypoints_old = keypoints_new;
    descriptors_old = descriptors_new;

  //  resize(im_with_keypoints, im_with_keypoints, cv::Size(im_with_keypoints.cols/2, im_with_keypoints.rows/2));
  //  resize(img_new, img_new, cv::Size(img_new.cols/2, img_new.rows/2));
     
  //  imshow("keypoints", im_with_keypoints);
  //  waitKey(1);

 
}
void cb_depth( const sensor_msgs::Image &msg){
    cv_bridge::CvImagePtr depth_image_ptr = cv_bridge::toCvCopy(msg);
 //   imshow("alinhado", depth_image_ptr->image);
 //   waitKey(1);

}

int main(int argc, char **argv)
{
    init = true;
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    get_depth_camera_info();
    get_color_camera_info();
    //  ros::Subscriber pcl_sub = nh.subscribe("/camera/depth_registered/points", 1, cb_pcl);
   // pub_cloud_depth = nh.advertise<sensor_msgs::PointCloud2> ("points_depth", 1);
    pub_noisy_depth = nh.advertise<sensor_msgs::Image> ("noisy_image", 1);
    pub_noisy_scan = nh.advertise<sensor_msgs::LaserScan> ("noisy_scan", 1);
    ros::Subscriber rgb_sub = nh.subscribe("d435/color/image_raw", 1, cb_rgb);

    message_filters::Subscriber<sensor_msgs::Image> cloud_depth_sub(nh, "/d435/aligned_depth_to_color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::LaserScan> lidar_sub(nh, "scan", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::LaserScan> MySyncPolicy;
    
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_depth_sub, lidar_sub);

    sync.registerCallback(boost::bind(&cb_align, _1, _2));
    


    std::cout << "o que e que se passa" << std::endl;
    while (ros::ok())
    {
        ros::spinOnce();
    }

    destroyAllWindows();
    return 0;
}
