#include<iostream>
#include<time.h>
#include "dataloader.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>  
#include <pcl/visualization/pcl_visualizer.h>  
#include <thread>
using namespace cv;
using namespace std;
Eigen::MatrixXf intrinsic_;    
Eigen::Matrix4f extrinsic_ = Eigen::Matrix4f::Identity();
std::vector<double> dist_;
pcl::PointCloud<pcl::PointXYZ>::Ptr pc_(new pcl::PointCloud<pcl::PointXYZ>);
int IMG_H, IMG_W;
Mat imageTranslation1(Mat& srcImage);
static void UndistImg(cv::Mat & img, Eigen::Matrix3f intrinsic, std::vector<double> dist)
{
    cv::Mat I = cv::Mat::eye(3, 3, CV_32FC1);
    cv::Mat mapX, mapY;
    cv::Mat img_undist = cv::Mat(img.size(), CV_32FC3);
    cv::Mat K;
    cv::eigen2cv(intrinsic, K);
    cv::initUndistortRectifyMap(K, dist, I, K, img.size(), CV_32FC1, mapX, mapY);
    cv::remap(img, img_undist, mapX, mapY, cv::INTER_LINEAR);
    img = img_undist;
    
}
//图像翻转，图像大小不变
Mat imageTranslation1(Mat& srcImage)
{
	int nRows = srcImage.rows;
	int nCols = srcImage.cols;
	Mat resultImage(srcImage.size(), srcImage.type());
	//遍历图像
	for (int i = 0; i < nRows; i++)
	{
		for (int j = 0; j < nCols; j++)
		{

				resultImage.at<Vec3b>(i, j) = srcImage.at<Vec3b>(nRows-1-i, nCols-1-j);
		
		}
	}
	return resultImage;
}

bool ProjectOnImage(const Eigen::Vector4f &vec, const Eigen::Matrix4f &T, int &x, int &y, int margin)
{   Eigen::Matrix4f reverse ;
    //reverse << 1 , 0, 0, 0 ,
    //            0. ,-1. ,0. ,0. , 
    //            0. ,0. ,-1.,0. ,
    //            0. ,0. ,0. ,1.;
    Eigen::Vector3f vec2;
    if (intrinsic_.cols() == 4)
    {
        vec2 = intrinsic_ * T * vec;
    }
    else
    {
        Eigen::Vector4f cam_point = T * vec;
        Eigen::Vector3f cam_vec;
        cam_vec << cam_point(0), cam_point(1), cam_point(2);
        vec2 = intrinsic_ * cam_vec;
    }
    if (vec2(2) <= 0)
        return false;
    x = (int)cvRound(vec2(0) / vec2(2));
    y = (int)cvRound(vec2(1) / vec2(2));
    if (x >= -margin && x < IMG_W + margin && y >= -margin && y < IMG_H + margin)
        return true;
    return false;
}
void VisualProjection(Eigen::Matrix4f T, std::string img_file, std::string save_name)
{
    cv::Mat img_color = cv::imread(img_file);
    //lbl
    std::vector<float> distance;
    float max_distance =0;
    float dis = 0;
    if(intrinsic_.cols() == 3)
    {
        UndistImg(img_color, intrinsic_, dist_);
    }
    //img_color =imageTranslation1(img_color);
    std::vector<cv::Point2f> lidar_points;
    for (const auto &src_pt : pc_->points)
    {
        Eigen::Vector4f vec;
        vec << src_pt.x, src_pt.y, src_pt.z, 1;
        int x, y;
        if (ProjectOnImage(vec, T, x, y, 0))
        {
            //if (src_pt.intensity > 0) {
                cv::Point2f lidar_point(x, y);
                lidar_points.push_back(lidar_point);
                dis=sqrt((src_pt.x)*(src_pt.x)+(src_pt.y)*(src_pt.y)+(src_pt.z)*(src_pt.z));
                distance.push_back(dis);
                max_distance =std::max(max_distance,dis);
                //std::cout << "max_distance: " << dis << std::endl;
            //}
        }
    }
    
    int i=0;
    for (cv::Point point : lidar_points)
    {   if(distance[i]>30)
        {
            //cv::circle(img_color, point, 1, cv::Scalar(0, 0, 255), -1, 0);
        }
        else{
            cv::circle(img_color, point, 0.3, cv::Scalar(0, 255*(1-distance[i]/30), 255*distance[i]/30), -1, 0);            
        }

        i++;
    }
    cv::imwrite(save_name, img_color);
    std::cout << "Image saved: " << save_name << std::endl;
}


int main(int argc , char *argv[]){
    if (argc != 2) {
    std::cout << "Usage: ./bin/run_lidar2camera <data_folder>\n"
                    "example:\n\t"
                    "./bin/run_lidar2camera data/st/1\n"
                    "./bin/run_lidar2camera data/kitti/1" << std::endl;
    return 0;
    }
    std::string data_folder = argv[1];
    std::string lidar_file,img_file,calib_file;
    
    DIR *dir;
    struct dirent *ptr;
    dir = opendir(data_folder.c_str());
    while ((ptr = readdir(dir)) != NULL)
    {
        std::string name = ptr->d_name;
        auto n = name.find_last_of('.');
        if(name == "." || name == ".." || n == std::string::npos){
            ptr++;
            continue;
        }
        std::string suffix = name.substr(n);
        if (suffix == ".png" || suffix == ".jpg" || suffix == ".jpeg")
            img_file = data_folder + '/' + ptr->d_name;
        else if (suffix == ".pcd"||suffix == ".ply")
            lidar_file = data_folder + '/' + ptr->d_name;
        else if (suffix == ".txt")
            calib_file = data_folder + '/' + ptr->d_name;
        ptr++;
    }
    cv::Mat img = cv::imread(img_file);
    IMG_H = img.rows;
    IMG_W = img.cols;
    // load point cloud

    DataLoader::LoadCalibFile(calib_file, intrinsic_, extrinsic_, dist_);
    DataLoader::LoadLidarFile(lidar_file, pc_);
    VisualProjection(extrinsic_, img_file, data_folder+"/proj.png");
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PointCloud Visualizer"));
    viewer->setBackgroundColor(0.5, 0.5, 0.5);
    viewer->addPointCloud<pcl::PointXYZ>(pc_, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
 
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}