#include <vector>
#include <unordered_map>
#include <limits>
#include <algorithm>
#include <utility>
#include <fstream>
#include <dirent.h>
#include <array>
#include <map>
#include <memory>
#include <string>
#include <chrono>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <pcl/io/ply_io.h>
class DataLoader {
    public:
        DataLoader() = default;
        ~DataLoader() = default;

        
        static void LoadCalibFile(
            const std::string filename, 
            Eigen::MatrixXf& intrinsic, 
            Eigen::Matrix4f& extrinsic,
            std::vector<double>& dist) 
        {
            std::ifstream file(filename);
            if (!file.is_open()) {
                std::cout << "open file " << filename << " failed." << std::endl;
                exit(1);
            }
            std::string line, tmpStr;
            getline(file, line);
            std::stringstream ss(line);
            std::vector<float> elements;
            std::string elem;
            getline(ss, elem, ' ');
            while (getline(ss, elem, ' '))
            {   
                elements.emplace_back(stof(elem));
            }
            std::cout<<"intrinsic is ok"<<std::endl;
            if(elements.size() == 9)
            {
                intrinsic = Eigen::Map<Eigen::MatrixXf>(elements.data(), 3, 3).transpose();
            }
            else if(elements.size() == 12)
            {
                intrinsic = Eigen::Map<Eigen::MatrixXf>(elements.data(), 4, 3).transpose();
            }
            else{
                std::cout << "Wrong intrinsic parameter number." << std::endl;
                exit(1);
            }

            getline(file, line);
            ss = std::stringstream(line);
            getline(ss, elem, ' ');
            while (getline(ss, elem, ' '))
            {
                dist.emplace_back(stod(elem));
            }
            std::cout<<"distort"<<std::endl;
            getline(file, line);
            ss = std::stringstream(line);
            ss >> tmpStr >> extrinsic(0, 0) >> extrinsic(0, 1) >> extrinsic(0, 2) >> extrinsic(0, 3)
                >> extrinsic(1, 0) >> extrinsic(1, 1) >> extrinsic(1, 2) >> extrinsic(1, 3)
                >> extrinsic(2, 0) >> extrinsic(2, 1) >> extrinsic(2, 2) >> extrinsic(2, 3);
        }


        static void LoadLidarFile(const std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr pc)
        {   std::cout<< filename<<std::endl;
            //if (pcl::io::loadPCDFile(filename, *pc) < 0)
            //{
            //    std::cout << "[ERROR] cannot open pcd_file: " << filename << "\n";
                //exit(1);
            //}
            if (pcl::io::loadPLYFile(filename,*pc)<0)
            {
                std::cout << "[ERROR] cannot open ply_file:" <<filename<<"\n";
                
            }
           
        }

    private:
};
