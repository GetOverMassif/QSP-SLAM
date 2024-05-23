// Builder generates dense point cloud from RGB-D data and groundtruth camera poses.

#ifndef DENSE_BUILDER_H
#define DENSE_BUILDER_H

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudPCL;

#include <string>

using namespace Eigen;
using namespace std;

struct Camera
{
    double fx;
    double fy;
    double cx;
    double cy;
};

PointCloudPCL::Ptr VoxelFilterPointCloud(PointCloudPCL::Ptr pointcloud, double grid_size);

class Builder
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Builder();

    void setCameraIntrinsic(Matrix3d &calibMat, double scale);

    void processFrame(cv::Mat &rgb, cv::Mat &depth, Eigen::VectorXd &pose, double depth_thresh = 1000); 
    PointCloudPCL::Ptr getMap();
    PointCloudPCL::Ptr getCurrentMap();
    void saveMap(const string& path);

    // void voxelFilter(PointCloudPCL::Ptr pointcloud, double grid_size);
    void voxelFilter(double grid_size);

private:
    void getCameraParam(Matrix3d &calib, Camera &cam);
    PointCloudPCL::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, double depth_thresh);  
    PointCloudPCL::Ptr transformToWolrd( PointCloudPCL::Ptr& pPointCloud, VectorXd &pose);

    void addPointCloudToMap(PointCloudPCL::Ptr pPointCloud);

private:

    Matrix3d mmCalib;    // calibration mat
    double mdScale;   // scale for depth

    bool mbInitialized;

    PointCloudPCL::Ptr mpMap;
    PointCloudPCL::Ptr mpCurrentMap;

};

#endif  // DENSE_BUILDER_H