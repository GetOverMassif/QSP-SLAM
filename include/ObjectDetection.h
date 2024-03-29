/**
* This file is part of https://github.com/JingwenWang95/DSP-SLAM
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#ifndef OBJECTDETECTION_H
#define OBJECTDETECTION_H

#include <mutex>
#include <vector>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZ PointType;

namespace ORB_SLAM2
{

class ObjectDetection {
public:
    // Detction from LiDAR frame, with initial pose, surface LiDAR points, rays and depth measurement
    ObjectDetection(const Eigen::Matrix4f &T, const Eigen::MatrixXf &Pts, const Eigen::MatrixXf &Rays,
                    const Eigen::VectorXf &Depth);

    ObjectDetection();  // Detection from Mono frame
    void SetPoseMeasurementSim3(const Eigen::Matrix4f &T);
    void SetPoseMeasurementSE3(const Eigen::Matrix4f &T);
    std::vector<int> GetFeaturePoints();
    void AddFeaturePoint(const int &i);
    int NumberOfPoints();

    void setPcdPtr(pcl::PointCloud<PointType>::Ptr& pcd_ptr_);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix4f Sim3Tco;
    Eigen::Matrix4f SE3Tco;
    float scale;
    Eigen::Matrix3f Rco;
    Eigen::Vector3f tco;
    Eigen::MatrixXf SurfacePoints;
    Eigen::MatrixXf RayDirections;
    Eigen::VectorXf DepthObs;
    Eigen::MatrixXf background_rays;
    std::vector<int> mvKeysIndices;
    int nRays;
    int nPts;
    bool isNew;  // 记录是否已经与一个地图物体关联
    bool isGood;  // 记录是否有足够多关联特征点
    bool isValidPcd;
    std::mutex mMutexFeatures;
    std::mutex mMutexDetection;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> valid_edges;
    Eigen::Vector4d bbox;
    int label;
    double prob;

    pcl::PointCloud<PointType>::Ptr pcd_ptr;
};
}


#endif //OBJECTDETECTION_H
