// 对于部分数据集，真值到相机之间还存在一个变换
// 本工具用于完成变换并保存
#include <iostream>

#include "include/utils/dataprocess_utils.h"
#include "include/utils/matrix_utils.h"
#include <Eigen/Core>

#include <Thirdparty/g2o/g2o/types/se3quat.h>

using namespace Eigen;
typedef Matrix<double, 7, 1> Vector7d;

int main(int argc, char *argv[])
{
    if( argc != 2)
    {
        std::cout << "usage: " << argv[0] << " path_to_odom" << std::endl;
        return 1;
    }
    const string gt_path(argv[1]);
    std::cout << " Reading odom from : " << gt_path << std::endl;
    MatrixXd gtMat = readDataFromFile(gt_path.c_str(), true);

    Vector7d transVec; 

    bool bUseMat = false;   // 若使用mat输入，打开这里
    if(bUseMat)
    {
        Matrix4d transMat;
        transMat << 9.9792816252667338e-03, 6.5348103708624539e-03,
                9.9992885256485176e-01, 2.2648368490900000e-01,
                -9.9982014658446139e-01, 1.6192923276330706e-02,
                9.8723715283343672e-03, -5.1141940356500000e-02,
                -1.6127257115523985e-02, -9.9984753112121250e-01,
                6.6952288046080444e-03, 9.1600000000000004e-01, 0., 0., 0.,
                1.;


        Matrix3d rotMat = transMat.block(0,0,3,3);
        Eigen::Quaterniond quat(rotMat);
        transVec << transMat(0,3), transMat(1,3), transMat(2,3), quat.x(), quat.y(), quat.z(), quat.w();
    }
    else
    {
        // 相机外参数 in NMB Dataset
        // transVec << 0.15, 0.15, 0.86, -0.5,0.5,-0.5,0.5; // x,y,z, qx,qy,qz,qw
        transVec << 0.15, 0.15, 0.893, -0.513766,  0.513766, -0.485844,  0.485844; // x,y,z, qx,qy,qz,qw
    }
    
    g2o::SE3Quat Toc;   // camera in object
    Toc.fromVector(transVec);
    std::cout << " Transform Vec: " << transVec.transpose() << std::endl;
    std::cout << " Transform Matrix: " << Toc.to_homogeneous_matrix() << std::endl;
    
    int num = gtMat.rows();
    MatrixXd camMat; camMat.resize(0,8);
    for(int i=0;i<num;i++)
    {
        VectorXd pose = gtMat.row(i);
        g2o::SE3Quat Two; Two.fromVector(pose.tail(7));
        g2o::SE3Quat Twc = Two * Toc;
        
        VectorXd pose_c = Twc.toVector();
        VectorXd pose_withTime; pose_withTime.resize(8);
        pose_withTime << pose[0], pose_c.tail(7);
        addVecToMatrix(camMat, pose_withTime);
    }

    saveMatToFile(camMat, "TransformedCameraPoses.txt");
    std::cout << "Transform " << num << "poses to : " << "TransformedCameraPoses.txt" << std::endl;
}