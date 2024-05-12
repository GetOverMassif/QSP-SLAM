// 本文件做简单的四元数计算

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

int main()
{
    Eigen::Quaterniond quat;
    quat.x() = -0.5;
    quat.y() = 0.5;
    quat.z() = -0.5;
    quat.w() = 0.5;

    std::cout << "init quat x,y,z,w: " << quat.coeffs().transpose() << std::endl;

    // Eigen::AngleAxisd z_rot(-M_PI/180*30, Vector3d(1,0,0));  // 尝试转30度
    Eigen::AngleAxisd z_rot(-M_PI/180*3.2, Vector3d(1,0,0));  // 尝试转
    auto quat_new = quat * z_rot;   // 右乘

    std::cout << "after, quat : " << quat_new.coeffs().transpose() << std::endl;

    return 0;


}