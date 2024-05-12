// Update : 2020-5-27 适应新版本.

// Update : 2020-1-4 完善椭球体评估功能
// 评估物体重建和真实位置之间的差值

#include <iostream>
#include <fstream>
#include <Eigen/Core>

#include "include/utils/dataprocess_utils.h"

#include <core/Ellipsoid.h>
#include <core/Geometry.h>
#include <core/System.h>

using namespace std;
using namespace Eigen;

EllipsoidSLAM::System* pSLAM;
EllipsoidSLAM::Map* pMap;

struct COMPARE_RESULT
{
    int instanceID;

    
    double IoU;  // 原始IoU
    double IoU_aligned;   // 对齐中心后的IoU

    double dis_trans;
    double dis_yaw;
};


// obj : instance x y z r p yaw a b c
//           0    1 2 3 4 5  6  7 8 9
double MonteCarloIoU(VectorXd &ob1, VectorXd &ob2)
{
    // 获得立方体.
    // 取 x,y,z最大值
    double radius1 = MAX(MAX(ob1[7], ob1[8]), ob1[9]);
    double radius2 = MAX(MAX(ob2[7], ob2[8]), ob2[9]);

    double x_max = MAX(ob1[1]+ radius1, ob2[1]+ radius2) ;
    double x_min = MIN(ob1[1]- radius1, ob2[1]- radius2);
    double y_max = MAX(ob1[2]+ radius1, ob2[2] + radius2);
    double y_min = MIN(ob1[2]- radius1, ob2[2] - radius2);
    double z_max = MAX(ob1[3]+ radius1, ob2[3] + radius2);
    double z_min = MIN(ob1[3]- radius1, ob2[3] - radius2);

    double resolution = 0.01; // 点的分辨率 m .
    // 随机 Sample 点云
    int total_num = 0;

    int ob1_num = 0; 
    int ob2_num = 0;
    int both_num = 0;

    g2o::ellipsoid e1; e1.fromMinimalVector(ob1.tail(9));
    Eigen::Matrix4d Q1_star = e1.generateQuadric();
    Eigen::Matrix4d Q1 = Q1_star.inverse();

    g2o::ellipsoid e2; e2.fromMinimalVector(ob2.tail(9));
    Eigen::Matrix4d Q2_star = e2.generateQuadric();
    Eigen::Matrix4d Q2 = Q2_star.inverse();

    EllipsoidSLAM::PointCloud* pCloud = new EllipsoidSLAM::PointCloud;
    for(double x=x_min; x<x_max; x+=resolution)
    {
        for(double y=y_min; y<y_max; y+=resolution)
        {
            for(double z=z_min; z<z_max; z+=resolution)
            {
                Vector3d point_vec(x,y,z);
                Eigen::Vector4d X = real_to_homo_coord_vec<double>(point_vec);

                bool isInside_1 = (X.transpose() * Q1 * X) < 0;
                bool isInside_2 = (X.transpose() * Q2 * X) < 0;

                if(isInside_1) ob1_num++;
                if(isInside_2) ob2_num++;
                if(isInside_1 && isInside_2 ) both_num++;
                
                total_num++;

                EllipsoidSLAM::PointXYZRGB p;
                p.x = x;p.y=y;p.z=z;

                // 默认颜色
                p.r=0;p.g=0;p.b=0;

                if(isInside_1) p.r=255;
                if(isInside_2) p.b=255;
                if(isInside_1&&isInside_2) p.g=128;
                p.size = 5;
                pCloud->push_back(p);
            }
        }

    }

    // 统计
    std::cout << "ob1/ob2/both : " << ob1_num << "/" << ob2_num << "/" << both_num << std::endl;
    std::cout << "Total Num : " << total_num << std::endl;

    // 可视化部分. 显示椭球体、所有数据点, 并按类别标记颜色
    g2o::ellipsoid* pE1 = new g2o::ellipsoid; pE1->fromMinimalVector(ob1.tail(9));
    pE1->setColor(Vector3d(0,0,1));
    g2o::ellipsoid* pE2 = new g2o::ellipsoid; pE2->fromMinimalVector(ob2.tail(9));
    pE2->setColor(Vector3d(1,0,0));
    pMap->ClearEllipsoidsVisual();
    pMap->addEllipsoidVisual(pE1);
    pMap->addEllipsoidVisual(pE2);

    pMap->clearPointCloud();
    pMap->addPointCloud(pCloud);

    // 显示大区域
    Vector9d area_vec;
    double x_center = (x_min+x_max)/2;
    double y_center = (y_min+y_max)/2;
    double z_center = (z_min+z_max)/2;
    double x_halfSize = (x_max-x_min)/2;
    double y_halfSize = (y_max-y_min)/2;
    double z_halfSize = (z_max-z_min)/2;
    area_vec << x_center,y_center,z_center,0,0,0,x_halfSize,y_halfSize,z_halfSize;
    g2o::ellipsoid* pE_Area = new g2o::ellipsoid; pE_Area->fromMinimalVector(area_vec);
    pE_Area->setColor(Vector3d(0,1,0));
    pMap->addEllipsoidVisual(pE_Area);

    // 输出结果.
    double IoU = both_num / double(ob1_num+ob2_num);

    //std::cout << "Push any key to continue ... " << std::endl;
    //getchar();

    return IoU;

}

double normalizeAngle(double angle)
{
    double angle_360 = atan2(sin(angle), cos(angle));   // -pi ~ pi

    double angle_180;   // 0 ~ pi
    if( angle_360 < 0 ) angle_180 = angle_360 + M_PI;
    else angle_180 = angle_360;
    return angle_180;
}

// ob1: ref
// ob2: est
COMPARE_RESULT compareObjectWithVector(VectorXd &ob1, VectorXd &ob2)
{
    // 综合评价: 直接比较IoU
    

    // 位置: 原点差距
    Vector3d ob1_ori = ob1.block(1,0,3,1);
    Vector3d ob2_ori = ob2.block(1,0,3,1);
    double origin_distance = (ob1_ori - ob2_ori).norm();

    // 原始IoU 
    double IoU_ori = MonteCarloIoU(ob1, ob2);

    // 放到原点比较IoU
    VectorXd ob1_atOrigin = ob1;
    VectorXd ob2_atOrigin = ob2;
    ob1_atOrigin.block(1,0,3,1) = Vector3d(0,0,0);
    ob2_atOrigin.block(1,0,3,1) = Vector3d(0,0,0);
    double IoU_aligned = MonteCarloIoU(ob1_atOrigin, ob2_atOrigin);

    // yaw angle compare
    double yaw1 = ob1[6]; yaw1=normalizeAngle(yaw1);
    double yaw2 = ob2[6]; yaw2=normalizeAngle(yaw2);    
    double dis_yaw = std::abs(yaw1 - yaw2);  // [ 0, 180 deg]
    double mini_dis_yaw = MIN(dis_yaw, std::abs(M_PI-dis_yaw));  // [ 0, 90deg ]

    double yaw_deg = mini_dis_yaw/M_PI*180; // 单位 deg
    
    COMPARE_RESULT output;
    output.instanceID = int(ob2[0]);

    output.dis_trans = origin_distance;
    output.dis_yaw = yaw_deg;
    output.IoU = IoU_ori;
    output.IoU_aligned = IoU_aligned;

    return output;

}

int main(int argc,char* argv[]) {

    if( argc != 3 && argc !=4)
    {
        std::cout << "usage: " << argv[0] << " path_to_ref path_to_estimate [path_to_settings]" << std::endl;
        return 1;
    }

    // 初始化 可视化系统
    pSLAM = new EllipsoidSLAM::System(argv[3]);
    pMap = pSLAM->getMap();

    // obj : instance x y z r p yaw a b c
    MatrixXd refObjMat = readDataFromFile(argv[1]);
    MatrixXd estObjMat = readDataFromFile(argv[2]);

    std::cout << "refObjMat: " << std::endl << refObjMat << std::endl;
    std::cout << "estObjMat: " << std::endl << estObjMat << std::endl;

    int refNum = refObjMat.rows();
    int estNum_ori = estObjMat.rows();
    int estNum = estNum_ori;

    std::map<int,COMPARE_RESULT> results;

    std::set<int> list_tooBig;
    for(int i=0; i<estNum_ori; i++)
    {
        // 计算单个物体差别, 再输出整体平均差别.
        VectorXd estOb = estObjMat.row(i);
        int instance = int(estOb[0]);

        // 过滤过大的估计.
        if( estOb[7] > 1.5 || estOb[8] > 1.5 || estOb[9] > 1.5 )   
        {
            list_tooBig.insert(instance);
            estNum--;
            continue;
        }

        // 根据id 寻找到estObj;
        VectorXd refOb = refObjMat.row(instance).head(10);      // 去掉最后一个没用的id
        int instanceRef = int(refOb[0]);

        // 这里假设ref文件按序列排列的
        assert(instance == instanceRef && "ref file should be sorted as instance id." );

        // 开始比较各项指标。
        COMPARE_RESULT result = compareObjectWithVector(refOb, estOb);

        results.insert(make_pair(result.instanceID, result));
    }

    // -------------------- 综合评估过程 -----------------------

    // 计算整体值.    // TODO 写整体计算，然后调试.
    double aver_IoU = 0;  // 原始IoU
    double aver_IoU_aligned = 0;   // 对齐中心后的IoU
    double aver_dis_trans = 0;
    double aver_dis_yaw = 0;

    // 计算中位数
    std::vector<double> vector_iou;
    std::vector<double> vector_iou_aligned;
    std::vector<double> vector_dis_trans;
    std::vector<double> vector_dis_yaw;
    for( auto iter: results)
    {
        aver_IoU += iter.second.IoU;
        aver_IoU_aligned += iter.second.IoU_aligned;
        aver_dis_trans += iter.second.dis_trans;
        aver_dis_yaw += iter.second.dis_yaw;        

        vector_iou.push_back(iter.second.IoU);
        vector_iou_aligned.push_back(iter.second.IoU_aligned);
        vector_dis_trans.push_back(iter.second.dis_trans);
        vector_dis_yaw.push_back(iter.second.dis_yaw);
        
    }
    aver_IoU/=estNum;
    aver_IoU_aligned/=estNum;
    aver_dis_trans/=estNum;
    aver_dis_yaw/=estNum;

    sort(vector_iou.begin(), vector_iou.end());
    sort(vector_iou_aligned.begin(), vector_iou_aligned.end());
    sort(vector_dis_trans.begin(), vector_dis_trans.end());
    sort(vector_dis_yaw.begin(), vector_dis_yaw.end());
    double mid_iou = vector_iou[estNum/2];
    double mid_iou_aligned = vector_iou_aligned[estNum/2];
    double mid_dis_trans = vector_dis_trans[estNum/2];
    double mid_dis_yaw = vector_dis_yaw[estNum/2];

    // 输出结果.
    string path_output("./objects_compare.txt");
    ofstream fio(path_output.c_str());
    fio << " ----------- AVERAGE ---------" << endl;
    fio << "aver_IoU: " << aver_IoU << endl;
    fio << "aver_IoU_aligned: " << aver_IoU_aligned << endl;
    fio << "aver_dis_trans: " << aver_dis_trans << endl;
    fio << "aver_dis_yaw: " << aver_dis_yaw << endl;
    
    fio << " ----------- Mid Value ---------" << endl;
    fio << "mid_iou: " << mid_iou << endl;
    fio << "mid_iou_aligned: " << mid_iou_aligned << endl;
    fio << "mid_dis_trans: " << mid_dis_trans << endl;
    fio << "mid_dis_yaw: " << mid_dis_yaw << endl;

    fio << " ----------- SINGLE OBJECTS COMPARE ---------" << endl;
    fio << "instanceID\t" << "IoU\t" << "IoU_aligned\t" << "dis_trans\t" << "dis_yaw\t" << endl;
    for(int n=0; n<refNum; n++){
        auto iter = results.find(n);
        if(iter != results.end())
        {
            fio << iter->first << "\t" << iter->second.IoU << "\t" << iter->second.IoU_aligned << "\t" << iter->second.dis_trans << "\t" << iter->second.dis_yaw << endl;
        }
        else
        {
            if( list_tooBig.find(n) == list_tooBig.end() )
                // 没有检测.
                fio << n << "\t" << "Empty." << endl;
            else
                fio << n << "\t" << "Too Big Jump." << endl;
        }
        
    }
    fio.close();

    cout << "Save result to " << path_output << endl;

    return 0;

}