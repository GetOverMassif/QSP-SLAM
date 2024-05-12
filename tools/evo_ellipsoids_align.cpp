// Update : 2020-5-27 
// 1) 适应新版本 Ellipsoid::Core.
// 2) 添加轨迹对齐和自动关联.

// Update : 2020-1-4 完善椭球体评估功能
// 评估物体重建和真实位置之间的差值

#include <iostream>
#include <fstream>
#include <Eigen/Core>

#include "include/utils/dataprocess_utils.h"

#include <core/Ellipsoid.h>
#include <core/Geometry.h>
#include <core/System.h>
#include <core/Map.h>

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

bool compare_pair_int_double(std::pair<int,double>&p1, std::pair<int,double>&p2)
{
    return p1.second < p2.second;
}

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

    cout << "Begin MCL Sampling, resolution : " << resolution << endl;
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

MatrixXd GenerateSelectedGtMat(MatrixXd &estMat, MatrixXd &gtMat)
{
    MatrixXd gtMatSelected; gtMatSelected.resize(0, gtMat.cols());
    int estNum = estMat.rows();
    int gtNum = gtMat.rows();
    for(int i=0;i<estNum;i++)
    {
        VectorXd estPose = estMat.row(i);
        double timestamp = estPose[0];

        // 寻找对应的gt
        bool bFindGT = false;
        VectorXd pose_gt;
        for( int n=0;n<gtNum;n++)
        {
            VectorXd gtPose = gtMat.row(n);
            double timestampGT = gtPose[0];
            if( std::abs(timestamp - timestampGT) < 0.001 )
            {
                bFindGT= true;
                pose_gt = gtPose;
                break;
            }
        }

        if(!bFindGT) {
            std::cout << "[ERROR in ODOM] No corresponding gt found. timestamp: " << timestamp << std::endl;
            break;  // 未找到对应的gt, 理应报错.
        }

        addVecToMatrix(gtMatSelected, pose_gt);

    }

    return gtMatSelected;
}

// 估计一个刚体变换使得误差最小. 并输出匹配的变换.
void alignTrajectory(MatrixXd& estMat, MatrixXd& gtMat, g2o::SE3Quat& transform)
{
    // 应该是一个闭式解.
    // 生成两个匹配的 Mat; 用 est 去寻找 gt ( 即认为 est <= gt )

    MatrixXd estPointMat = estMat.block(0,1,estMat.rows(),3);
    MatrixXd gtMatSelected = GenerateSelectedGtMat(estMat, gtMat);
    MatrixXd gtPointMat = gtMatSelected.block(0,1,gtMatSelected.rows(),3);

    // std::cout << "gtPointMat : " << std::endl << gtPointMat << std::endl;
    // std::cout << "estPointMat : " << std::endl << estPointMat << std::endl;

    // 开始求解 : 要求输入的点是一列一个.
    MatrixXd result = Eigen::umeyama(estPointMat.transpose(), gtPointMat.transpose(), false);
    std::cout << " ----- Umeyama result ---- " << std::endl;
    std::cout << result << std::endl;

    g2o::SE3Quat trans(result.topLeftCorner(3,3), result.topRightCorner(3,1));

    // // 将gt点变换到 est 坐标系下.
    // Trajectory tGt;
    // int num = gtMatSelected.rows(); // 这里只可视化所有对应帧得了.
    // for(int i=0;i<num;i++)
    // {
    //     VectorXd gtPose = gtMatSelected.row(i);
    //     SE3QuatWithStamp* pGtSE3T = new SE3QuatWithStamp();
    //     pGtSE3T->pose.fromVector(gtPose.tail(7));
    //     pGtSE3T->timestamp = gtPose[0];

    //     // 应用变换
    //     pGtSE3T->pose = trans.inverse() * pGtSE3T->pose;

    //     tGt.push_back(pGtSE3T);
    // }
    transform = trans;

    // 返回RMSE误差
    return;
}

// edit
// bool deleteElement(std::vector<Vector4d, Eigen::aligned_allocator<Vector3d>>& vec, int id)
bool deleteElement(std::vector<Vector4d, Eigen::aligned_allocator<Vector4d>>& vec, int id)
{
    for(auto iter = vec.begin(); iter!=vec.end(); iter++)
    {
        double id_d = (*iter)[0];
        if((round(id_d)==id))
        {
            // 删除
            vec.erase(iter);
            return true;
        }
    }
    return false;
}


std::vector<int> AlignEllipsoids(MatrixXd& estObjMat, MatrixXd& refObjMat)
{
    // 都生成x,y,z Vec
    MatrixXd estPointMat = estObjMat.block(0,1,estObjMat.rows(),3);
    MatrixXd refPointMat = refObjMat.block(0,1,refObjMat.rows(),3);

    cout << "estPointMat : \n " << estPointMat << endl;
    cout << "refPointMat : \n " << refPointMat << endl;

    // 将 refPoint 放到一个 vector中，以实现动态删除
    int ref_num = refPointMat.rows();
    // edit
    // std::vector<Vector4d, Eigen::aligned_allocator<Vector3d>> vRefPointsWithID;
    std::vector<Vector4d, Eigen::aligned_allocator<Vector4d>> vRefPointsWithID;
    for( int i=0;i<ref_num;i++)
    {
        VectorXd ref_vec = refPointMat.row(i);
        Vector3d center_ref = ref_vec;
        Vector4d center_ref_withID;
        center_ref_withID << i, center_ref;

        vRefPointsWithID.push_back(center_ref_withID);
    }

    // Easy版本: 通过椭球体之间xyz距离做 alignment.
    int est_num = estPointMat.rows();
    std::vector<int> associations; associations.resize(est_num);
    fill(associations.begin(), associations.end(), -1);
    for( int i=0; i<est_num; i++)
    {
        VectorXd est_vec = estObjMat.row(i);
        Vector3d center_est = est_vec.block(1,0,3,1);

        // 寻找距离最小的拿走.
        int ref_num_left = vRefPointsWithID.size();

        if(ref_num_left==0) break;  // 已经没有剩余的ref objects了, 结束.

        std::vector<std::pair<int,double>> vIdDis;
        for(int m=0;m<ref_num_left;m++)
        {
            double dis = (center_est - vRefPointsWithID[m].tail(3)).norm();
            int id = round(vRefPointsWithID[m][0]);
            vIdDis.push_back(std::make_pair(id, dis));
        }
        std::sort(vIdDis.begin(),vIdDis.end(),compare_pair_int_double);
        
        double min_dis = vIdDis[0].second;
        if(min_dis < 1.0)   // 要求 1m 以内
        {
            int id = vIdDis[0].first;
            associations[i] = vIdDis[0].first;
            // 动态删除
            bool result = deleteElement(vRefPointsWithID, id);
            if(!result) 
            {
                std::cout << " Possible BUG: delte element fails." << std::endl;
            }
        }
    }

    return associations;
}

// objMat : id x y z r p y a b c
MatrixXd transformObjMat(MatrixXd& objMat, g2o::SE3Quat& T)
{
    int num = objMat.rows();
    MatrixXd objMatTrans; objMatTrans.resize(0, 10);
    for( int i=0;i<num;i++)
    {
        VectorXd vec = objMat.row(i);
        g2o::ellipsoid e; e.fromMinimalVector(vec.tail(9));
        g2o::ellipsoid e_trans = e.transform_from(T);
        Vector9d e_miniVec = e_trans.toMinimalVector();
        
        VectorXd vec_trans; vec_trans.resize(9+1);
        vec_trans << vec[0], e_miniVec;
        
        // 变换到世界系
        addVecToMatrix(objMatTrans, vec_trans);
    }
    return objMatTrans;
}

int main(int argc,char* argv[]) {

    if( argc != 6)
    {
        std::cout << "usage: " << argv[0] << " path_to_ref path_to_estimate gt_traj_path est_traj_path path_to_settings" << std::endl;
        return 1;
    }

    // 初始化 可视化系统
    pSLAM = new EllipsoidSLAM::System(argv[5]);
    pMap = pSLAM->getMap();

    // obj : instance x y z r p yaw a b c
    MatrixXd refObjMat = readDataFromFile(argv[1]);
    MatrixXd estObjMat = readDataFromFile(argv[2]);

    MatrixXd gtTrajMat = readDataFromFile(argv[3]);
    MatrixXd estTrajMat = readDataFromFile(argv[4]);

    std::cout << "refObjMat: " << std::endl << refObjMat << std::endl;
    std::cout << "estObjMat: " << std::endl << estObjMat << std::endl;
    
    std::cout << "gtTrajNum : " << gtTrajMat.rows() << std::endl;
    std::cout << "estTrajNum : " << estTrajMat.rows() << std::endl;

    int refNum = refObjMat.rows();
    int estNum_ori = estObjMat.rows();
    int estNum = estNum_ori;

    // 关联物体
    g2o::SE3Quat Tre;
    alignTrajectory(estTrajMat, gtTrajMat, Tre);
    std::cout << "Umeda Align Result [x y z roll pitch yaw] : " << std::endl;
    std::cout << Tre.toMinimalVector().transpose() << std::endl;

    MatrixXd estObjMatInRef = transformObjMat(estObjMat, Tre);
    std::vector<int> vAlignEstToRef = AlignEllipsoids(estObjMatInRef, refObjMat);

    std::map<int,COMPARE_RESULT> results;

    std::set<int> list_noAlign;
    for(int i=0; i<estNum_ori; i++)
    {
        // 计算单个物体差别, 再输出整体平均差别.
        VectorXd estOb = estObjMatInRef.row(i);
        int ref_id = vAlignEstToRef[i];
        if(ref_id == -1)
        {
            // no align
            list_noAlign.insert(i);
            estNum--;
            continue;
        }

        // // 过滤过大的估计.
        // if( estOb[7] > 1.5 || estOb[8] > 1.5 || estOb[9] > 1.5 )   
        // {
        //     list_tooBig.insert(instance);
        //     estNum--;
        //     continue;
        // }

        // 根据id 寻找到estObj;
        VectorXd refOb = refObjMat.row(ref_id).head(10);      // 去掉最后一个没用的id

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

    if(estNum>0){
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
                if( list_noAlign.find(n) == list_noAlign.end() )
                    // 没有检测.
                    fio << n << "\t" << "Empty." << endl;
                else
                    fio << n << "\t" << "No align." << endl;
            }
            
        }
        fio.close();

        cout << "Save result to " << path_output << endl;
    }
    else 
        cout << " Est num only : " << estNum << endl;

    return 0;

}