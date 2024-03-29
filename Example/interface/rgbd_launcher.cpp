/*  This file is created on 2020-4-22 by Ziwei Liao.
*   In order to test the automatic probabilistic data association, based on paper:
*   IROS 2017, MIT.
*/

// Update 2020-6-11
// 对入口做改进： 至此数据集路径由yaml文件给定。

#include "Initializer.h"
#include "core/Geometry.h"
#include "utils/dataprocess_utils.h"
#include "utils/matrix_utils.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Viewer.h"
#include "MapDrawer.h"
#include "Map.h"

#include <thread>
#include <string>

#include "include/core/Ellipsoid.h"
#include "src/tum_rgbd/io.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudPCL;

#include "src/config/Config.h"
#include "func/func.h"

#include<opencv2/core/eigen.hpp>

#include <chrono>

using namespace std;
using namespace Eigen;

// 统计每帧运行情况
struct FrameState
{
    double timestamp;
    double time_cost;  // second
    int obj_num;
};

void OutputFrameStates(const string & file_name, std::vector<FrameState>& vFrameStates)
{
    ofstream out(file_name.c_str());
    int num = vFrameStates.size();
    for(int i=0;i<num;i++)
    {
        FrameState& fs = vFrameStates[i];
        out << fs.timestamp << "\t" << fs.time_cost << "\t" << fs.obj_num << std::endl;
    }

    out.close();
    std::cout << "Save " << num << " frame states to " << file_name << std::endl;
}

int main(int argc,char* argv[]) {
    if( argc != 2)
    {
        std::cout << "usage: " << argv[0] << " path_to_settings" << std::endl;
        return 1;
    }
    string strSettingPath = string(argv[1]);

    ORB_SLAM2::System SLAM(strSettingPath, true);
    SLAM.CloseOptimization();   // close incremental calculation; 
                              // the full optimization will be processed after the last frame.


    //  path_to_dataset [path_to_map]
    string dataset_path = Config::Get<string>("Dataset.Path.Root");
    string dataset_path_map = Config::Get<string>("Dataset.Path.Map");
    if(dataset_path_map.size()>0)
    {
        if(dataset_path_map[0]=='.')    // 相对路径
            dataset_path_map = dataset_path + "/" + dataset_path_map;
    }
    string dataset_path_savedir = dataset_path + "/result/";

    string strDetectionDir = dataset_path + "/bbox/";

    std::cout << "- settings file: " << strSettingPath << std::endl;
    std::cout << "- dataset_path: " << dataset_path << std::endl;
    std::cout << "- strDetectionDir: " << strDetectionDir << std::endl;
    string dataset_type = Config::Get<string>("Dataset.Type");
    std::cout << "- dataset_type : " << dataset_type << std::endl;

    TUMRGBD::Dataset loader;
    loader.loadDataset(dataset_path);
    loader.loadDetectionDir(strDetectionDir);

    if(!(Config::Get<int>("Dataset.UseGroundtruth") > 0))
    {
        string odom_path = dataset_path+"/CameraTrajectoryVO.txt";
        string asso_path = dataset_path+"/associateOdom.txt";
        std::cout << "Load odometry from : " << odom_path << std::endl;
        std::cout << "Associate file : " << asso_path << std::endl;
        if(!loader.SetOdometryAndLoadAssociate(odom_path, asso_path, false)){
            std::cout << "Load odometry failed, please check." << std::endl;
            return 0;
        }
    }
    else
        std::cout << "Load [Groundtruth] as Odometry." << std::endl;

    if(loader.empty())
    {
        std::cout << "Fail to load the dataset. Please check the path." << std::endl;
        abort();
    }
    
    // set groundplane mannually
    cv::Mat paramMat = Config::GetMat("Plane.Groundplane.param");
    if(!paramMat.empty())
    {
        Eigen::Vector4d plane_param;
        cv::cv2eigen(paramMat, plane_param);
        SLAM.getTracker()->SetGroundPlaneMannually(plane_param);
    }

    
    // 从开始时间之后的第一帧开始
    double start_time_stamp = Config::Get<double>("Dataset.Timestamp.Start");
    double end_time_stamp = Config::Get<double>("Dataset.Timestamp.End");
    bool config_timestamp_thresh = (start_time_stamp > 0 || end_time_stamp > 0);

    std::cout << "start_time_stamp = " << start_time_stamp 
              << ", end_time_stamp = " << end_time_stamp << std::endl;

    if(config_timestamp_thresh)
    {
        std::cout << "[ start_time_stamp : " << start_time_stamp << std::endl;
        std::cout << "[ end_time_stamp : " << end_time_stamp << std::endl;

        for(int goal_id = 0; goal_id<loader.getTotalNum(); goal_id++){
            double current_time_stamp = loader.GetTimestamp(goal_id);
            if(current_time_stamp>start_time_stamp) 
            {
                loader.SetCurrentID(goal_id);
                std::cout << "Set current ID to : " << goal_id << std::endl;
                break;
            }
        }
    }

    std::chrono::system_clock::time_point a = std::chrono::system_clock::now();
    std::chrono::system_clock::time_point b = std::chrono::system_clock::now();
    cv::Mat rgb,depth;
    VectorXd pose;

    bool config_one_frame_one_key = true;

    int jump_frame = Config::Get<int>("Dataset.Jumpframe");
    bool bJumpFrame = (jump_frame > 0);
    int last_frame_id = 0;

    std::vector<FrameState> vFrameStates;
    while(!loader.empty())
    {
        bool valid = loader.readFrame(rgb,depth,pose);
        int current_id = loader.getCurrentID();

        Eigen::MatrixXd detMat = loader.getDetectionMat();
        double timestamp = loader.GetCurrentTimestamp();

        if( config_timestamp_thresh && timestamp > end_time_stamp) break;

        std::cout << "Frame Valid = " << valid << std::endl;

        if(valid)
        {
            // jump frame
            // std::cout 
            if(bJumpFrame){
                if((current_id - last_frame_id) < jump_frame)
                {
                    continue;
                }
                last_frame_id = current_id;
            }



            if(config_one_frame_one_key)
            {
                std::cout << "*****************************" << std::endl;
                std::cout << "Press [ENTER] to continue ... [y] for automatic mode. " << std::endl;
                std::cout << "*****************************" << std::endl;
                char key = getchar();
                if(key == 'y') config_one_frame_one_key = false;
            }

            {
                // Maintain designated frequency of 5 Hz (200 ms per frame)
                a = std::chrono::system_clock::now();
                std::chrono::duration<double, std::milli> work_time = a - b;

                if (work_time.count() < 100.0)
                {
                    std::chrono::duration<double, std::milli> delta_ms(200.0 - work_time.count());
                    auto delta_ms_duration = std::chrono::duration_cast<std::chrono::milliseconds>(delta_ms);
                    std::this_thread::sleep_for(std::chrono::milliseconds(delta_ms_duration.count()));
                }

                b = std::chrono::system_clock::now();
                std::chrono::duration<double, std::milli> sleep_time = b - a;
            }

            // 在此统计每帧时间
            FrameState fs;
            clock_t time_frameStart = clock();
            SLAM.TrackWithObjects(timestamp, pose, detMat, depth, rgb, false);        // Process frame.    
            clock_t time_frameEnd = clock();

            std::cout << std::endl;

            fs.time_cost = (double)(time_frameEnd - time_frameStart) / CLOCKS_PER_SEC;
            fs.timestamp = timestamp;
            fs.obj_num = detMat.rows();
            vFrameStates.push_back(fs);

            // debug : 只处理两帧
            // if(current_id > 3) break;
        }

        std::cout << " -> " << loader.getCurrentID() << "/" << loader.getTotalNum() << std::endl;
    }

    std::cout << "Finished all data." << std::endl;

    std::cout << "Begin Optimization ... " << std::endl;
    
    // 此处进入交互式优化调试
    MatrixXd gtMat = loader.GetGroundtruthTrajectory(); // 该方法读取的似乎是 Odometry???

    double last_debug_odom_weight = 0, debug_odom_weight = 0.1;
    double last_debug_plane_dis_sigma = 0, debug_plane_dis_sigma = 0.1;
    double last_debug_plane_weight = 0, debug_plane_weight = 1;
    while(debug_odom_weight > 0)
    {
        debug_odom_weight = Config::ReadValue<double>("DEBUG.ODOM.WEIGHT");
        debug_plane_dis_sigma = Config::ReadValue<double>("DataAssociation.PlaneError.DisSigma");
        debug_plane_weight = Config::ReadValue<double>("DEBUG.PLANE.WEIGHT");

        bool flag_changed = std::abs(debug_odom_weight - last_debug_odom_weight)>0.01;
        flag_changed |= std::abs(last_debug_plane_dis_sigma - debug_plane_dis_sigma)>0.01;
        flag_changed |= std::abs(last_debug_plane_weight - debug_plane_weight)>0.01;
        if( flag_changed ){
            SLAM.getTracker()->NonparamOptimization();        // Optimize with probabilistic data association
            last_debug_odom_weight = debug_odom_weight;
            last_debug_plane_dis_sigma = debug_plane_dis_sigma;
            last_debug_plane_weight = debug_plane_weight;

            std::cout << std::endl << " ============= Begin Evaluation ============= " << std::endl << std::endl;
            // 获得当前轨迹.
            Trajectory estTraj = SLAM.getMap()->getTrajectoryWithName("OptimizedTrajectory");

            // 与真实轨迹评估输出结果.
            MatrixXd estMat = TrajetoryToMat(estTraj);
            
            Trajectory gtTrajInEst, estTrajSelected; g2o::SE3Quat Tge;
            alignTrajectory(estMat, gtMat, Tge, gtTrajInEst, estTrajSelected);

            // 计算轨迹误差.
            // error1 : 原始odometry 与 Gt 误差.
            double rmse = CalculateRMSE(estTrajSelected, gtTrajInEst);
            // error2 : 当前optimized traj 与 Gt 误差.
            std::cout << " [ RMSE: " << rmse << " m ]" << std::endl;

            // 可视化对齐后的真实轨迹.
            SLAM.getMap()->addOneTrajectory(gtTrajInEst, "AlignedGroundtruth");
            std::cout << "End of this test." << std::endl;

            // save objects
            string output_path(dataset_path_savedir+"./objects.txt");
            SLAM.SaveObjectsToFile(output_path);
            SLAM.getTracker()->SaveObjectHistory(dataset_path_savedir+"./object_history.txt");
            SLAM.SaveTrajectoryTUM(dataset_path_savedir+"./traj.txt");

            // 读取gt点云并可视化. 并且根据 estimate traj 和 gt traj 做矫正.
            if(dataset_path_map.size() > 0){
                const string path_pcd = dataset_path_map;
                std::cout << "Load background pointcloud from " << path_pcd << "..." << std::endl;
                SLAM.getTracker()->LoadPointcloud(path_pcd, "background_world", Tge.inverse());
                std::cout << "ok." << std::endl;
            }

        }
        // 此时直接可视化结果, 若退出将保存该结果.
    }

    // Save frame running-times
    OutputFrameStates("./frame_states.txt", vFrameStates);

    // Save pointcloud
    bool mbOpenBuilder = Config::Get<int>("Visualization.Builder.Open") == 1;
    if(mbOpenBuilder)
        SLAM.getTracker()->SavePointCloudMap(dataset_path_savedir+"./map.pcd");

    cout << "Use Ctrl+C to quit." << endl;
    while(1);

    cout << "End." << endl;
    return 0;
}