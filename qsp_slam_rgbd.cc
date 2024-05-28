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


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>

#include <opencv2/core/core.hpp>

#include "System.h"

#include "src/config/Config.h"

#include "include/core/Ellipsoid.h"
#include "src/tum_rgbd/io.h"

#include <src/dense_builder/builder.h>

#include "include/utils/file_operate.h"

#include <pangolin/pangolin.h>

#include <sys/resource.h>
#include <iostream>

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudPCL;

/**
 * @brief 加载图像
 * 
 * @param[in] path_to_association     关联文件的访问路径
 * @param[out] vstrImageFilenamesRGB     彩色图像路径序列
 * @param[out] vstrImageFilenamesD       深度图像路径序列
 * @param[out] vTimestamps               时间戳
 */

void LoadImages(const string &path_to_association, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);


bool checkEmpty(const vector<string> str_vec, std::string str1);
bool checkEqual(int a, int b, string str1);

void ReconstrucScenePointCloud(ORB_SLAM2::System& SLAM, 
    cv::FileStorage& fSettings, Builder* mp_Builder, string dataset_path,
    vector<string>& rgbFilenames, vector<string>& depthFilenames,
    Trajectory& gtTrajVO);

int main(int argc, char **argv)
{
    if(argc != 2)
    {
        cerr << endl << "Usage: ./qsp_slam_rgbd path_to_settings" << endl;
        return 1;
    }
    
    std::string settings_path = std::string(argv[1]);
    std::cout << "path_to_settings = " << settings_path << std::endl;

    cv::FileStorage fSettings(settings_path, cv::FileStorage::READ);

    std::cout << "Check Error" << std::endl;

    std::string vocabulary_path = fSettings["Vocabulary.Path"];
    std::string dataset_path = fSettings["Dataset.Path.Root"];
    std::string save_map_dir = fSettings["Dataset.Path.Save"];
    std::string associate_file = fSettings["Dataset.Path.AssociateFile"];
    std::string strAssociationFilename = dataset_path + associate_file;

    std::string pcd_filepath = fSettings["Dataset.Path.MapPcd"];

    if(pcd_filepath.size()>0)
    {
        if(pcd_filepath[0]=='.')    // 相对路径
            pcd_filepath = dataset_path + "/" + pcd_filepath.substr(2);
    }

    string save_result_dir = dataset_path + "/result/";
    string strDetectionDir = dataset_path + "/bbox/";

    std::cout << "dataset_path = " << dataset_path << std::endl;
    std::cout << "save_map_dir = " << save_map_dir << std::endl;
    std::cout << "save_result_dir = " << save_result_dir << std::endl;
    std::cout << "strDetectionDir = " << strDetectionDir << std::endl;

    assert(CreateDirIfNotExist(save_map_dir));
    assert(CreateDirIfNotExist(save_result_dir));
    assert(CreateDirIfNotExist(strDetectionDir));
    
    auto msensor = ORB_SLAM2::System::MONOCULAR;
    
    if(fSettings["System.mode"].isNamed()) {
        string system_mode = fSettings["System.mode"];
        if (system_mode=="RGBD") {
            msensor = ORB_SLAM2::System::RGBD;
        }
    }

    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;

    // if(fSettings["CoutToFile"].isNamed()) {
    //     ofstream fout("logs/log.txt"); //文件输出流对象
    //     streambuf* pOld =cout.rdbuf(fout.rdbuf());
    // }

    std::cout << "- settings file: " << settings_path << std::endl;
    std::cout << "- dataset_path: " << dataset_path << std::endl;
    std::cout << "- strDetectionDir: " << strDetectionDir << std::endl;
    string dataset_type = fSettings["Dataset.Type"];
    std::cout << "- dataset_type : " << dataset_type << std::endl;

    //! 加载数据集数据
    TUMRGBD::Dataset loader;
    loader.loadDataset(dataset_path);
    loader.loadDetectionDir(strDetectionDir);
 
    std::cout << "Finish loadDetectionDir" << std::endl;

    if(loader.empty())
    {
        std::cout << "Fail to load the dataset. Please check the path." << std::endl;
        abort();
    }


    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    std::vector<std::string> vstrImageFilenamesRGB_all(vstrImageFilenamesRGB);
    std::vector<std::string> vstrImageFilenamesD_all(vstrImageFilenamesD);
    std::vector<double> vTimestamps_all(vTimestamps);

    int nImages = vstrImageFilenamesRGB.size();

    if (checkEmpty(vstrImageFilenamesRGB, "\nNo images found in provided path.\n"))
        return 1;
    else if (checkEqual(int(vstrImageFilenamesD.size()), int(vstrImageFilenamesRGB.size()), \
            "\nDifferent number of images for rgb and depth.\n"))
        return 1;

    // Create SLAM system. 
    // It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(vocabulary_path, settings_path, dataset_path, msensor);

    //! 显示内存使用情况
    struct rusage rusage; long long memory_init, memory_last, memory_curr;
    if (getrusage(RUSAGE_SELF, &rusage) == 0) {
        memory_init = rusage.ru_maxrss;
        memory_last = memory_init;
        std::cout << "\n Memory usage: " << (double)memory_init / 1024. << " MB" << std::endl;
    } else {
        std::cerr << "Failed to get memory usage." << std::endl;
    }

    //! set groundplane mannually
    cv::Mat paramMat = Config::GetMat("Plane.Groundplane.param");
    if(!paramMat.empty()){
        Eigen::Vector4d plane_param = Converter::toVector4d(paramMat);
        // cout << "plane_param = " << plane_param.matrix() << endl;
        SLAM.getTracker()->SetGroundPlaneMannually(plane_param);
    }

    //! 设置跳步运行
    int skip_num = Config::Get<int>("Running.skip_num");
    double total_ratio = Config::Get<double>("Running.total_ratio");
    cout << "nImages = " << nImages << std::endl;
    int total_num = std::min(int(total_ratio * (double)nImages + 1), nImages);
    skip_num = std::max(1, skip_num);
    std::cout << "total_num = " << total_num << ", skip_num = " << skip_num << std::endl;

    int new_index = 0, raw_index = 0;
    int index_num = total_num / skip_num + 1;
    vector<int> indexes(index_num);
    
    for(; raw_index < total_num; new_index+=1, raw_index+=skip_num) {
        vstrImageFilenamesRGB[new_index] = vstrImageFilenamesRGB[raw_index];
        vstrImageFilenamesD[new_index] = vstrImageFilenamesD[raw_index];
        vTimestamps[new_index] = vTimestamps[raw_index];
        indexes[new_index] = raw_index;
    }

    nImages = new_index;
    vstrImageFilenamesRGB.resize(nImages);
    vstrImageFilenamesD.resize(nImages);
    vTimestamps.resize(nImages);

    //! 这里为了能够让SLAM系统可以通过创建的 Frame ID 获取到正确的图像文件
    SLAM.SetImageNames(vstrImageFilenamesRGB);
    

    // if(!(Config::Get<int>("Dataset.UseGroundtruth") > 0))
    // {
    //     string odom_path = dataset_path+"/CameraTrajectoryVO.txt";
    //     string asso_path = dataset_path+"/associateOdom.txt";
    //     std::cout << "Load odometry from : " << odom_path << std::endl;
    //     std::cout << "Associate file : " << asso_path << std::endl;

    //     if(!loader.SetOdometryAndLoadAssociate(odom_path, asso_path, false)){
    //         std::cout << "Load odometry failed, please check." << std::endl;
    //         return 0;
    //     }
    // }
    // else
    //     std::cout << "Load [Groundtruth] as Odometry." << std::endl;

    //! 获取轨迹真值数据，并放入图像中
    auto p_gtMat = loader.GetGroundtruthTrajectory_ptr(); // 该方法读取的似乎是 Odometry???
    std::cout << "p_gtMat->rows() = " << p_gtMat->rows() << std::endl;
    Trajectory gtTrajVO;
    gtTrajVO.resize(p_gtMat->rows());

    // g2o::SE3Quat init_pose, init_pose_inv, random_pose;
    // init_pose.fromVector(p_gtMat->row(0).tail(7));
    // init_pose_inv = init_pose.inverse();

    for (int i = 0; i < p_gtMat->rows(); i++) {
        SE3QuatWithStamp* pGtSE3T = new SE3QuatWithStamp();
        VectorXd gtPose = p_gtMat->row(i);
        pGtSE3T->pose.fromVector(gtPose.tail(7));
        // pGtSE3T->pose^=init_pose_inv;
        pGtSE3T->timestamp = gtPose[0];
        gtTrajVO[i] = pGtSE3T;
    }

    SLAM.getMap()->addOneTrajectory(gtTrajVO, "Groundtruth");

    //! 处理场景点云真值，如果有，直接进入真值标注环节
    //! 否则使用 traj_gt 和 depth,rgb 进行场景点云重建

    //! 添加点云真值
    bool need_bg_pointcloud = Config::Get<int>("QSPSLAM.NeedDatasetPointCloud");;

    if (need_bg_pointcloud) {
        PointCloudPCL::Ptr pCloudPCL(new PointCloudPCL);
        bool valid_pcd = true;

        std::cout << "pcd_filepath = " << pcd_filepath << std::endl;

        // 读取.pcd文件到 PointCloudPCL 中
        if (pcl::io::loadPCDFile<PointT>(pcd_filepath, *pCloudPCL) == -1) {
            cout << "Couldn't read pcd file: " << pcd_filepath << std::endl;
            valid_pcd = false;

            Builder* mp_Builder = new Builder();

            ReconstrucScenePointCloud(SLAM, fSettings, mp_Builder, dataset_path,
                vstrImageFilenamesRGB_all, vstrImageFilenamesD_all, gtTrajVO);

            std::cout << "Save pointcloud Map to : " << pcd_filepath << std::endl;
            mp_Builder->saveMap(pcd_filepath);

            if (pcl::io::loadPCDFile<PointT>(pcd_filepath, *pCloudPCL) == -1) {
                cout << "Couldn't read file again: " << pcd_filepath << std::endl;
                valid_pcd = false;
                return -1;
            }
            else{
                valid_pcd = true;
            }
        }
        std::cout << "Finish dataset.pcd generation." << std::endl;

        printMemoryUsage();

        if (valid_pcd) {
            SLAM.getTracker()->LoadPointcloud(pcd_filepath, "Dataset PointCloud");
        }
    }

    // SLAM.getMap()->addOneTrajectory(gtTrajInEst, "AlignedGroundtruth");

    // Vector for tracking time statistics
    vector<float> vTimesTrack(nImages);

    bool frame_by_frame = fSettings["frame_by_frame"].isNamed();

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;

    if(frame_by_frame) {
        std::cout << "*****************************" << std::endl;
        std::cout << "Press [ENTER] to continue ... , [y] to autonomous mode" << std::endl;
        std::cout << "*****************************" << std::endl;
        char key = getchar();
        if (key=='y')
        {
            frame_by_frame = false;
        }
    }

    for(int ni = 0; ni < nImages; ni++)
    {
        int goal_id = indexes[ni];
        loader.SetCurrentID(goal_id);

        Eigen::MatrixXd* p_detMat = loader.getDetectionMatPointer(goal_id);

        // std::cout << "p_detMat = " << p_detMat->matrix() << std::endl;

        // std::cout << "Set current ID to : " << goal_id << std::endl;
        
        std::cout << "\n========================================" << std::endl;
        std::cout << "=> Inputting Image " << ni << "/" << nImages;
        std::cout << "\n========================================" << std::endl;

        std::chrono::steady_clock::time_point t1_read = std::chrono::steady_clock::now();

        //! 读取图像
        imRGB = cv::imread(dataset_path+"/"+vstrImageFilenamesRGB[ni], CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(dataset_path+"/"+vstrImageFilenamesD[ni], CV_LOAD_IMAGE_UNCHANGED);

        // std::cout << "Debug: Read imRGB and imD" <<std::endl;

        std::chrono::steady_clock::time_point t2_read = std::chrono::steady_clock::now();
        double t_read = std::chrono::duration_cast<std::chrono::duration<double> >(t2_read - t1_read).count();

        // cout << " Reading Image costs: " << t_read << "s" << endl;

        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << dataset_path << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        assert(imRGB.type()==16);
        assert(imD.type()==2);

        // std::cout << "Debug: Send into TrackRGBD" <<std::endl;

        if (msensor == ORB_SLAM2::System::RGBD)
            SLAM.TrackRGBD(imRGB, imD, tframe);
        else if(msensor == ORB_SLAM2::System::MONOCULAR)
            SLAM.TrackMonocular(imRGB, tframe);
        else
            printf("暂时不支持这个模式\n");

        // std::cout << "Debug: get out of SLAM.TrackRGBD" << std::endl;

        // FIXME: 如有必要，将内存使用的变化情况存储下来检查一下
        if (getrusage(RUSAGE_SELF, &rusage) == 0) {
            memory_curr = rusage.ru_maxrss;

            double me_ratio = memory_curr / memory_init;
            std::cout << "\nMemory curr/add/init = "
                      << (double)memory_curr / 1024. << " MB /"
                      << (double)(memory_curr - memory_last) / 1024. << "MB /"
                      << (double)memory_init / 1024. << " MB"
                      << std::endl;
            memory_last = memory_curr;
        } else {
            std::cerr << "Failed to get memory usage." << std::endl;
        }

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        cout << " - [ total_frame: " << (double) ttrack  << "s ]" << endl;

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T = 0.0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<size_t>((T- ttrack)*1e6)));
        }

        SLAM.LoadPause();

        if(frame_by_frame) {
            std::cout << "*****************************" << std::endl;
            std::cout << "Press [ENTER] to continue ... , [y] to autonomous mode" << std::endl;
            std::cout << "*****************************" << std::endl;
            char key = getchar();
            if (key=='y')
            {
                frame_by_frame = false;
            }
            else if (key=='e'){
                break;
            }
        }
    }

    SLAM.SaveEntireMap(save_map_dir);

    // if(fSettings["System.output"].isNamed()) {
    //     if(fSettings["System.output_path"].isNamed()) {
    //         traj_path = std::string(fSettings["System.output_path"]) + std::string("/") + std::string(fSettings["System.output"]);
    //     }
    //     else{
    //         traj_path = dataset_path + std::string("/") + fSettings["System.output"];
    //     }
    // }
    
    bool use_index = true;

    SLAM.SaveTrajectoryTUM(dataset_path + "/CameraTrajectory_QSP_times.txt");
    SLAM.SaveTrajectoryTUM(dataset_path + "/CameraTrajectory_QSP_indexes.txt", use_index);

    // SLAM.SaveTrajectoryTwc(dataset_path + "/CameraTrajectory_Twc_QSP_times.txt");
    // SLAM.SaveTrajectoryTwc(dataset_path + "/CameraTrajectory_Twc_QSP_indexes.txt", use_index);


    SLAM.SaveKeyFrameTrajectoryTUM(dataset_path + "/KeyFrameTrajectory_QSP.txt");

    SLAM.SaveEstimatedObjectsForEvo(dataset_path + "/result/objects.txt");
    SLAM.SaveEstimatedTrajForEvo(dataset_path + "/result/traj.txt");

    // Save pointcloud
    bool mbOpenBuilder = Config::Get<int>("Visualization.Builder.Open") == 1;

    // assert(CreateDirIfNotExist(save_result_dir));

    if(mbOpenBuilder)
        SLAM.getTracker()->SavePointCloudMap(save_map_dir + "/map.pcd");


    // cv::waitKey(0);
    // cv::destroyAllWindows();

    // // Stop all threads
    // SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    cout << "Press y to quit." << endl;
    char key;
    key = getchar();

    while(key!='y' && key!='Y'){
        cout << "Press y to quit." << endl;
        key = getchar();
    }

    // SLAM.Shutdown();
    cv::destroyAllWindows();

    cout << "End." << endl;
    return 0;
}

//从关联文件中提取这些需要加载的图像的路径和时间戳
void LoadImages(const string &path_to_association, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    //输入文件流
    ifstream fAssociation;
    //打开关联文件
    fAssociation.open(path_to_association.c_str());
    //一直读取,知道文件结束
    while(!fAssociation.eof())
    {
        string s;
        //读取一行的内容到字符串s中
        getline(fAssociation,s);
        //如果不是空行就可以分析数据了
        if(!s.empty())
        {
            //字符串流
            stringstream ss;
            ss << s;
            //字符串格式:  时间戳 rgb图像路径 时间戳 深度图像路径
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}

bool checkEmpty(const vector<string> str_vec, std::string str1)
{
    if(str_vec.empty()){
        cerr << str1;
        return true;
    }
    else{
        return false;
    }
    
}

bool checkEqual(int a, int b, string str1)
{
    if(a!=b){
        cerr << str1;
        return true;
    }
    else{
        return false;
    }
}

double GetMemoryUsage(){
  struct rusage rusage;
  if (getrusage(RUSAGE_SELF, &rusage) == 0) {
    return (double)rusage.ru_maxrss / 1024.; // MB
  } else {
    return -1.0;
  }
}

void ReconstrucScenePointCloud( ORB_SLAM2::System& SLAM, 
    cv::FileStorage& fSettings, Builder* mp_Builder, string dataset_path,
    vector<string>& rgbFilenames, vector<string>& depthFilenames,
    Trajectory& gtTrajVO)
{
    std::cout << "Reconstruct by gt_traj" << std::endl;

    /**在标注之前，如果没有现成的数据集点云，首先要使用轨迹真值进行点云重建*/

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    Eigen::Matrix3d mCalib;
    mCalib << fx,  0, cx,
                0, fy, cy,
                0,  0,  1;

    double mCamera_scale = fSettings["Camera.scale"];
    mp_Builder->setCameraIntrinsic(mCalib, mCamera_scale);
    double depth_range = Config::ReadValue<double>("EllipsoidExtractor_DEPTH_RANGE");   // Only consider pointcloud within depth_range

    int frame_num = rgbFilenames.size();
    int gt_traj_num = gtTrajVO.size();
    cout << "frame_num = " << frame_num << ", gt_traj_num = " << gt_traj_num << std::endl;
    assert((frame_num <= gt_traj_num) && "frame_num should be equal to gt_traj_num");

    std::cout << std::endl;
    // std::cout << "set frame_num = 100" << std::endl;
    // frame_num = 100;

    for (int i_frame = 0; i_frame < frame_num; i_frame++)
    {
        std::cout << "frame: " << i_frame << "/" << frame_num \
                  << ", memory: " << GetMemoryUsage() << "MB" << std::endl;

        cv::Mat rgb_img = cv::imread(dataset_path+"/"+rgbFilenames[i_frame], CV_LOAD_IMAGE_UNCHANGED);
        cv::Mat depth_img = cv::imread(dataset_path+"/"+depthFilenames[i_frame], CV_LOAD_IMAGE_UNCHANGED);

        cv::imshow("rgb_img", rgb_img);
        // cv::imshow("depth_range", depth_range);
        cv::waitKey(0);

        cv::imshow("depth_img", depth_img);
        // cv::imshow("depth_range", depth_range);
        cv::waitKey(0);

        // Tcw: x,y,z,qx,qy,qz,qw
        Eigen::VectorXd pose = gtTrajVO[i_frame]->pose.toVector();

        mp_Builder->processFrame(rgb_img, depth_img, pose, depth_range);


        mp_Builder->voxelFilter(0.01);   // Down sample threshold; smaller the finer; depend on the hardware.

        // PointCloudPCL::Ptr pCloudPCL = mp_Builder->getMap();
        // auto pCloud = pclToQuadricPointCloudPtr(pCloudPCL);
        // SLAM.getMap()->AddPointCloudList("Builder.GT Global Points", pCloud);
        // delete pCloudPCL;
    }

    std::cout << "Hello  " << std::endl << std::endl;
}
