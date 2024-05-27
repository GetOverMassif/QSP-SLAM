// created on 2020-6-18
// 封装接口.

#include "src/evo/evo.h"

using namespace evo;

inline bool is_fileExist(const string& name)
{
    ifstream f(name.c_str());
    return f.good();
}

string output_check(const string& path)
{
    bool exist = is_fileExist(path);
    string exist_str;
    if(exist)
        exist_str = "[YES]";
    else 
        exist_str = "[NO]";
    string output = path + exist_str;
    return output;
}

/**
 * 需要准备：轨迹真值、物体真值、轨迹估计值、物体估计值
*/

int main(int argc,char* argv[]) {
    if(argc != 2)
    {
        cerr << endl << "Usage: ./qsp_slam_rgbd path_to_vocabulary path_to_settings path_to_sequence path_to_association path_to_saved_trajectory" << endl;
        return 1;
    }

    std::string settings_path = std::string(argv[1]);
    cv::FileStorage fSettings(settings_path, cv::FileStorage::READ);

    std::string vocabulary_path = fSettings["Vocabulary.Path"];
    std::string dataset_path = fSettings["Dataset.Path.Root"];
    std::string save_map_dir = fSettings["Dataset.Path.Save"];

    std::string obj_gt_file = fSettings["Dataset.Path.ObjectGT"];
    std::string obj_ground_gt_file = fSettings["Dataset.Path.ObjectGroundGT"];
    std::string traj_gt_file = fSettings["Dataset.Path.TrajectoryGT"];


    std::string obj_est_file = fSettings["Result.ObjectEstimate"];
    std::string traj_est_file = fSettings["Result.TrajectoryEstimate"];


    string dataset_path_savedir = dataset_path + "/result/";
    string strDetectionDir = dataset_path + "/bbox/";

    std::cout << "- settings file: " << settings_path << std::endl;
    std::cout << "- dataset_path: " << dataset_path << std::endl;
    std::cout << "- strDetectionDir: " << strDetectionDir << std::endl;
    string dataset_type = fSettings["Dataset.Type"];
    std::cout << "- dataset_type : " << dataset_type << std::endl;

    const string path_refObjMat = dataset_path + obj_ground_gt_file; // 注意 _ground 表示仅包括位于地面的物体!
    const string path_estObjMat = dataset_path + obj_est_file;
    const string path_gtTrajMat = dataset_path + traj_gt_file;
    const string path_estTrajMat = dataset_path + traj_est_file;


    // std::string strAssociationFilename = dataset_path + "/associate.txt";

    auto msensor = ORB_SLAM2::System::RGBD;
    pSLAM = new ORB_SLAM2::System(vocabulary_path, settings_path, dataset_path, msensor);
    pMap = pSLAM->getMap();

    string pcd_filepath = Config::Get<string>("Dataset.Path.MapPcd");


    //! set groundplane mannually
    cv::Mat paramMat = Config::GetMat("Plane.Groundplane.param");
    if(!paramMat.empty()){
        Eigen::Vector4d plane_param = Converter::toVector4d(paramMat);
        // cout << "plane_param = " << plane_param.matrix() << endl;
        pSLAM->getTracker()->SetGroundPlaneMannually(plane_param);
    }

    if(pcd_filepath.size()>0)
    {
        if(pcd_filepath[0]=='.')    // 相对路径
            pcd_filepath = dataset_path + "/" + pcd_filepath;
    }


    std::cout << "Load background pointcloud from " << pcd_filepath << "..." << std::endl;

    pSLAM->getTracker()->LoadPointcloud(pcd_filepath, "background_world");
    std::cout << "ok." << std::endl;

    
    MatrixXd refObjMat = readDataFromFile(path_refObjMat.c_str());

    if(refObjMat.rows()==0){    // 如果不存在ground版本则读原始gt
        string path_gt_objects = dataset_path + obj_gt_file; 
        refObjMat = readDataFromFile(path_gt_objects.c_str());
    }

    std::cout << "\nRead Est Obj from : " << path_estObjMat << std::endl;

    /**
     * 读取轨迹真值、是否使用轨迹真值
    */

    // bool bUseGtTraj = false;

    // if(argc >= 5)
    //     bUseGtTraj = (stoi(string(argv[4])) == 1);

    // string path_estTrajMat;

    // if(!bUseGtTraj)
    //     path_estTrajMat = dataset_path + "/result/traj.txt";
    // else
    //     path_estTrajMat = path_gtTrajMat;


    /**
    std::cout << "path_gtTrajMat = " << path_gtTrajMat << std::endl;
    std::cout << "path_estTrajMat = " << path_estTrajMat << std::endl;

    */

    // obj : instance x y z r p yaw a b c
    MatrixXd estObjMat = readDataFromFile(path_estObjMat.c_str());
    MatrixXd gtTrajMat = readDataFromFile(path_gtTrajMat.c_str());
    MatrixXd estTrajMat = readDataFromFile(path_estTrajMat.c_str());

    std::cout << "refObjMat: " << std::endl << refObjMat << std::endl;
    std::cout << "estObjMat: " << std::endl << estObjMat << std::endl;
    std::cout << "gtTrajNum : " << gtTrajMat.rows() << std::endl;
    std::cout << "estTrajNum : " << estTrajMat.rows() << std::endl;

    evo::StaticResult staticResult;
    std::map<int, COMPARE_RESULT> results;

    bool bUseId = false;

    // if(argc>=4){
    //     int asso_method = stoi(string(argv[3]));
    //     bUseId = (asso_method == 1);
    // }

    bool manual_check = false;
    bool filter_sym = false;

    bool bResult;
    if(!bUseId)  //  bool manual_check, bool filter_sym, bool use_id_to_associate
        bResult = evo::Evaluate(refObjMat, estObjMat, gtTrajMat, estTrajMat, staticResult, results, pMap, manual_check);
    else 
        bResult = evo::Evaluate(refObjMat, estObjMat, gtTrajMat, estTrajMat, staticResult, results, pMap, manual_check, filter_sym, true);

    if(!bResult) {
        std::cout << "Tips: Please check the following files exist ... "     << std::endl;
        std::cout << output_check(path_refObjMat) << std::endl;
        std::cout << output_check(path_estObjMat) << std::endl;
        std::cout << output_check(path_estTrajMat) << std::endl;
        std::cout << output_check(path_gtTrajMat) << std::endl;
        return 0;
    }

    // 输出结果.
    string path_output(dataset_path_savedir + "objects_compare.txt");
    ofstream fio(path_output.c_str());
    OutputResult(fio, staticResult, results, refObjMat);
    fio.close();
    cout << "Save result to " << path_output << endl;

    cout << endl << endl;
    OutputResult(cout, staticResult, results, refObjMat);

    // // 开启加载背景地图功能
    // std::cout << "Load background pointcloud from " << dataset_path_map << "..." << std::endl;
    // pSLAM->getTracker()->LoadPointcloud(dataset_path_map, "background_world");
    // std::cout << "ok." << std::endl;

    std::cout << "ok." << std::endl;

    cout << "Press y to quit." << endl;
    char key;
    key = getchar();

    // while(1);

    while(key!='y' && key!='Y'){
        cout << "Press y to quit." << endl;
        key = getchar();
    }

    cv::destroyAllWindows();

    return 0;

}

