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
    if(argc != 6)
    {
        cerr << endl << "Usage: ./qsp_slam_rgbd path_to_vocabulary path_to_settings path_to_sequence path_to_association path_to_saved_trajectory" << endl;
        return 1;
    }

    // vector<string> vstrImageFilenamesRGB;
    // vector<string> vstrImageFilenamesD;
    // vector<double> vTimestamps;
    // string strAssociationFilename = string(argv[4]);

    cv::FileStorage fSettings(string(argv[2]), cv::FileStorage::READ);

    if(fSettings["CoutToFile"].isNamed()) {
        ofstream fout("logs/log.txt"); //文件输出流对象
        streambuf* pOld =cout.rdbuf(fout.rdbuf());
    }

    // LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // int nImages = vstrImageFilenamesRGB.size();

    // if(vstrImageFilenamesRGB.empty()){
    //     cerr << endl << "No images found in provided path." << endl;
    //     return 1;
    // }
    // else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size()){
    //     cerr << endl << "Different number of images for rgb and depth." << endl;
    //     return 1;
    // }

    auto msensor = ORB_SLAM2::System::MONOCULAR;

    if(fSettings["System.mode"].isNamed()) {
        string system_mode = fSettings["System.mode"];
        if (system_mode=="RGBD") {
            msensor = ORB_SLAM2::System::RGBD;
        }
    }

    pSLAM = new ORB_SLAM2::System(argv[1], argv[2], argv[3], msensor);
    pMap = pSLAM->getMap();

    string strSettingsFile = argv[2];
    std::string dataset_path = std::string(argv[3]);
    std::string save_map_dir = std::string(argv[5]);

    // //  path_to_dataset [path_to_map]
    // string dataset_path = Config::Get<string>("Dataset.Path.Root");

    string dataset_path_map = Config::Get<string>("Dataset.Path.Map");


    if(dataset_path_map.size()>0)
    {
        if(dataset_path_map[0]=='.')    // 相对路径
            dataset_path_map = dataset_path + "/" + dataset_path_map;
    }

    string dataset_path_savedir = dataset_path + "/result/";

    std::cout << "Load background pointcloud from " << dataset_path_map << "..." << std::endl;
    pSLAM->getTracker()->LoadPointcloud(dataset_path_map, "background_world");
    std::cout << "ok." << std::endl;

    string strDetectionDir = dataset_path + "/bbox/";

    std::cout << "- settings file: " << strSettingsFile << std::endl;
    std::cout << "- dataset_path: " << dataset_path << std::endl;
    std::cout << "- strDetectionDir: " << strDetectionDir << std::endl;
    string dataset_type = Config::Get<string>("Dataset.Type");
    std::cout << "- dataset_type : " << dataset_type << std::endl;

    /**
     * gt_traj: "/groundtruth.txt"
     * gt_object_ground: "/objects_gt_ground.txt"
     * gt_object: "/objects_gt.txt"
     * 
     * est_traj: "/result/traj.txt"
     * est_object: "/result/objects.txt"
    */
    // const string path_refObjMat = dataset_path + "/objects_gt.txt";
    const string path_refObjMat = dataset_path + "/objects_gt_ground.txt"; // 注意 _ground 表示仅包括位于地面的物体!
    MatrixXd refObjMat = readDataFromFile(path_refObjMat.c_str());

    if(refObjMat.rows()==0){    // 如果不存在ground版本则读原始gt
        string path_gt_objects = dataset_path + "/objects_gt.txt"; 
        refObjMat = readDataFromFile(path_gt_objects.c_str());
    }

    string estfile_name ("/result/objects.txt");

    // if(argc >= 3){
    //     estfile_name = string(argv[2]);
    // }

    const string path_estObjMat = dataset_path + "/" + estfile_name;

    std::cout << "\nRead Est Obj from : " << path_estObjMat << std::endl;

    /**
     * 读取轨迹真值、是否使用轨迹真值
    */

    const string path_gtTrajMat = dataset_path + "/groundtruth.txt";
    
    // bool bUseGtTraj = false;

    // if(argc >= 5)
    //     bUseGtTraj = (stoi(string(argv[4])) == 1);
    
    // string path_estTrajMat;

    // if(!bUseGtTraj)
    //     path_estTrajMat = dataset_path + "/result/traj.txt";
    // else
    //     path_estTrajMat = path_gtTrajMat;

    const string path_estTrajMat = dataset_path + "/result/traj.txt";

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
    string path_output("./objects_compare.txt");
    ofstream fio(path_output.c_str());
    OutputResult(fio, staticResult, results, refObjMat);
    fio.close();
    cout << "Save result to " << path_output << endl;

    cout << endl << endl << endl;
    OutputResult(cout, staticResult, results, refObjMat);

    // // 开启加载背景地图功能
    // std::cout << "Load background pointcloud from " << dataset_path_map << "..." << std::endl;
    // pSLAM->getTracker()->LoadPointcloud(dataset_path_map, "background_world");
    // std::cout << "ok." << std::endl;

    // while(1);

    cout << "Press y to quit." << endl;
    char key;
    key = getchar();

    while(key!='y' && key!='Y'){
        cout << "Press y to quit." << endl;
        key = getchar();
    }

    cv::destroyAllWindows();

    return 0;

}

