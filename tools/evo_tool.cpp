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

int main(int argc,char* argv[]) {
    if( argc < 2 || argc > 5)
    {
        std::cout << "usage: " << argv[0] << " path_to_settings [file_name (/result/objects.txt)] [associate (0:auto) 1:use_id] [UseGtTraj (0:no) 1:yes]" << std::endl;
        std::cout << "Dataset path: will be read from the path. " << std::endl;
        return 1;
    }
    string strSettingPath = string(argv[1]);
    // 初始化 可视化系统
    pSLAM = new EllipsoidSLAM::System(strSettingPath);
    pMap = pSLAM->getMap();

    //  path_to_dataset [path_to_map]
    string dataset_path = Config::Get<string>("Dataset.Path.Root");
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

    std::cout << "- settings file: " << strSettingPath << std::endl;
    std::cout << "- dataset_path: " << dataset_path << std::endl;
    std::cout << "- strDetectionDir: " << strDetectionDir << std::endl;
    string dataset_type = Config::Get<string>("Dataset.Type");
    std::cout << "- dataset_type : " << dataset_type << std::endl;

    // const string path_refObjMat = dataset_path + "/objects_gt.txt";
    const string path_refObjMat = dataset_path+"/objects_gt_ground.txt"; // 注意 _ground 表示仅包括位于地面的物体!
    MatrixXd refObjMat = readDataFromFile(path_refObjMat.c_str());
    if(refObjMat.rows()==0){    // 如果不存在ground版本则读原始gt
        string path_gt_objects = dataset_path+"/objects_gt.txt"; 
        refObjMat = readDataFromFile(path_gt_objects.c_str());
    }

    string estfile_name ("/result/objects.txt");
    if(argc >= 3){
        estfile_name = string(argv[2]);
    }
    const string path_estObjMat = dataset_path + "/" + estfile_name;
    std::cout << "Read Est Obj from : " << path_estObjMat << std::endl;

    const string path_gtTrajMat = dataset_path + "/groundtruth.txt";
    
    bool bUseGtTraj = false;
    if(argc >= 5)
        bUseGtTraj = (stoi(string(argv[4])) == 1);
    string path_estTrajMat;
    if(!bUseGtTraj)
        path_estTrajMat = dataset_path + "/result/traj.txt";
    else
        path_estTrajMat = path_gtTrajMat;

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
    if(argc>=4){
        int asso_method = stoi(string(argv[3]));
        bUseId = (asso_method == 1);
    }
    bool bResult;
    if(!bUseId)
        bResult = evo::Evaluate(refObjMat, estObjMat, gtTrajMat, estTrajMat, staticResult, results, pMap, false);
    else 
        bResult = evo::Evaluate(refObjMat, estObjMat, gtTrajMat, estTrajMat, staticResult, results, pMap, false, false, true);

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

    // 开启加载背景地图功能
    std::cout << "Load background pointcloud from " << dataset_path_map << "..." << std::endl;
    pSLAM->getTracker()->LoadPointcloud(dataset_path_map, "background_world");
    std::cout << "ok." << std::endl;

    while(1);

    return 0;

}

