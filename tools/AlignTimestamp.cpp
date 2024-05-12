// 文件用于处理生成的 bbox 的timestamp与 rgb相差几个小数位的问题。

// 传入： bbox 文件夹位置; rgb.txt
// 输出： 对齐后的 bbox文件

#include "include/utils/dataprocess_utils.h"
#include "include/utils/matrix_utils.h"
#include <Eigen/Core>

using namespace Eigen;

VectorXd GetFirstCol(std::vector<std::vector<string>>& mvec)
{
    int num = mvec.size();
    VectorXd vec; vec.resize(num);
    for(int i=0;i<num;i++)
    {
        double timestamp = stod(mvec[i][0]);
        vec[i] = timestamp;
    }
    return vec;
}

int main(int argc, char *argv[])
{
    if(argc!=3)
    {
        std::cout << "Usage: " << argv[0] << " path_to_bbox path_to_rgb.txt" << std::endl;
        return 0;
    }
    const string path_to_bbox(argv[1]);
    const string path_to_rgbTimestamp(argv[2]);
    std::cout << "path_to_bbox : " << path_to_bbox << std::endl;
    std::cout << "path_to_rgbTimestamp : " << path_to_rgbTimestamp << std::endl;

    std::vector<std::vector<string>> timestampMatVec = readStringFromFile(path_to_rgbTimestamp.c_str());
    VectorXd vec_timestamps = GetFirstCol(timestampMatVec);

    std::vector<string> bboxFileNames;
    GetFileNamesUnderDir(path_to_bbox, bboxFileNames);

    int bbox_num = bboxFileNames.size();
    std::cout << "Dealing with bbox files : " << bbox_num << std::endl;
    for(int i=0;i<bbox_num;i++)
    {
        string fileFullName = bboxFileNames[i];
        string timestamp_str = splitFileNameFromFullDir(fileFullName, true);
        double timestamp = stod(timestamp_str);

        // rgb中寻找最近timestamp
        VectorXd timestamp_bordcast; timestamp_bordcast.resize(vec_timestamps.size());
        timestamp_bordcast.fill(timestamp);
        VectorXd error = (vec_timestamps - timestamp_bordcast).cwiseAbs();

        // 最小元素
	    MatrixXd::Index minRow, minCol;
        double min = error.minCoeff(&minRow,&minCol);

        // 此处判断 error 不能太大.
        if(min > 0.01) std::cout << "error too large, please check : " << fileFullName << std::endl;

        double real_timestamp = vec_timestamps[minRow];
        string str_real_timestamp = to_string(real_timestamp);

        // cp xxx to xxx
        string command;
        command = string("cp ") + fileFullName + " " + "./bbox/" + str_real_timestamp +".txt";
        std::cout << "Run : " << command << std::endl;
        system(command.c_str());
    }


    std::cout << "DONE." << std::endl;
    return 0;

}