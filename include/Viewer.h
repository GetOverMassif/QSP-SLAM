/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef VIEWER_H
#define VIEWER_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"

#include <mutex>

struct MenuStruct
{
    MenuStruct(string name_,double def_,double min_,double max_)
        :name(name_), def(def_), min(min_), max(max_)
    {};
    double min;
    double max;
    double def;
    string name;
};

namespace ORB_SLAM2
{

class Tracking;
class FrameDrawer;
class MapDrawer;
class ObjectDrawer;
class System;

class AnnoObj{
public:
    int index = 0, label = 0;
    double x = 0,y = 0,z = 0;
    double roll = 0, pitch = 0, yaw = 0;
    double width = 0.1, height = 0.2, length = 0.4;

};

class Viewer
{
public:
    Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, ObjectDrawer* pObjectDrawer, Tracking *pTracking, const string &strSettingPath);

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();

    cv::Mat GetFrame();

private:

    bool Stop();
    void RefreshMenu();

    System* mpSystem;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    ObjectDrawer* mpObjectDrawer;
    Tracking* mpTracker;

    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;

    // Manual control of the point cloud visualization
    map<string,pangolin::Var<bool>*> mmPointCloudOptionMenus;
    std::map<std::string,bool> mmPointCloudOptionMap;


    // menu lists
    // vector<pangolin::Var<double>*> mvAnnoDoubleMenus;

    pangolin::Var<int>* p_menuAnnoIndex;
    pangolin::Var<bool>* p_menuAddAnnoObj;

    vector<pangolin::Var<double>*> mvDoubleMenus;
    vector<MenuStruct*> mvMenuStruct;

    map<string,pangolin::Var<double>*> mmAnnoOptionMenus;
    std::map<std::string,double> mmAnnoOptionMap;

    void RefreshPointCloudOptions();
    void RefreshAnnoOptions();

private:
    // 标注专区
    int AnnoNum = 0;
    int cur_AnnoIndex = -1;
    int prev_AnnoIndex = -1;
    vector<AnnoObj*> AnnoObjs;
    AnnoObj* cur_AnnoObj;
    AnnoObj* prev_AnnoObj;

};

}


#endif // VIEWER_H
	

