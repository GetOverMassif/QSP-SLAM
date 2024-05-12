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

#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM2
{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, ObjectDrawer *pObjectDrawer, Tracking *pTracking, const string &strSettingPath):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpObjectDrawer(pObjectDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
}

cv::Mat Viewer::GetFrame()
{
    return mpFrameDrawer->DrawFrame();
}

void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;
    int w = 1024;
    int h = 576;
    pangolin::CreateWindowAndBind("QSP-SLAM: Map Viewer",w,h);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));

    mvMenuStruct.resize(9); mvDoubleMenus.resize(9);
    mvMenuStruct[0] = new MenuStruct(" - X (red)", 0, -5, 5);
    mvMenuStruct[1] = new MenuStruct(" - Y (green)", 0, -5, 5);
    mvMenuStruct[2] = new MenuStruct(" - Z (blue)", 0, -5, 5);

    double PI = 3.1415927;
    double h_PI = PI / 2.;
    mvMenuStruct[3] = new MenuStruct(" - Roll (red)", 0,-h_PI,h_PI);
    mvMenuStruct[4] = new MenuStruct(" - Pitch (green)", 0,-h_PI,h_PI);
    mvMenuStruct[5] = new MenuStruct(" - Yaw (blue)", 0,-h_PI,h_PI);

    mvMenuStruct[6] = new MenuStruct(" - Len (red)", 0.5,0.001,3);
    mvMenuStruct[7] = new MenuStruct(" - Wid (green)", 0.5,0.001,3);
    mvMenuStruct[8] = new MenuStruct(" - H (blue)", 0.5,0.001,3);

    pangolin::Var<bool> menuShowAnnoObj("menu.Show AnnoObj",true,true);
    pangolin::Var<bool> menuAddAnnoObj("menu.AddAnnoObj",false,false);
    pangolin::Var<bool> menuSaveAnnoObj("menu.SaveAnnoObj",false,false);
    // pangolin::Var<int> menuAnnoIndex("menu.AnnoIndex",cur_AnnoIndex,-1,AnnoNum);
    pangolin::Var<int>* p_menuAnnoIndex = new pangolin::Var<int>("menu.AnnoIndex",-1,-1,15);

    // 首先创建这些
    for(int i_d = 0; i_d < 9; i_d++)
    {
        pangolin::Var<double>* pMenu = 
            new pangolin::Var<double>(
                string("menu.") + mvMenuStruct[i_d]->name,
                mvMenuStruct[i_d]->def, 
                mvMenuStruct[i_d]->min, 
                mvMenuStruct[i_d]->max
            );
        // std::cout << "mvMenuStruct[" << i_d << "]->def = " << mvMenuStruct[i_d]->def << std::endl;
        mvDoubleMenus[i_d] = pMenu;
    }


    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",false,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",false,true);
    pangolin::Var<bool> menuShowRawObjPoints("menu.Show Raw Obj Points",true,true);


    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);
    pangolin::Var<bool> menuClose("menu.Close",false,false);

    pangolin::Var<bool> menuPause("menu.Pause",false,false);

    pangolin::Var<bool> menuShowMapObjects("menu.Show MapObjects",true,true);

    pangolin::Var<bool> menuShowObjMesh("menu. - Show Mesh",true,true);
    pangolin::Var<bool> menuShowObjCube("menu. - Show Cuboid",true,true);

    pangolin::Var<bool> menuShowEllipsoidsDetails("menu. - Show Details", true, true);

    pangolin::Var<bool> menuShowGroundPlane("menu.Show GroundPlane",true,true);
    pangolin::Var<bool> menuShowEllipsoids("menu.Show Ellipsoids Visual", true, true);
    pangolin::Var<bool> menuShowEllipsoidsObjects("menu.Show Ellipsoids EllipObjects", true, true);
    // pangolin::Var<bool> menuShowPointCloudLists("menu.Show PointCloudLists", true, true);

    pangolin::Var<bool> menuShowEllipsoidsObservation("menu.Ellipsoids-Ob", true, true);
    // pangolin::Var<bool> menuShowCuboids("menu. - Show Cuboids", false, true);
    // pangolin::Var<bool> menuShowPlanes("menu.Show Planes", true, true);
    // pangolin::Var<bool> menuShowBoundingboxes("menu.Show Bboxes", false, true);
    // pangolin::Var<bool> menuShowConstrainPlanesBbox("menu.ConstrainPlanes-bbox", false, true);
    // pangolin::Var<bool> menuShowConstrainPlanesCuboids("menu.ConstrainPlanes-cuboids", false, true);
    pangolin::Var<double> SliderEllipsoidProbThresh("menu.Ellipsoid Prob", 0.3, 0.0, 1.0);
    pangolin::Var<float> SliderMapPointSize("menu.MapPoint Size", 1.0, 0.5, 5.0);
    pangolin::Var<float> SliderPointCloudListSize("menu.Pointcloud Size", 1.0, 0.5, 5.0);

    // pangolin::Var<bool> menuShowWorldAxis("menu.Draw World Axis", false, true);

    // pangolin::Var<bool> menuAMeaningLessBar("menu.----------", false, false);

    // pangolin::Var<bool> menuShowOptimizedTraj("menu.Optimized Traj", true, true);
    
    pangolin::Var<bool> menuShowGtTraj("menu.Gt Traj", true, true);
    pangolin::Var<bool> menuShowCamTraj("menu.Cam Traj", true, true);


    // pangolin::Var<bool> menuShowRelationArrow("menu.Relation Arrow", false, true);
    // pangolin::Var<int> SliderOdometryWeight("menu.Odometry Weight", 40, 0, 40);
    // pangolin::Var<double> SliderPlaneWeight("menu.Plane Weight", 1, 0, 100);
    // pangolin::Var<double> SliderPlaneDisSigma("menu.PlaneDisSigma", Config::Get<double>("DataAssociation.PlaneError.DisSigma"), 0.01, 0.5);
    // pangolin::Var<int> Slider3DEllipsoidScale("menu.3DEllipsoidScale(10^)", log10(Config::Get<double>("Optimizer.Edges.3DEllipsoid.Scale")), -5, 10);
    // pangolin::Var<int> Slider2DEllipsoidScale("menu.2DEllipsoidScale(10^)", log10(Config::Get<double>("Optimizer.Edges.2D.Scale")), -5, 10);
    // pangolin::Var<int> SliderUseProbThresh("menu.Use Prob Thresh", 0, 0, 1);
    // pangolin::Var<int> SliderOptimizeRelationPlane("menu.OptimizeRelationPlane", 0, 0, 1);
    // pangolin::Var<bool> menuShowOptimizedSupPlanes("menu.Optimized SupPlanes", false, true);
    // pangolin::Var<bool> menuShowSupPlanesObservation("menu.SupPlanes Observations", false, true);
    // pangolin::Var<bool> menuOpenQuadricSLAM("menu.Open QuadricSLAM", false, true);

    // pangolin::Var<std::function<void(void)>> menuSaveView("menu.Save View", func_save_view);//设置一个按钮，用于调用function函数
    // pangolin::Var<std::function<void(void)>> menuLoadView("menu.Load View", func_load_view);//设置一个按钮，用于调用function函数

    // pangolin::Var<bool> menuOpenOptimization("menu.Open Optimization", false, true);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(w,h, mViewpointF,mViewpointF,w / 2,h / 2,0.1,5000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -float(w) / float(h))
            .SetHandler(new pangolin::Handler3D(s_cam));

    auto mpRenderer = new ObjectRenderer(w, h);
    mpRenderer->SetupCamera(mViewpointF, mViewpointF, double(w) / 2, double(h) / 2, 0.1, 5000);
    mpObjectDrawer->SetRenderer(mpRenderer);

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    cv::namedWindow("QSP-SLAM: Current Frame");

    bool bFollow = true;
    bool bLocalizationMode = false;
    Eigen::Matrix4f Tec;

    while(1)
    {

        RefreshMenu();  // Deal with dynamic menu bars

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        if(menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);

        // Used for object drawer
        Tec = s_cam.GetModelViewMatrix();
        Tec.row(1) = -Tec.row(1);
        Tec.row(2) = -Tec.row(2);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        mpMapDrawer->DrawCurrentCamera(Twc);

        if (menuShowAnnoObj)
        {
            // Anno objects
            for (int i_anno = 0; i_anno < AnnoObjs.size(); i_anno++)
            {
                AnnoObj* p_obj = AnnoObjs[i_anno];
                double x = p_obj->x, y = p_obj->y, z = p_obj->z;
                double roll = p_obj->roll, pitch = p_obj->pitch, yaw = p_obj->yaw;
                double length = p_obj->length, width = p_obj->width, height = p_obj->height;
                Eigen::Matrix3f rotation;
                rotation = Eigen::AngleAxisf(float(roll), Eigen::Vector3f::UnitX())
                        * Eigen::AngleAxisf(float(pitch), Eigen::Vector3f::UnitY())
                        * Eigen::AngleAxisf(float(yaw), Eigen::Vector3f::UnitZ());

                Eigen::Matrix4f transformationMatrix = Eigen::Matrix4f::Identity();
                transformationMatrix.block<3,3>(0,0) = rotation;
                transformationMatrix(0,3) = float(x);
                transformationMatrix(1,3) = float(y);
                transformationMatrix(2,3) = float(z);

                Eigen::Matrix4f Two = transformationMatrix;
                Eigen::Vector3f Scale;
                Scale << float(length),float(width),float(height);
                bool with_mesh = true;
                mpMapDrawer->drawCuboid9Dof(Two, Scale, with_mesh);
                
            }
        }



        // 
        float mappointSize = SliderMapPointSize;

        if(menuShowCamTraj)
            mpMapDrawer->drawTrajectory();

        /** From ORB-SLAM*/
        if(menuShowKeyFrames || menuShowGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);

        // bool drawRawObjPoint = 
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints(mappointSize, menuShowRawObjPoints);

        
        if(menuShowGroundPlane)
            mpMapDrawer->drawPlanes(0); // 0:default.


        double ellipsoidProbThresh = SliderEllipsoidProbThresh;

        // draw ellipsoids
        // 这里绘制了
        if(menuShowEllipsoids)
            mpMapDrawer->drawEllipsoidsVisual(ellipsoidProbThresh);

        if(menuShowEllipsoidsObjects)
            mpMapDrawer->drawEllipsoidsObjects(ellipsoidProbThresh);

        // // draw the result of the single-frame ellipsoid extraction
        // if(menuShowEllipsoidsObservation)
        //     mpMapDrawer->drawObservationEllipsoids(ellipsoidProbThresh);
        
        // mpMapDrawer->drawEllipsoid();

        // draw pointclouds with names
        float pointcloudSize = SliderPointCloudListSize;

        RefreshPointCloudOptions();

        mpMapDrawer->drawPointCloudWithOptions(mmPointCloudOptionMap, pointcloudSize);

        // float pointcloudSize = SliderPointCloudListSize;
        // if(menuShowPointCloudLists)
        //     mpMapDrawer->drawPointCloudLists(pointcloudSize);


        // if(menuShowPcdGlobal)
        //     mpMapDrawer->drawPointCloudLists(pointcloudSize);
        
        // if(menuShowPcdLocal)
        //     mpMapDrawer->drawPointCloudLists(pointcloudSize);
        
        //     mpMap->AddPointCloudList("Builder.Global Points", pCloud);
        // mpMap->AddPointCloudList("Builder.Local Points", pCloudLocal);

        

        if(menuShowGtTraj)
            mpMapDrawer->drawTrajectoryWithName("Groundtruth");

        mpObjectDrawer->ProcessNewObjects();

        // menuShowObjMesh,menuShowObjCube

        if(menuShowMapObjects){
            mpObjectDrawer->DrawObjects(bFollow, Tec, menuShowObjMesh,menuShowObjCube, ellipsoidProbThresh, pointcloudSize);
        }

        // // 如果是RGBD模式，则绘制深度图对应的点云
        // if (mpTracker->mSensor == System::RGBD) {
        //     mpMapDrawer->DrawDepthPointCloud();
            
        // }

        // std::cout << "After mpObjectDrawer->DrawObjects" << std::endl;

        pangolin::FinishFrame();



        cv::Mat im = GetFrame();
//        double scale = float(w) / im.size().width;
//        cv::Mat scaled_im;
//        cv::resize(im, scaled_im, cv::Size(0, 0), scale, scale);
        cv::imshow("QSP-SLAM: Current Frame", im);
        cv::waitKey(mT);

        if(menuPause) {
            mpTracker->SetFrameByFrame();
            menuPause = false;
        }

        if (menuSaveAnnoObj) {
            menuSaveAnnoObj = false;
            std::string dataset_path = "/media/lj/TOSHIBA/dataset/ICL-NUIM/home/living_room_traj2_frei_png";
            std::string obj_gt_txt = dataset_path + "/objects_gt.txt";
            ofstream f;
            f.open(obj_gt_txt.c_str());
            f << fixed;
            // index, x,y,z, roll, pitch, yaw, half length of axis x,y,z, label
            for (int i_anno = 0; i_anno < AnnoObjs.size(); i_anno++)
            {
                AnnoObj* p_obj = AnnoObjs[i_anno];
                int index = p_obj->index, label = p_obj->label;
                double x = p_obj->x, y = p_obj->y, z = p_obj->z;
                double roll = p_obj->roll, pitch = p_obj->pitch, yaw = p_obj->yaw;
                double length = p_obj->length, width = p_obj->width, height = p_obj->height;

                f << index << " " << setprecision(6)
                  << x << " " << y << " " << z << " "
                  << roll << " " << pitch << " " << yaw << " "
                  << length << " " << width << " " << height << " "
                  << label << endl;
            }
            f.close();
            std::cout << "Save obj gt to : " << obj_gt_txt << std::endl;
        }


        if(menuAddAnnoObj){
            AnnoNum++;
            menuAddAnnoObj = false;
            cout << " AnnoNum = " << AnnoNum << std::endl;
            if (AnnoNum > AnnoObjs.size()){
                AnnoObjs.resize(AnnoNum);
                AnnoObjs[AnnoNum - 1] = new AnnoObj();
                AnnoObjs[AnnoNum - 1]->index = AnnoNum - 1;
            }
        }

        cur_AnnoIndex = p_menuAnnoIndex->Get();

        if (cur_AnnoIndex >= AnnoNum) {
            std::cout << "Please set cur_AnnoIndex = 0 ~ " << AnnoNum - 1 << std::endl;
            p_menuAnnoIndex->Ref().Set(AnnoNum - 1);
            cur_AnnoIndex = p_menuAnnoIndex->Get();
        }

        if (AnnoNum > 0 && cur_AnnoIndex < AnnoNum) {
            if (cur_AnnoIndex == -1) {
                if (prev_AnnoIndex != cur_AnnoIndex) {
                    for (int i_menu = 0; i_menu < 9; i_menu++) {
                        mvDoubleMenus[i_menu]->Ref().Set(mvMenuStruct[i_menu]->def);
                    }
                }
            }
            else{
                cur_AnnoObj = AnnoObjs[cur_AnnoIndex];
                // 如果 menuAnnoIndex 没有发生变化，无需手动更新滑动条数值
                if (prev_AnnoIndex == cur_AnnoIndex){
                    cur_AnnoObj->x = mvDoubleMenus[0]->Get();
                    cur_AnnoObj->y = mvDoubleMenus[1]->Get();
                    cur_AnnoObj->z = mvDoubleMenus[2]->Get();
                    cur_AnnoObj->roll = mvDoubleMenus[3]->Get();
                    cur_AnnoObj->pitch = mvDoubleMenus[4]->Get();
                    cur_AnnoObj->yaw = mvDoubleMenus[5]->Get();
                    cur_AnnoObj->length = mvDoubleMenus[6]->Get();
                    cur_AnnoObj->width = mvDoubleMenus[7]->Get();
                    cur_AnnoObj->height = mvDoubleMenus[8]->Get();
                }
                else{
                    mvDoubleMenus[0]->Ref().Set(cur_AnnoObj->x);
                    mvDoubleMenus[1]->Ref().Set(cur_AnnoObj->y);
                    mvDoubleMenus[2]->Ref().Set(cur_AnnoObj->z);
                    mvDoubleMenus[3]->Ref().Set(cur_AnnoObj->roll);
                    mvDoubleMenus[4]->Ref().Set(cur_AnnoObj->pitch);
                    mvDoubleMenus[5]->Ref().Set(cur_AnnoObj->yaw);
                    mvDoubleMenus[6]->Ref().Set(cur_AnnoObj->length);
                    mvDoubleMenus[7]->Ref().Set(cur_AnnoObj->width);
                    mvDoubleMenus[8]->Ref().Set(cur_AnnoObj->height);
                }
            }

            prev_AnnoIndex = cur_AnnoIndex;
        }
        

        // RefreshAnnoOptions();

        if(menuReset)
        {
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            mpSystem->Reset();
            menuReset = false;

        }

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if (menuClose) 
        {
            mpSystem->Shutdown();
        }

        if(CheckFinish())
            break;

    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

void Viewer::RefreshPointCloudOptions()
{
    // generate options from mmPointCloudOptionMenus, pointclouds with names will only be drawn when their options are activated.
    std::map<std::string,bool> options;
    for( auto pair : mmPointCloudOptionMenus)
        options.insert(make_pair(pair.first, pair.second->Get()));
    
    mmPointCloudOptionMap.clear();
    mmPointCloudOptionMap = options;
}

void Viewer::RefreshAnnoOptions()
{
    // std::map<std::string, double> options;

    // for (auto pair : mmAnnoOptionMenus) {
    //     options.insert(make_pair(pair.first, pair.second->Get()));
    // }

    // mmAnnoOptionMap.clear();
    // mmAnnoOptionMap = options;
}

void Viewer::RefreshMenu(){
    unique_lock<mutex> lock(mMutexFinish);

    // 以名称为单位，给 pointcloud list 中的每个点云设置菜单
    auto pointLists = mpSystem->getMap()->GetPointCloudList();

    // Iterate over the menu and delete the menu if the corresponding clouds are no longer available
    // 遍历菜单，如果对应的点云没有了则删除菜单
    for( auto menuPair = mmPointCloudOptionMenus.begin(); menuPair!=mmPointCloudOptionMenus.end();)
    {
        if(pointLists.find(menuPair->first) == pointLists.end())
        {
            if( menuPair->second !=NULL ){
                delete menuPair->second;        // destroy the dynamic menu 
                menuPair->second = NULL;
            }
            menuPair = mmPointCloudOptionMenus.erase(menuPair);  
            continue;
        }
        menuPair++;
    }

    // Iterate over the cloud lists to add new menu.
    // 遍历点云列表，添加新菜单
    for( auto cloudPair: pointLists )
    {
        if(mmPointCloudOptionMenus.find(cloudPair.first) == mmPointCloudOptionMenus.end())
        {
            pangolin::Var<bool>* pMenu = new pangolin::Var<bool>(string("menu.") + cloudPair.first, false, true);
            mmPointCloudOptionMenus.insert(make_pair(cloudPair.first, pMenu));            
        }
    }

    // refresh double bars
    // int doubleBarNum = mvDoubleMenus.size();

    // 标注专区
    // 管理熟悉：当前物体编号，总物体数量
    // 每个物体属性： 物体编号、物体语义标签

    // std::cout << "set p_menuAddAnnoObj false" << std::endl;

    // if (p_menuAddAnnoObj != nullptr) {
    //     std::cout << "p_menuAddAnnoObj = " << p_menuAddAnnoObj->Get() << std::endl;
    // }

    // delete p_menuAnnoIndex;
    // p_menuAnnoIndex = new pangolin::Var<int>("menu.AnnoIndex",cur_AnnoIndex,-1,AnnoNum);

    // std::cout << "Set AnnoNum = " << AnnoNum << std::endl;
    // p_menuAddAnnoObj = new pangolin::Var<bool>("menu.AddAnnoObj",false,false);

    



}


}
