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

#include <LocalMapping.h>
#include <ORBmatcher.h>

using namespace std;

namespace ORB_SLAM2
{

/*
 * Tracking utils for stereo+lidar on KITTI
 */
void LocalMapping::MapObjectCulling()
{
    // Check Recent Added MapObjects
    list<MapObject*>::iterator lit = mlpRecentAddedMapObjects.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    const int cnThObs = 2;

    // Treat static and dynamic objects differently
    while(lit != mlpRecentAddedMapObjects.end())
    {
        MapObject* pMO = *lit;
        if (pMO->isDynamic())
        {
            if ((int) nCurrentKFid - (int) pMO->mpNewestKF->mnId  >= 2)
            {
                pMO->SetBadFlag();
                lit = mlpRecentAddedMapObjects.erase(lit);
                mpMap->mnDynamicObj--;
            }
        }

        if(pMO->isBad())
        {
            lit = mlpRecentAddedMapObjects.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMO->mnFirstKFid) >= 2 && pMO->Observations() <= cnThObs)
        {
            pMO->SetBadFlag();
            lit = mlpRecentAddedMapObjects.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMO->mnFirstKFid) >= 3)
            lit = mlpRecentAddedMapObjects.erase(lit);
        else
            lit++;
    }

    // Dynamic objects that aren't recently added
    if (mpMap->mnDynamicObj > 0)
    {
        std::vector<MapObject*> pMOs = mpMap->GetAllMapObjects();
        for (MapObject *pMO : pMOs)
        {
            if (pMO->isDynamic())
            {
                if ((int) nCurrentKFid - (int) pMO->mpNewestKF->mnId  >= 2)
                {
                    pMO->SetBadFlag();
                    mpMap->mnDynamicObj--;
                }
            }
        }
    }
}

void LocalMapping::GetNewObservations()
{
    PyThreadStateLock PyThreadLock;

    // cout << "LocalMapping: Estimating new poses for associated objects" << endl;

    auto Tcw = Converter::toMatrix4f(mpCurrentKeyFrame->GetPose());
    auto mvpAssociatedObjects = mpCurrentKeyFrame->GetMapObjectMatches();
    auto mvpObjectDetections = mpCurrentKeyFrame->GetObjectDetections();

    for (int i = 0; i < mvpObjectDetections.size(); i++)
    {
        auto det = mvpObjectDetections[i];
        if (det->isNew)
            continue;
        if (!det->isGood)
            continue;

        auto pMO = mvpAssociatedObjects[i];
        if (pMO)
        {
            // Tco obtained by transforming Two to camera frame
            Eigen::Matrix4f iniSE3Tco = Tcw * pMO->GetPoseSE3();
            g2o::SE3Quat Tco = Converter::toSE3Quat(iniSE3Tco);
            // Tco after running ICP, use Tco provided by detector
            Eigen::Matrix4f SE3Tco = pyOptimizer.attr("estimate_pose_cam_obj")
                    (det->SE3Tco, pMO->scale, det->SurfacePoints, pMO->GetShapeCode()).cast<Eigen::Matrix4f>();
            g2o::SE3Quat Zco = Converter::toSE3Quat(SE3Tco);
            // error
            Eigen::Vector3f dist3D = SE3Tco.topRightCorner<3, 1>() - iniSE3Tco.topRightCorner<3, 1>();
            Eigen::Vector2f dist2D; dist2D << dist3D[0], dist3D[2];
            Eigen::Vector<double , 6> e = (Tco.inverse() * Zco).log();

            if (pMO->isDynamic()) // if associated with a dynamic object
            {
                auto motion = pMO->SE3Tow * Tcw.inverse() * SE3Tco;
                float deltaT = (float)(mpCurrentKeyFrame->mnFrameId - mpLastKeyFrame->mnFrameId);
                auto speed = motion.topRightCorner<3, 1>() / deltaT;
                pMO->SetObjectPoseSE3(Tcw.inverse() * SE3Tco);
                pMO->SetVelocity(speed);
            }
            else // associated with a static object
            {
                if (dist2D.norm() < 1.0 && e.norm() < 1.5) // if the change of translation is very small, then it really is a static object
                {
                    det->SetPoseMeasurementSE3(SE3Tco);
                }
                else // if change is large, it could be dynamic object or false association
                {
                    // If just observed, assume it is dynamic
                    if (pMO->Observations() <= 2)
                    {
                        pMO->SetDynamicFlag();
                        auto motion = pMO->SE3Tow * Tcw.inverse() * SE3Tco;
                        float deltaT = (float)(mpCurrentKeyFrame->mnFrameId - mpLastKeyFrame->mnFrameId);
                        auto speed = motion.topRightCorner<3, 1>() / deltaT;
                        pMO->SetObjectPoseSE3(Tcw.inverse() * SE3Tco);
                        pMO->SetVelocity(speed);
                        mpMap->mnDynamicObj++;
                    }
                    else
                    {
                        det->isNew = true;
                        mpCurrentKeyFrame->EraseMapObjectMatch(i);
                        pMO->EraseObservation(mpCurrentKeyFrame);
                    }
                }
            }
        }
    }
}

void LocalMapping::CreateNewMapObjects()
{
    PyThreadStateLock PyThreadLock;

    // cout << "LocalMapping: Started new objects creation" << endl;

    auto SE3Twc = Converter::toMatrix4f(mpCurrentKeyFrame->GetPoseInverse());
    auto mvpObjectDetections = mpCurrentKeyFrame->GetObjectDetections();
    // cout << "LocalMapping: mvpObjectDetections.size() = " << mvpObjectDetections.size() << endl;

    for (int i = 0; i < mvpObjectDetections.size(); i++)
    {
        // This might happen when a new KF is created in Tracking thread
        if (mbAbortBA)
            return;

        auto det = mvpObjectDetections[i];

        if (det->nRays == 0)
            continue;
        if (!det->isNew)
            continue;
        if (!det->isNew)
            continue;
        auto pyMapObject = pyOptimizer.attr("reconstruct_object")
                (det->Sim3Tco, det->SurfacePoints, det->RayDirections, det->DepthObs);
        if (!pyMapObject.attr("is_good").cast<bool>())
            continue;

        if (mbAbortBA)
            return;

        auto Sim3Tco = pyMapObject.attr("t_cam_obj").cast<Eigen::Matrix4f>();
        det->SetPoseMeasurementSim3(Sim3Tco);
        // Sim3, SE3, Sim3
        Eigen::Matrix4f Sim3Two = SE3Twc * Sim3Tco;
        auto code = pyMapObject.attr("code").cast<Eigen::Vector<float, 64>>();
        auto pNewObj = new MapObject(Sim3Two, code, mpCurrentKeyFrame, mpMap);

        auto pyMesh = pyMeshExtractor.attr("extract_mesh_from_code")(code);
        pNewObj->vertices = pyMesh.attr("vertices").cast<Eigen::MatrixXf>();
        pNewObj->faces = pyMesh.attr("faces").cast<Eigen::MatrixXi>();

        pNewObj->AddObservation(mpCurrentKeyFrame, i);
        mpCurrentKeyFrame->AddMapObject(pNewObj, i);
        mpMap->AddMapObject(pNewObj);
        mpObjectDrawer->AddObject(pNewObj);
        cout << "mpObjectDrawer->AddObject" << endl;
        mlpRecentAddedMapObjects.push_back(pNewObj);
    }
    // cout << "LocalMapping: Finished new objects creation" << endl;
}

/*
 * Tracking utils for monocular input on Freiburg Cars and Redwood OS
 */
void LocalMapping::CreateNewObjectsFromDetections()
{

    /*这一帧中进行新物体的创建*/

    cout << "[ LocalMapping - CreateNewObjectsFromDetections ]" << endl;

    // Step 1: 获取当前帧的旋转、平移和物体检测
    cv::Mat Rcw = mpCurrentKeyFrame->GetRotation();
    cv::Mat tcw = mpCurrentKeyFrame->GetTranslation();
    auto mvpObjectDetections = mpCurrentKeyFrame->GetObjectDetections();

    std::cout << " => " << "KF" << mpCurrentKeyFrame->mnId << ", mvpObjectDetections.size() = " << mvpObjectDetections.size() << std::endl;

    // Step 2: 遍历检测
    // Create new objects first, otherwise data association might fail
    for (int det_i = 0; det_i < mvpObjectDetections.size(); det_i++)
    {
        // Step 2.1: 如果该检测如果已经与物体关联/关键点过少，则continue

        auto det = mvpObjectDetections[det_i];

        // If the detection is a new object, create a new map object.
        // todo: 这里原本根据点云数量进行判断是否创建物体

        if (!det->isNew) {
            std::cout << "continue because !det->isNew" << std::endl;
            continue;
        }
            
        if (!det->isGood) {
            std::cout << "continue because !det->isGood" << std::endl;
            continue;
        }

        // Step 2.2: 创建地图物体，向当前帧、地图中进行添加，向该物体添加关联地图点
        auto pNewObj = new MapObject(mpCurrentKeyFrame, mpMap);
        mpCurrentKeyFrame->AddMapObject(pNewObj, det_i);
        mpMap->AddMapObject(pNewObj);

        // New add
        det->isNew = false;

        auto mvpMapPoints = mpCurrentKeyFrame->GetMapPointMatches();
        int n_valid_points = 0;
        for (int k_i : det->GetFeaturePoints())
        {
            auto pMP = mvpMapPoints[k_i];
            if (!pMP)
                continue;
            if (pMP->isBad())
                continue;
            pMP->in_any_object = true;
            pMP->object_id = pNewObj->mnId;
            pMP->keyframe_id_added_to_object = int(mpCurrentKeyFrame->mnId);
            pNewObj->AddMapPoints(pMP);
            n_valid_points++;
        }
        std::cout << "n_valid_points = " << n_valid_points << std::endl;
        std::cout << "det->isNew" << det->isNew << std::endl;
        // todo: 这里只处理了一个结果
        return;  // for mono sequences, we only focus on the single object in the middle
    }
}

void LocalMapping::ProcessDetectedObjects()
{
    
    std::cout << "[ LocalMapping - ProcessDetectedObjects ]" << std::endl;
    char key;
    // std::cout << "Ready to reconstruct_object" << std::endl;
    // std::cout << "Press [ENTER] to continue ... " << std::endl;
    // key = getchar();
    auto SE3Twc = Converter::toMatrix4f(mpCurrentKeyFrame->GetPoseInverse());
    auto SE3Tcw = Converter::toMatrix4f(mpCurrentKeyFrame->GetPose());
    cv::Mat Rcw = mpCurrentKeyFrame->GetRotation();
    cv::Mat tcw = mpCurrentKeyFrame->GetTranslation();
    auto mvpObjectDetections = mpCurrentKeyFrame->GetObjectDetections();
    auto mvpAssociatedObjects = mpCurrentKeyFrame->GetMapObjectMatches();

    // std::cout << " => " << "mvpObjectDetections.size() = " << mvpObjectDetections.size() << std::endl;
    std::cout << " => " << "KF" << mpCurrentKeyFrame->mnId << ", mvpObjectDetections.size() = " << mvpObjectDetections.size() << std::endl;


    // 处理当前帧的所有detection

    for (int det_i = 0; det_i < mvpObjectDetections.size(); det_i++)
    {
        // std::cout << "det_i " << det_i << std::endl;
        // std::cout << "Press [ENTER] to continue ... " << std::endl;
        // key = getchar();

        auto det = mvpObjectDetections[det_i];

        // If the detection is associated with an existing map object, we consider 2 different situations:
        // 1. the object has been reconstructed: update observations 
        // 2. the object has not been reconstructed:
        // check if it's ready for reconstruction, reconstruct if it's got enough points

        if (det->isNew) {
            std::cout << "  Conitinue because det->isNew" << std::endl;
            continue;
        }

        if (!det->isGood) {
            std::cout << "  Conitinue because !det->isGood" << std::endl;
            continue;
        }

        // 只处理有关联物体的观测
        MapObject *pMO = mvpAssociatedObjects[det_i];
        if (!pMO) {
            std::cout << "  Conitinue because !pMO" << std::endl;
            continue;
        }

        // We only consider the object in the middle
        if (pMO->mnId != 0) {
            std::cout << "  Conitinue because pMO->mnId != 0" << std::endl;
            continue;
        }

        int numKFsPassedSinceInit = int(mpCurrentKeyFrame->mnId - pMO->mpRefKF->mnId);

        // bool isShortInternal = numKFsPassedSinceInit < 15;
        // std::string condition_str = "numKFsPassedSinceInit < 15";
        bool isShortInternal = numKFsPassedSinceInit < 15 and (mpCurrentKeyFrame->mnId >= 3);
        std::string condition_str = "numKFsPassedSinceInit < 15 and (mpCurrentKeyFrame->mnId >= 3)";

        // 只在前50帧进行PCA初始位姿估计
        if (numKFsPassedSinceInit < 50) {
            std::cout << "ComputeCuboidPCA" << std::endl;
            pMO->ComputeCuboidPCA(numKFsPassedSinceInit < 15);
        }
        else { // when we have relative good object shape
            pMO->RemoveOutliersModel();
        }
        
        // only begin to reconstruct the object if it is observed for enough amoubt of time (15 KFs)
        if(isShortInternal) {
            std::cout << "  Conitinue because " << condition_str << std::endl;
            continue;
        }

        if ((numKFsPassedSinceInit - 15) % 5 != 0) {
            std::cout << "  Conitinue because (numKFsPassedSinceInit - 15) % 5 != 0" << std::endl;
            continue;
        }

//        int numKFsPassedSinceLastRecon = int(mpCurrentKeyFrame->mnId) - nLastReconKFID;
//        if (numKFsPassedSinceLastRecon  < 8)
//            continue;

        std::vector<MapPoint*> points_on_object = pMO->GetMapPointsOnObject();
        int n_points = points_on_object.size();


        int n_valid_points = 0;
        

        for (auto pMP : points_on_object)
        {
            if (!pMP)
                continue;
            if (pMP->isBad())
                continue;
            if (pMP->isOutlier())
                continue;
            n_valid_points++;
        }
        
        // 记录物体上的关键点的数量
        int n_rays = 0;
        auto map_points_vector = mpCurrentKeyFrame->GetMapPointMatches();
        for (auto idx : det->GetFeaturePoints())
        {
            auto pMP = map_points_vector[idx];
            if (!pMP)
                continue;
            if (pMP->isBad())
                continue;
            if (pMP->object_id != pMO->mnId)
                continue;
            if (pMP->isOutlier())
                continue;
            n_rays++;
        }
        // cout << "Object " << pMO->mnId << ": " << n_points << " points observed, " << "with " << n_valid_points << " valid points, and " << n_rays << " rays" << endl;


        // Surface points
        std::cout << " =>" << "n_valid_points: " << n_valid_points << ", n_rays: " << n_rays << std::endl;
        if (n_valid_points >= 50 && n_rays > 20)
        {   
            //！获取surface_points_cam
            Eigen::MatrixXf surface_points_cam = Eigen::MatrixXf::Zero(n_valid_points, 3);
            int p_i = 0;
            for (auto pMP : points_on_object)
            {
                if (!pMP)
                    continue;
                if (pMP->isBad())
                    continue;
                if (pMP->isOutlier())
                    continue;

                cv::Mat x3Dw = pMP->GetWorldPos();
                cv::Mat x3Dc = Rcw * x3Dw + tcw;
                float xc = x3Dc.at<float>(0);
                float yc = x3Dc.at<float>(1);
                float zc = x3Dc.at<float>(2);
                surface_points_cam(p_i, 0) = xc;
                surface_points_cam(p_i, 1) = yc;
                surface_points_cam(p_i, 2) = zc;
                p_i++;
            }

            //！ 获取ray_pixels和depth_obs
            Eigen::MatrixXf ray_pixels = Eigen::MatrixXf::Zero(n_rays, 2);
            Eigen::VectorXf depth_obs = Eigen::VectorXf::Zero(n_rays);

            int k_i = 0;
            for (auto point_idx : det->GetFeaturePoints())
            {
                auto pMP = map_points_vector[point_idx];
                if (!pMP)
                    continue;
                if(pMP->isBad())
                    continue;
                if(pMP->object_id != pMO->mnId)
                    continue;
                if (pMP->isOutlier())
                    continue;

                cv::Mat x3Dw = pMP->GetWorldPos();
                cv::Mat x3Dc = Rcw * x3Dw + tcw;
                depth_obs(k_i) = x3Dc.at<float>(2);
                ray_pixels(k_i, 0) = mpCurrentKeyFrame->mvKeysUn[point_idx].pt.x;
                ray_pixels(k_i, 1 ) = mpCurrentKeyFrame->mvKeysUn[point_idx].pt.y;
                k_i++;
            }

            // 像素点的归一化的向量
            Eigen::MatrixXf u_hom(n_rays, 3);
            u_hom << ray_pixels, Eigen::MatrixXf::Ones(n_rays, 1);

            // 转换到相机坐标系的射线向量
            Eigen::MatrixXf fg_rays(n_rays, 3);
            Eigen::Matrix3f invK = Converter::toMatrix3f(mpTracker->GetCameraIntrinsics()).inverse();
            
            for (int i = 0; i  < n_rays; i++)
            {
                auto x = u_hom.row(i).transpose();
                fg_rays.row(i) = (invK * x).transpose();
            }

            Eigen::MatrixXf rays(fg_rays.rows() + det->background_rays.rows(), 3);
            rays << fg_rays, det->background_rays;
            
            std::cout << "Ready to reconstruct_object" << std::endl;
            // std::cout << "Press [ENTER] to continue ... " << std::endl;
            // key = getchar();

            PyThreadStateLock PyThreadLock;

            std::cout << "Before reconstruct_object" << std::endl;

            std::cout << "SE3Tcw = \n" << SE3Tcw.matrix() << std::endl;
            std::cout << "pMO->Sim3Two = \n" << pMO->Sim3Two.matrix() << std::endl;

            auto pyMapObject = pyOptimizer.attr("reconstruct_object")
                    (SE3Tcw * pMO->Sim3Two, surface_points_cam, rays, depth_obs, pMO->vShapeCode);
            std::cout << "After reconstruct_object" << std::endl;

            // cout << "Number of KF passed: " << numKFsPassedSinceInit << endl;

            // std::cout << "pMO->reconstructed = " << pMO->reconstructed << std::endl;
            // If not initialized, duplicate optimization to resolve orientation ambiguity
            if (!pMO->reconstructed)
            {
                // 重建失败
                // std::cout << "Reconstruction failed" << std::endl;
                // continue;
                
                // 这里后面的代码会让定位失效

                auto flipped_Two = pMO->Sim3Two;
                flipped_Two.col(0) *= -1;
                flipped_Two.col(2) *= -1;
                auto pyMapObjectFlipped = pyOptimizer.attr("reconstruct_object")
                        (SE3Tcw * flipped_Two, surface_points_cam, rays, depth_obs, pMO->vShapeCode);

                if (pyMapObject.attr("loss").cast<float>() > pyMapObjectFlipped.attr("loss").cast<float>())
                    pyMapObject = pyMapObjectFlipped;
            }

            // std::cout << "pMO->reconstructed = " << pMO->reconstructed << std::endl;

            std::cout << "Reconstruction successed" << std::endl;

            auto Sim3Tco = pyMapObject.attr("t_cam_obj").cast<Eigen::Matrix4f>();
            det->SetPoseMeasurementSim3(Sim3Tco);
            // Sim3, SE3, Sim3
            Eigen::Matrix4f Sim3Two = SE3Twc * Sim3Tco;

            std::cout << "Sim3Two = " << Sim3Two.matrix() << std::endl;

            int code_len = pyOptimizer.attr("code_len").cast<int>();
            Eigen::Vector<float, 64> code = Eigen::VectorXf::Zero(64);
            if (code_len == 32)
            {
                auto code_32 = pyMapObject.attr("code").cast<Eigen::Vector<float, 32>>();
                code.head(32) = code_32;
            }
            else
            {
                code = pyMapObject.attr("code").cast<Eigen::Vector<float, 64>>();
            }

            pMO->UpdateReconstruction(Sim3Two, code);
            auto pyMesh = pyMeshExtractor.attr("extract_mesh_from_code")(code);
            pMO->vertices = pyMesh.attr("vertices").cast<Eigen::MatrixXf>();
            pMO->faces = pyMesh.attr("faces").cast<Eigen::MatrixXi>();
            pMO->reconstructed = true;
            pMO->AddObservation(mpCurrentKeyFrame, det_i);
            mpCurrentKeyFrame->AddMapObject(pMO, det_i);
            mpObjectDrawer->AddObject(pMO);
            std::cout << "mpObjectDrawer->AddObject(pMO);" << std::endl;

            mlpRecentAddedMapObjects.push_back(pMO);

            nLastReconKFID = int(mpCurrentKeyFrame->mnId);

            // std::cout << "Finish reconstructing one object" << std::endl;

            // std::cout << "Press [ENTER] to continue ... " << std::endl;
            // key = getchar();
        }
    }
}

}
