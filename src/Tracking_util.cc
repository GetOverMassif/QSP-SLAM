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

#include "Tracking.h"
#include "ObjectDetection.h"
#include "ORBmatcher.h"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

using namespace std;

namespace ORB_SLAM2 {

/*
 * Tracking utils for stereo+lidar on KITTI
 */
void Tracking::GetObjectDetectionsLiDAR(KeyFrame *pKF) {

    PyThreadStateLock PyThreadLock;

    std::string frame_name = mvstrImageFilenamesRGB[pKF->mnFrameId];

    std::cout << "frame_name = " << frame_name << std::endl;

    // py::list detections = mpSystem->pySequence.attr("get_frame_by_id")(pKF->mnFrameId);
    py::list detections = mpSystem->pySequence.attr("get_frame_by_name")(pKF->mnFrameId, frame_name);

    for (auto det : detections) {
        auto pts = det.attr("surface_points").cast<Eigen::MatrixXf>();
        auto Sim3Tco = det.attr("T_cam_obj").cast<Eigen::Matrix4f>();
        auto rays = det.attr("rays");
        Eigen::MatrixXf rays_mat;
        Eigen::VectorXf depth;

        if (rays.is_none()) {
            // std::cout << "No 2D masks associated!" << std::endl;
            rays_mat = Eigen::Matrix<float, 0, 0>::Zero();
            depth = Eigen::Vector<float, 0>::Zero();
        } else {
            rays_mat = rays.cast<Eigen::MatrixXf>();
            depth = det.attr("depth").cast<Eigen::VectorXf>();
        }
        // Create C++ detection instance
        auto o = new ObjectDetection(Sim3Tco, pts, rays_mat, depth);
        pKF->mvpDetectedObjects.push_back(o);
    }
    pKF->nObj = pKF->mvpDetectedObjects.size();
    pKF->mvpMapObjects = vector<MapObject *>(pKF->nObj, static_cast<MapObject *>(NULL));
}

void Tracking::ObjectDataAssociation(KeyFrame *pKF)
{
    vector<MapObject *> vpLocalMapObjects;
    // Loop over all the local frames to find matches
    for (KeyFrame *plKF : mvpLocalKeyFrames)
    {
        vector<MapObject *> vpMOs = plKF->GetMapObjectMatches();
        for (MapObject *pMO : vpMOs)
        {
            if (pMO)
            {
                // Prevent multiple association to the same object
                if (pMO->mnAssoRefID != pKF->mnId)
                {
                    vpLocalMapObjects.push_back(pMO);
                    pMO->mnAssoRefID = pKF->mnId;
                }
            }
        }
    }
    if (vpLocalMapObjects.empty())
        return;

    Eigen::Matrix4f Tcw = Converter::toMatrix4f(mCurrentFrame.mTcw);
    Eigen::Matrix3f Rcw = Tcw.topLeftCorner<3, 3>();
    Eigen::Vector3f tcw = Tcw.topRightCorner<3, 1>();
    auto vDetections = pKF->mvpDetectedObjects;
    // loop over all the detections.
    for (int i = 0; i < pKF->nObj; i++)
    {
        auto det = vDetections[i];
        Eigen::Vector3f transDet = det->tco;
        vector<float> dist;

        for (auto pObj : vpLocalMapObjects)
        {
            if (!pObj || pObj->isBad())
            {
                dist.push_back(1000.0);
                continue;
            }

            if (!pObj->isDynamic()) {
                Eigen::Vector3f dist3D = Rcw * pObj->two + tcw - transDet;
                Eigen::Vector2f dist2D;
                dist2D << dist3D[0], dist3D[2];
                dist.push_back((dist2D).norm());
            }
            else
            {
                float deltaT = (float) (mCurrentFrame.mnId - mpLastKeyFrame->mnFrameId);
                auto twoPredicted = pObj->two + pObj->velocity * deltaT;
                Eigen::Vector3f dist3D = Rcw * twoPredicted + tcw - transDet;
                Eigen::Vector2f dist2D;
                dist2D << dist3D[0], dist3D[2];
                dist.push_back((dist2D).norm());
            }
        }
        float minDist = *min_element(dist.begin(), dist.end());

        // Start with a loose threshold
        if (minDist < 5.0)
        {
            det->isNew = false;
            if (det->nPts < 25)
                det->isGood = false;

            int idx = min_element(dist.begin(), dist.end()) - dist.begin();
            MapObject *pMO = vpLocalMapObjects[idx];
            if (!pKF->mdAssociatedObjects.count(pMO)) {
                pKF->mdAssociatedObjects[pMO] = minDist;
                pKF->AddMapObject(pMO, i);
                pMO->AddObservation(pKF, i);
            } else // Another detection is associated with pMO, compare distance
            {
                if (minDist < pKF->mdAssociatedObjects[pMO]) {
                    // cout << "Associated to: " << pMO->mnId << ", Distance: " << minDist << endl;
                    pKF->mdAssociatedObjects[pMO] = minDist;
                    int detId = pMO->GetObservations()[pKF];
                    pKF->EraseMapObjectMatch(detId);
                    vDetections[detId]->isNew = true;
                    pKF->AddMapObject(pMO, i);
                    pMO->AddObservation(pKF, i);
                }
            }
        }
        else
        {
            det->isNew = true;
            if (det->nPts < 50)
                det->isGood = false;
        }
    }
}

/*
 * Tracking utils for monocular input on Freiburg Cars and Redwood OS
 */
cv::Mat Tracking::GetCameraIntrinsics()
{
    return mK;
}

void Tracking::GetObjectDetectionsMono(KeyFrame *pKF)
{
    std::cout << "[ Tracking - GetObjectDetectionsMono ]" << std::endl;
    // => Step 1: 准备Python线程锁，清空物体 mask 的 vector
    PyThreadStateLock PyThreadLock;
    mvImObjectMasks.clear();

    // => Step 2: 获取帧号/帧名称，调用物体检测
    std::string frame_name = mvstrImageFilenamesRGB[pKF->mnFrameId];
    // std::cout << "frame_name = " << frame_name << std::endl;
    // py::list detections = mpSystem->pySequence.attr("get_frame_by_id")(pKF->mnFrameId);
    py::list detections = mpSystem->pySequence.attr("get_frame_by_name")(pKF->mnFrameId, frame_name);

    std::cout << " => " << "detections.size() = " << detections.size() << std::endl;

    if (detections.size() == 0) // No detections, return immediately 
    {
        std::cout << "Return because detections.size() == 0" << std::endl;
        return;
    }

    // => Step 3: 遍历这些检测结果
    for (int detected_idx = 0; detected_idx < detections.size(); detected_idx++)
    {
        // => Step 3.1: 创建物体检测类，暂时保存mask，
        //              并将ray、bbox和mask内的关键点保存到检测实例中
        auto det = new ObjectDetection();
        auto py_det = detections[detected_idx];
        det->background_rays = py_det.attr("background_rays").cast<Eigen::MatrixXf>();
        auto mask = py_det.attr("mask").cast<Eigen::MatrixXf>();

        cv::Mat mask_cv;
        cv::eigen2cv(mask, mask_cv);
        // cv::imwrite("mask.png", mask_cv);
        cv::Mat mask_erro = mask_cv.clone();
        cv::Mat kernel = getStructuringElement(cv::MORPH_ELLIPSE,
                                               cv::Size(2 * maskErrosion + 1, 2 * maskErrosion + 1),
                                               cv::Point(maskErrosion, maskErrosion));
        cv::erode(mask_cv, mask_erro, kernel);

        mvImObjectMasks.push_back(std::move(mask_cv));

        // get 2D feature points inside mask
        for (int i = 0; i < pKF->mvKeys.size(); i++)
        {
            int val = (int) mask_erro.at<float>(pKF->mvKeys[i].pt.y, pKF->mvKeys[i].pt.x);
            if (val > 0)  // inside the mask
            {
                det->AddFeaturePoint(i);
            }
        }

        // Step 3.2: 将少于20个关键点的det定义为isGood = false，
        //           将det加入当前帧物体检测vector中

        // Reject the detection if too few keypoints are extracted
        if (det->NumberOfPoints() < 20)
        {
            std::cout << " => det" << detected_idx << "->NumberOfPoints() < 20" << std::endl;
            det->isGood = false;
        }
        pKF->mvpDetectedObjects.push_back(det);
        std::cout << " => " << "pKF->mvpDetectedObjects.push_back(det)" << std::endl;
    }

    auto mvpObjectDetections = pKF->GetObjectDetections();

    std::cout << " => " << "KF" << pKF->mnId << ", mvpObjectDetections.size() = " << mvpObjectDetections.size() << std::endl;

    // => Step 4: 记录当前帧的物体检测数量，预留同样数量的物体Vector
    pKF->nObj = pKF->mvpDetectedObjects.size();
    pKF->mvpMapObjects = vector<MapObject *>(pKF->nObj, static_cast<MapObject *>(NULL));
}


void Tracking::GetObjectDetectionsRGBD(KeyFrame *pKF)
{

    PyThreadStateLock PyThreadLock;

    mvImObjectMasks.clear();

    std::string frame_name = mvstrImageFilenamesRGB[pKF->mnFrameId];

    // std::cout << "frame_name = " << frame_name << std::endl;

    // py::list detections = mpSystem->pySequence.attr("get_frame_by_id")(pKF->mnFrameId);

    // Get a series of object detections
    py::list detections = mpSystem->pySequence.attr("get_frame_by_name")(pKF->mnFrameId, frame_name);

    int num_dets = detections.size();
    // No detections, return immediately
    if (num_dets == 0)
        return;

    std::cout << "Detects " << num_dets << " objects Observations." << std::endl;
    for (int detected_idx = 0; detected_idx < num_dets; detected_idx++)
    {
        auto det = new ObjectDetection();
        auto py_det = detections[detected_idx];
        det->background_rays = py_det.attr("background_rays").cast<Eigen::MatrixXf>();
        auto mask = py_det.attr("mask").cast<Eigen::MatrixXf>();
        auto bbox = py_det.attr("bbox").cast<Eigen::MatrixXf>();


        cv::Mat mask_cv;
        cv::eigen2cv(mask, mask_cv);
        // cv::imwrite("mask.png", mask_cv);
        cv::Mat mask_erro = mask_cv.clone();
        // std::cout << "maskErrosion = " << maskErrosion << std::endl;
        cv::Mat kernel = getStructuringElement(cv::MORPH_ELLIPSE,
                                               cv::Size(2 * maskErrosion + 1, 2 * maskErrosion + 1),
                                               cv::Point(maskErrosion, maskErrosion));
                                            //    cv::Point(-1, -1));

        
        cv::erode(mask_cv, mask_erro, kernel);

        mvImObjectMasks.push_back(std::move(mask_cv));
        
        // get 2D feature points inside mask
        for (int i = 0; i < pKF->mvKeys.size(); i++)
        {
            int val = (int)mask_erro.at<float>(pKF->mvKeys[i].pt.y, pKF->mvKeys[i].pt.x);
            if (val > 0)  // inside the mask
            {
                det->AddFeaturePoint(i);
            }
        }

        // Reject the detection if too few keypoints are extracted
        // todo: 设置点数量数量而不是isGood
        if (det->NumberOfPoints() < 20)
        {
            det->isGood = false;
        }

        pKF->mvpDetectedObjects.push_back(det);
    }
    pKF->nObj = pKF->mvpDetectedObjects.size();
    pKF->mvpMapObjects = vector<MapObject *>(pKF->nObj, static_cast<MapObject *>(NULL));
}

void Tracking::AssociateObjectsByProjection(ORB_SLAM2::KeyFrame *pKF)
{
    std::cout << "[ Tracking - AssociateObjectsByProjection ]" << std::endl;
    // => Step 1: 获取该关键帧关联的地图点、物体检测
    auto mvpMapPoints = pKF->GetMapPointMatches();
    // Try to match and triangulate key-points with last key-frame
    auto detectionsKF1 = pKF->mvpDetectedObjects;
    
    // => Step 2: 遍历所有检测结果
    for (int d_i = 0; d_i < detectionsKF1.size(); d_i++)
    {
        // Step 2.1: 遍历检测关联的地图点，判断其是否合法且为内点
        //           记录这些地图点中已经关联的物体编号
        // cout << "Detection: " << d_i + 1 << endl;
        auto detKF1 = detectionsKF1[d_i];
        map<int, int> observed_object_id;
        int nOutliers = 0;

        for (int k_i : detKF1->GetFeaturePoints()) {
            auto pMP = mvpMapPoints[k_i];
            if (!pMP)
                continue;
            if (pMP->isOutlier())
            {
                nOutliers++;
                continue;
            }

            // 在初始观测时都不加入具体的object_id，即为-1
            if (pMP->object_id < 0)
                continue;

            if (observed_object_id.count(pMP->object_id))
                observed_object_id[pMP->object_id] += 1;
            else
                observed_object_id[pMP->object_id] = 1;
        }

        // Step 2.2: 根据关键点数量寻找与哪个object编号更匹配
        //           获取对应的物体指针，将物体、检测添加到关键帧，
        //           设置该 det->isNew = false
        //           将其他的检测关联地图点设置与物体关联
        if (!observed_object_id.empty())
        {
            // Find object that has the most matches
            int object_id_max_matches = 0;  // global object id
            int max_matches = 0;
            for (auto it = observed_object_id.begin(); it != observed_object_id.end(); it++) {
                if (it->second > max_matches) {
                    max_matches = it->second;
                    object_id_max_matches = it->first;
                }
            }

            // associated object
            // todo: 这里的关联方式过于粗糙，只要有相同地图点就关联到一起了
            auto pMO = mpMap->GetMapObject(object_id_max_matches);
            pKF->AddMapObject(pMO, d_i);
            detKF1->isNew = false;

            // add newly detected feature points to object
            int newly_matched_points = 0;
            for (int k_i : detKF1->GetFeaturePoints()) {
                auto pMP = mvpMapPoints[k_i];
                if (pMP && !pMP->isBad())
                {
                    // new map points
                    if (pMP->object_id < 0)
                    {
                        pMP->in_any_object = true;
                        pMP->object_id = object_id_max_matches;
                        pMO->AddMapPoints(pMP);
                        newly_matched_points++;
                    }
                    else
                    {
                        // if pMP is already associate to a different object, set bad flag
                        // 一个特征点在不同帧可以在不同物体的mask内
                        if (pMP->object_id != object_id_max_matches)
                            pMP->SetBadFlag();
                    }
                }
            }
            cout <<  "Matches: " << max_matches << ", New points: " << newly_matched_points << ", Keypoints: " <<
                 detKF1->mvKeysIndices.size() << ", Associated to object by projection " << object_id_max_matches
                 << endl << endl;
            /*cout <<  "Matches: " << max_matches << ", New points: " << newly_matched_points << ", Keypoints: " <<
                 detKF1->mvKeysIndices.size() << ", Associated to object by projection " << object_id_max_matches
                 << endl << endl;*/
        }

    }
}

void Tracking::SetImageNames(vector<string>& vstrImageFilenamesRGB)
{
    mvstrImageFilenamesRGB.resize(vstrImageFilenamesRGB.size());
    mvstrImageFilenamesRGB = std::vector<string>(vstrImageFilenamesRGB.begin(), vstrImageFilenamesRGB.end());
}

}