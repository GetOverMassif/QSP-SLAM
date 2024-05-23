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

#include "FrameDrawer.h"
#include "Tracking.h"

#include "utils/dataprocess_utils.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mutex>

namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));

    mmRGB = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
    mmDepth = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

void FrameDrawer::setTracker(Tracking* tracker)
{
    mpTracking = tracker;
}

cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    vector<cv::Mat> vmasks;
    vector<vector<int>> vbboxs;

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);

        state=mState;

        for (auto mask: mvImObjectMasks) {
            vmasks.push_back(mask);
        }

        for (auto bbox: mvImObjectBboxs) {
            vbboxs.push_back(bbox);
        }

        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);
    
    for (auto &mask: vmasks) {
        cv::Mat mask_rgb = cv::Mat::zeros(mask.rows, mask.cols, CV_8UC3);
        cv::Mat mask_grey = cv::Mat::zeros(mask.rows, mask.cols, CV_8UC1);
        mask.convertTo(mask, CV_8U);
        // cout << "       im info : " << im.rows << "," << im.cols << "," << im.type() << endl;
        // cout << "     mask info : " << mask.rows << "," << mask.cols << "," << mask.type() << endl;
        // cout << " mask_rgb info : " << mask_rgb.rows << "," << mask_rgb.cols << "," << mask_rgb.type() << endl;
        // cout << "mask_grey info : " << mask_grey.rows << "," << mask_grey.cols << "," << mask_grey.type() << endl;
        vector<cv::Mat> channels;
        channels.push_back(mask_grey);
        channels.emplace_back(mask_grey);
        channels.emplace_back(mask);
        merge(channels, mask_rgb);
        cv::addWeighted(im, 1, mask_rgb, 0.2, 0.0, im);
    }

    for (auto &bbox: vbboxs) {
        int x1 = bbox[0], y1 = bbox[1], x2 = bbox[2], y2 = bbox[3];
        cv::rectangle(im, cv::Point2f(float(x1), float(y1)), cv::Point2f(float(x2), float(y2)), \
                      cv::Scalar(0,0,200));
    }

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }        
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im, state, imWithInfo);

    mmRGB = imWithInfo.clone();

    return mmRGB;

    // return imWithInfo;
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame->mvKeys;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;
    mvImObjectMasks.clear();
    mvImObjectBboxs.clear();

    for (auto img: pTracker->mvImObjectMasks) {
        mvImObjectMasks.push_back(img);
    }

    for (auto bbox: pTracker->mvImObjectBboxs) {
        mvImObjectBboxs.push_back(bbox);
    }

    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        cout << "size of pTracker->mInitialFrame->mvKeys = " << pTracker->mInitialFrame->mvKeys.size() << endl;
        mvIniKeys=pTracker->mInitialFrame->mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame->mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame->mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}


cv::Mat FrameDrawer::drawFrameOnImage(cv::Mat &in) {
    cv::Mat out = in.clone();
    out = drawObservationOnImage(out, true);
    // out = drawProjectionOnImage(out);
    // out = draw3DProjectionOnImage(out);
    out = drawVisualEllipsoidsProjectionOnImage(out);
    return out.clone();
}

cv::Mat FrameDrawer::drawRGBFrame() {
    Frame * frame = mpTracking->mCurrentFrame;
    if( frame == NULL ) return cv::Mat();
    cv::Mat im;
    if(!frame->rgb_img.empty()) // use rgb image if it exists, or use depth image instead.
        im = frame->rgb_img;
    else 
        im = frame->frame_img;

    cv::Mat out = drawFrameOnImage(im);

    mmRGB = out.clone();

    return mmRGB;
}

cv::Mat FrameDrawer::drawDepthFrame() {
    Frame * frame = mpTracking->mCurrentFrame;
    cv::Mat I = frame->frame_img;   // U16C1 , ushort
    cv::Mat im,R,G,B;

    double min;
    double max;
    cv::minMaxIdx(I, &min, &max);

    I.convertTo(im,CV_8UC1, 255 / (max-min), -min/(max-min)*255); 

    Vector3d color1(51,25,0);
    Vector3d color2(255,229,204);
    // Vector3d color1(255,20,0);
    // Vector3d color2(0,20,255);

    Vector3d scale_color = (color2-color1) / 255.0;
    // r = color1 + value/255*(color2-color1)
    // (255-51)/255   51
    // 220-25

    // ********************************************
    // Vector3d color1(0,204,102);
    // Vector3d color2(255,204,102);
    // double depth_thresh = 8;
    // double scale = Config::Get<double>("Camera.scale"); 

    // Vector3d scale_param = (color2-color1) / scale / depth_thresh;
    // // I / scale / depth_thresh   (0-1)   * (color2-color1) + color 1

    // I.convertTo(B, CV_8UC1, scale_param[0], color1[0]);
    // I.convertTo(G, CV_8UC1, scale_param[1], color1[1]);
    // I.convertTo(R, CV_8UC1, scale_param[2], color1[2]);
    // *********************************************

    im.convertTo(R, CV_8UC1, scale_color[0], color1[0]);
    im.convertTo(G, CV_8UC1, scale_color[1], color1[1]);
    im.convertTo(B, CV_8UC1, scale_color[2], color1[2]);

    std::vector<cv::Mat> array_to_merge;
    array_to_merge.push_back(B);
    array_to_merge.push_back(G);
    array_to_merge.push_back(R);
    cv::merge(array_to_merge, im);

    mmDepth = im.clone();

    // cv::Mat out = drawObservationOnImage(im, false);
    // mmDepth = out.clone();

    return mmDepth;
}

cv::Mat drawPointCloudOnImage(cv::Mat& im, PointCloud& cloud, const Vector3d& color, g2o::SE3Quat& trans, Matrix3d& calib)
{
    cv::Mat out = im.clone();
    if(cloud.size()==0) return out;
    PointCloud* pCloud_local = transformPointCloud(&cloud, &trans);

    // 局部点云投影到二维坐标
    int num = pCloud_local->size();
    for( int i=0;i<num;i++)
    {
        PointXYZRGB& p = (*pCloud_local)[i];
        // Vector4d xyz_homo; xyz_homo << p.x, p.y, p.z, 1;
        Vector3d xyz; xyz << p.x, p.y, p.z;
        Vector3d uv_homo = calib * xyz;
        Vector2d uv = uv_homo.head(2) / uv_homo[2];

        int x = round(uv[0]);
        int y = round(uv[1]);

        // 绘制到图像上
        if(color[0] >= 0)
            out.at<cv::Vec3b>(y,x) = cv::Vec3b(color[2]*255, color[1]*255, color[0]*255);
        else 
            out.at<cv::Vec3b>(y,x) = cv::Vec3b(p.b, p.g, p.r);
    }

    return out;
}

cv::Mat FrameDrawer::drawDepthFrameWithVisualPoints() {
    // cv::Mat depthMat = mmDepth;
    cv::Mat depthMat = drawDepthFrame();

    Matrix3d calib = mpTracking->mCalib;
    g2o::SE3Quat campose_cw = mpTracking->mCurrentFrame->cam_pose_Tcw;

    // ----- 绘制投影的点云
    // 得到将要绘制的点云
    PointCloud cloud = mpMap->GetPointCloudInList("EllipsoidExtractor.EuclideanFiltered");
    PointCloud cloud_plane = mpMap->GetPointCloudInList("pPlaneExtractorManhattan.MH-Plane Points");
    // 要求先变换回局部坐标系, 然后投影
    cv::Mat out = drawPointCloudOnImage(depthMat, cloud, Vector3d(0,0,1.0), campose_cw, calib);
    out = drawPointCloudOnImage(out, cloud_plane, Vector3d(-1,0,0), campose_cw, calib); // -1 use own color
    
    mmDepth = out;
    return out;
}

cv::Mat FrameDrawer::drawVisualEllipsoidsProjectionOnImage(cv::Mat &im) {
    std::vector<g2o::ellipsoid*> vElps = mpMap->GetAllEllipsoidsVisual();

    // draw projected bounding boxes of the ellipsoids in the map
    cv::Mat imageProj = im.clone();
    for(auto iter=vElps.begin(); iter!=vElps.end();iter++)
    {
        ellipsoid* e = *iter;
        if(e==NULL) continue;

        // Check Prob!
        double config_prob_thresh = Config::ReadValue<double>("Dynamic.Optimizer.EllipsoidProbThresh");
        if(e->prob < config_prob_thresh) continue;

        // check whether it could be seen
        if( e->CheckObservability(mpTracking->mCurrentFrame->cam_pose_Tcw) )
        {
            // draw rectangle
            // Vector4d rect = e->getBoundingBoxFromProjection(mpTracking->mCurrFrame->cam_pose_Tcw, mpTracking->mCalib); // center, width, height
            // cv::rectangle(imageProj, cv::Rect(cv::Point(rect[0],rect[1]),cv::Point(rect[2],rect[3])), cv::Scalar(255,0,255), 2);
            
            // draw ellipse
            Vector5d ellipse = e->projectOntoImageEllipse(mpTracking->mCurrentFrame->cam_pose_Tcw, mpTracking->mCalib);
            Vector3d color = e->getColor() * 255;
            cv::RotatedRect rotbox2(cv::Point2f(ellipse[0],ellipse[1]), cv::Size2f(ellipse[3]*2,ellipse[4]*2), ellipse[2]/M_PI*180);
            try
            {
                cv::ellipse(imageProj, rotbox2, cv::Scalar(color[2], color[1], color[0]), 3);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }
    }

    return imageProj.clone();
}

const char* GetLabelText(int id)
{
    static const char *coco_classes[] = {"person","bicycle","car","motorcycle","airplane","bus","train",
    "truck","boat","traffic light","fire hydrant","stop sign","parking meter","bench","bird",
    "cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack","umbrella",
    "handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite","baseball bat",
    "baseball glove","skateboard","surfboard","tennis racket","bottle","wine glass","cup","fork",
    "knife","spoon","bowl","banana","apple","sandwich","orange","broccoli","carrot","hot dog",
    "pizza","donut","cake","chair","couch","potted plant","bed","dining table","toilet","monitor",
    "laptop","mouse","remote","keyboard","cell phone","microwave","oven","toaster","sink",
    "refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"};

    return coco_classes[id];
}

cv::Mat FrameDrawer::drawObservationOnImage(cv::Mat &in, bool draw_text) {
    cv::Mat im = in.clone();

    Frame * frame = mpTracking->mCurrentFrame;
    // draw observation
    Eigen::MatrixXd mat_det = frame->mmObservations;
    int obs = mat_det.rows();
    
    int fontFace = CV_FONT_HERSHEY_SIMPLEX;

    double size_scale = in.cols / 960.0;
    double fontScale = 1.3 * size_scale;
    int thickness = 2;
    for(int r=0;r<obs;r++){
        VectorXd vDet = mat_det.row(r);

        Vector4d measure; measure << vDet(1), vDet(2), vDet(3), vDet(4);
        bool is_border = calibrateMeasurement(measure, im.rows, im.cols, Config::Get<int>("Measurement.Border.Pixels"), Config::Get<int>("Measurement.LengthLimit.Pixels"));

        int labelId = int(vDet(5));
        cv::Rect2d rec(cv::Point2d(vDet(1), vDet(2)), cv::Point2d(vDet(3), vDet(4)));

        double prob = vDet(6);
        double prob_thresh = Config::Get<double>("Measurement.Probability.Thresh");
        bool prob_check = (prob > prob_thresh);

        int line_size = 3;
        cv::Scalar color = cv::Scalar(0,255,0);

        // 若该检测在边界，则灰度显示
        if( is_border )
        {
            // line_size = 3;
            color = cv::Scalar(128,128,128);
        }
        
        rectangle(im, rec, color, line_size);

        cv::Point2d textOrg = cv::Point2d(vDet(1), vDet(2));
        
        if(draw_text){
            // prob
            std::stringstream ss;
            ss << std::setprecision(2) << prob;
            string str_prob = ss.str();  
            // label
            const char* labelText = GetLabelText(labelId);
            string str_labelText(labelText);
            // total
            string str_id_text_prob = to_string(labelId) + " " + str_labelText + " [" + str_prob + "]";
            int baseline = 0;
            cv::Size2d textSize = cv::getTextSize(str_id_text_prob, fontFace, fontScale, thickness, &baseline);

            // draw
            cv::rectangle(im, textOrg + cv::Point2d(0, baseline), textOrg + cv::Point2d(textSize.width, -textSize.height), cv::Scalar(0, 255, 0), -1);
            cv::putText(im, str_id_text_prob, textOrg, fontFace, fontScale, cv::Scalar(255,0,0), thickness);
        }
        else
        {
            // 如果没有draw text, 则draw id: 即在bboxMat中的位置
            string str_id = to_string(r);
            int baseline = 0;
            cv::Size textSize = cv::getTextSize(str_id, fontFace, fontScale, thickness, &baseline);

            rectangle(im, textOrg + cv::Point2d(0, baseline), textOrg + cv::Point2d(textSize.width, -textSize.height), cv::Scalar(0, 255, 0), -1);
            putText(im, str_id, textOrg, fontFace, fontScale, cv::Scalar(255,0,0), thickness);
        }
    }

    return im.clone();
}

cv::Mat FrameDrawer::getCurrentFrameImage(){
    return mmRGB;
}

cv::Mat FrameDrawer::getCurrentDepthFrameImage(){
    return mmDepth;
}

} //namespace ORB_SLAM
