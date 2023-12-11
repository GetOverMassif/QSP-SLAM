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

#include "MapDrawer.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM2
{

    // draw axis for ellipsoids
    void drawAxisNormal()
    {
        float length = 2.0;
        
        // x
        glColor3f(1.0,0.0,0.0); // red x
        glBegin(GL_LINES);
        glVertex3f(0.0, 0.0f, 0.0f);
        glVertex3f(length, 0.0f, 0.0f);
        glEnd();
    
        // y 
        glColor3f(0.0,1.0,0.0); // green y
        glBegin(GL_LINES);
        glVertex3f(0.0, 0.0f, 0.0f);
        glVertex3f(0.0, length, 0.0f);
    
        glEnd();
    
        // z 
        glColor3f(0.0,0.0,1.0); // blue z
        glBegin(GL_LINES);
        glVertex3f(0.0, 0.0f ,0.0f );
        glVertex3f(0.0, 0.0f ,length );
    
        glEnd();
    }

    // In : Tcw
    // Out: Twc
    // void MapDrawer::SE3ToOpenGLCameraMatrix(g2o::SE3Quat &matInSe3, pangolin::OpenGlMatrix &M)
    // {
    //     // eigen to cv
    //     Eigen::Matrix4d matEigen = matInSe3.to_homogeneous_matrix();
    //     cv::Mat matIn;
    //     eigen2cv(matEigen, matIn);

    //     if(!matIn.empty())
    //     {
    //         cv::Mat Rwc(3,3,CV_64F);
    //         cv::Mat twc(3,1,CV_64F);
    //         {
    //             unique_lock<mutex> lock(mMutexCamera);
    //             Rwc = matIn.rowRange(0,3).colRange(0,3).t();
    //             twc = -Rwc*matIn.rowRange(0,3).col(3);
    //         }

    //         M.m[0] = Rwc.at<double>(0,0);
    //         M.m[1] = Rwc.at<double>(1,0);
    //         M.m[2] = Rwc.at<double>(2,0);
    //         M.m[3]  = 0.0;

    //         M.m[4] = Rwc.at<double>(0,1);
    //         M.m[5] = Rwc.at<double>(1,1);
    //         M.m[6] = Rwc.at<double>(2,1);
    //         M.m[7]  = 0.0;

    //         M.m[8] = Rwc.at<double>(0,2);
    //         M.m[9] = Rwc.at<double>(1,2);
    //         M.m[10] = Rwc.at<double>(2,2);
    //         M.m[11]  = 0.0;

    //         M.m[12] = twc.at<double>(0);
    //         M.m[13] = twc.at<double>(1);
    //         M.m[14] = twc.at<double>(2);
    //         M.m[15]  = 1.0;
    //     }
    //     else
    //         M.SetIdentity();
    // }

    // draw ellipsoids
    // bool MapDrawer::drawEllipsoids() {
    //     // std::vector<ellipsoid*> ellipsoids = mpMap->GetAllEllipsoids();

    //     // std::vector<ellipsoid*> ellipsoidsVisual = mpMap->GetAllEllipsoidsVisual();
    //     // ellipsoids.insert(ellipsoids.end(), ellipsoidsVisual.begin(), ellipsoidsVisual.end());


    //     for( size_t i=0; i<ellipsoids.size(); i++)
    //     {
    //         SE3Quat TmwSE3 = ellipsoids[i]->pose.inverse();
    //         Vector3d scale = ellipsoids[i]->scale;

    //         glPushMatrix();

    //         glLineWidth(mCameraLineWidth*3/4.0);

    //         if(ellipsoids[i]->isColorSet()){
    //             Vector4d color = ellipsoids[i]->getColorWithAlpha();
    //             glColor4f(color(0),color(1),color(2),color(3));
    //         }
    //         else
    //             glColor3f(0.0f,0.0f,1.0f);

    //         GLUquadricObj *pObj;
    //         pObj = gluNewQuadric();
    //         gluQuadricDrawStyle(pObj, GLU_LINE);

    //         pangolin::OpenGlMatrix Twm;   // model to world
    //         SE3ToOpenGLCameraMatrix(TmwSE3, Twm);
    //         glMultMatrixd(Twm.m);  
    //         glScaled(scale[0],scale[1],scale[2]);

    //         gluSphere(pObj, 1.0, 26, 13); // draw a sphere with radius 1.0, center (0,0,0), slices 26, and stacks 13.
    //         drawAxisNormal();

    //         glPopMatrix();
    //     }

    //     return true;
    // }

    void MapDrawer::drawEllipsoid() {
        // SE3Quat TmwSE3 = ellipsoids[i]->pose.inverse();
        // Vector3d scale = ellipsoids[i]->scale;

        glPushMatrix();

        glLineWidth(mCameraLineWidth*3/4.0);

        // if(ellipsoids[i]->isColorSet()){
        //     Vector4d color = ellipsoids[i]->getColorWithAlpha();
        //     glColor4f(color(0),color(1),color(2),color(3));
        // }
        // else
        //     glColor3f(0.0f,0.0f,1.0f);

        GLUquadricObj *pObj;
        pObj = gluNewQuadric();
        gluQuadricDrawStyle(pObj, GLU_LINE);

        pangolin::OpenGlMatrix Twm;   // model to world
        // SE3ToOpenGLCameraMatrix(TmwSE3, Twm);

        Eigen::Vector3d scale(0.5,0.5,0.5);
        Twm.SetIdentity();

        glMultMatrixd(Twm.m);  
        glScaled(scale[0],scale[1],scale[2]);

        gluSphere(pObj, 1.0, 26, 13); // draw a sphere with radius 1.0, center (0,0,0), slices 26, and stacks 13.
        drawAxisNormal();

        glPopMatrix();

        // return true;
    }

} //namespace ORB_SLAM
