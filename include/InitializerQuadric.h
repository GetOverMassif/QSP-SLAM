#ifndef ELLIPSOIDSLAM_INITIALIZER_H
#define ELLIPSOIDSLAM_INITIALIZER_H

#include "Ellipsoid.h"
#include "Frame.h"

#include <Eigen/Core>

using namespace Eigen;
using namespace g2o;

namespace ORB_SLAM2
{
    class InitializerQuadric
    {

    public:
        InitializerQuadric(int rows, int cols);    // image rows and cols

        /*
         * pose_mat:  x y z qx qy qz qw
         * detection_mat: x1 y1 x2 y2 (accuracy)
         */
        g2o::ellipsoid initializeQuadric(MatrixXd &pose_mat, MatrixXd &detection_mat, Matrix3d &calib);
        g2o::ellipsoid initializeQuadric(Observations &obs, Matrix3d &calib);

        // calculate the error between a given ellipsoid and planes
        double quadricErrorWithPlanes(MatrixXd &pose_mat, MatrixXd &detection_mat, Matrix3d &calib, g2o::ellipsoid &e);

        // solve pose and scale from a matrix Q^* and construct an Ellipsoid class
        g2o::ellipsoid getEllipsoidFromQStar(Matrix4d &QStar);

        // get the initialization result.
        bool getInitializeResult();
		bool flag_openFilter = true;        // filter those lines lying on the image boundary
    private:

        // get the homogeneous expressions of planes
        MatrixXd getPlanesHomo(MatrixXd &pose_mat, MatrixXd &detection_mat, Matrix3d &calib);
        // get the vector form of plane constraints to prepare a linear equation system
        MatrixXd getVectorFromPlanesHomo(MatrixXd &planes);
        // solve a least square solution from SVD
        Matrix4d getQStarFromVectors(MatrixXd &planeVecs);

        Matrix3Xd generateProjectionMatrix(const SE3Quat& campose_cw, const Matrix3d& Kalib) const;
        MatrixXd fromDetectionsToLines(VectorXd &detection_mat);

        void getDetectionAndPoseMatFromObservations(ORB_SLAM2::Observations &obs,
                                                    MatrixXd &pose_mat, MatrixXd &detection_mat);

        // sort eigen values in ascending order.
        void sortEigenValues(VectorXcd &eigens, MatrixXcd &V);

        bool mbResult;

        int miImageRows, miImageCols;


    };
}

#endif //ELLIPSOIDSLAM_INITIALIZER_H
