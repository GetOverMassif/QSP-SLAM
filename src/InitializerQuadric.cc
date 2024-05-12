#include "InitializerQuadric.h"
#include "core/ConstrainPlane.h"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

namespace ORB_SLAM2
{
    InitializerQuadric::InitializerQuadric(int rows, int cols) {
        miImageRows = rows;
        miImageCols = cols;
    }

    // ascending sort
    bool cmp(pair<int, double>a, pair<int, double>b)
    {
        return a.second > b.second;
    }

    // @input: 一行一个plane
    std::vector<ConstrainPlane*> MatToCPlanes(const MatrixXd& mat)
    {
        int num = mat.rows();
        std::vector<ConstrainPlane*> vpCplanes;
        for(int i=0;i<num;i++)
        {
            g2o::plane* ppl = new g2o::plane(mat.row(i));
            ConstrainPlane* pCPlane = new ConstrainPlane(ppl);
            vpCplanes.push_back(pCPlane);
        }
        return vpCplanes;
    }

    // Initialize an ellipsoid from several bounding boxes and camera poses.
    g2o::ellipsoid InitializerQuadric::initializeQuadric(MatrixXd &pose_mat, MatrixXd &detection_mat, Matrix3d &calib) {
        mbResult = false;

        int input_size = pose_mat.rows();

        // 1) Generate tangent planes from bounding boxes and camera cneter. 
        //    Those invalid observations are ignored.
        MatrixXd planesHomo = getPlanesHomo(pose_mat, detection_mat, calib); 

        int plane_size = planesHomo.cols();
        int invalid_plane = input_size*4 - plane_size;
        std::cout << " * inputplane / validplane: " << input_size*4 << " / " << plane_size << std::endl;
        if( invalid_plane > 0)
            std::cout << " * invalid_plane: " << invalid_plane << std::endl;

        if(plane_size < 9)  // at least 9 planes are needed
        {
            g2o::ellipsoid e_bad;
            mbResult = false;
            return e_bad;
        }

        // Using SVD to generate a quadric
        MatrixXd planesVector = getVectorFromPlanesHomo(planesHomo);
        Matrix4d QStar = getQStarFromVectors(planesVector);

        // generate ellipsoid from the quadric
        g2o::ellipsoid e = getEllipsoidFromQStar(QStar);

        // planesHomo 按col存储了世界坐标系下的切平面
        e.mvCPlanesWorld = MatToCPlanes(planesHomo.transpose());

        // Set color : blue.
        Eigen::Vector3d blueScalar(0,0,255);
        e.setColor(blueScalar);
        return e;
    }

    MatrixXd InitializerQuadric::getPlanesHomo(MatrixXd &pose_mat, MatrixXd &detection_mat, Matrix3d &calib) {
        assert( pose_mat.rows() == detection_mat.rows() && " Two matrices should match. " );
        assert( pose_mat.rows() > 2 && " At least 3 measurements are required. " );

        MatrixXd planes_all(4,0);

        int rows = pose_mat.rows();
        for(int i=0;i<rows;i++)
        {
            VectorXd pose = pose_mat.row(i);
            VectorXd detection = detection_mat.row(i);

            // filter invalid detections
            if( detection(0) < 1 && detection(1) < 1 && detection(2) < 1 && detection(3) < 1  )
                continue;

            Vector7d pose_vec = pose.tail(7);
            g2o::SE3Quat campose_wc(pose_vec);
            // get projection matrix
            MatrixXd P = generateProjectionMatrix(campose_wc.inverse(), calib);

            MatrixXd lines = fromDetectionsToLines(detection);
            MatrixXd planes = P.transpose() * lines;

            // add to matrix
            for( int m=0;m<planes.cols();m++)
            {
                planes_all.conservativeResize(planes_all.rows(), planes_all.cols()+1);
                planes_all.col(planes_all.cols()-1) = planes.col(m);
            }
        }

        return planes_all;
    }

    Matrix3Xd InitializerQuadric::generateProjectionMatrix(const SE3Quat& campose_cw, const Matrix3d& Kalib) const
    {
        Matrix3Xd identity_lefttop;
        identity_lefttop.resize(3, 4);
        identity_lefttop.col(3)=Vector3d(0,0,0);
        identity_lefttop.topLeftCorner<3,3>() = Matrix3d::Identity(3,3);

        Matrix3Xd proj_mat = Kalib * identity_lefttop;

        proj_mat = proj_mat * campose_cw.to_homogeneous_matrix();

        return proj_mat;
    }

    MatrixXd InitializerQuadric::fromDetectionsToLines(VectorXd &detections) {
        bool flag_openFilter = true;        // filter those lines lying on the image boundary

        double x1 = detections(0);
        double y1 = detections(1);
        double x2 = detections(2);
        double y2 = detections(3);

        Vector3d line1 (1, 0, -x1);
        Vector3d line2 (0, 1, -y1);
        Vector3d line3 (1, 0, -x2);
        Vector3d line4 (0, 1, -y2);

        // those lying on the image boundary have been marked -1 
        MatrixXd line_selected(3, 0);
        MatrixXd line_selected_none(3, 0);
        if( !flag_openFilter || ( x1>0 && x1<miImageCols-1 ))
        {
            line_selected.conservativeResize(3, line_selected.cols()+1);
            line_selected.col(line_selected.cols()-1) = line1;
        }
        if( !flag_openFilter || (y1>0 && y1<miImageRows-1 ))
        {
            line_selected.conservativeResize(3, line_selected.cols()+1);
            line_selected.col(line_selected.cols()-1) = line2;
        }
        if( !flag_openFilter || (x2>0 && x2<miImageCols-1 ))
        {
            line_selected.conservativeResize(3, line_selected.cols()+1);
            line_selected.col(line_selected.cols()-1) = line3;
        }
        if( !flag_openFilter || (y2>0 && y2<miImageRows-1 ))
        {
            line_selected.conservativeResize(3, line_selected.cols()+1);
            line_selected.col(line_selected.cols()-1) = line4;
        }

        return line_selected;
    }

    MatrixXd InitializerQuadric::getVectorFromPlanesHomo(MatrixXd &planes) {
        int cols = planes.cols();

        MatrixXd planes_vector(10,0);

        for(int i=0;i<cols;i++)
        {
            VectorXd p = planes.col(i);
            Vector10d v;

            v << p(0)*p(0),2*p(0)*p(1),2*p(0)*p(2),2*p(0)*p(3),p(1)*p(1),2*p(1)*p(2),2*p(1)*p(3),p(2)*p(2),2*p(2)*p(3),p(3)*p(3);

            planes_vector.conservativeResize(planes_vector.rows(), planes_vector.cols()+1);
            planes_vector.col(planes_vector.cols()-1) = v;
        }

        return planes_vector;
    }

    Matrix4d InitializerQuadric::getQStarFromVectors(MatrixXd &planeVecs) {
        MatrixXd A = planeVecs.transpose();

        // svd decompose
        JacobiSVD<Eigen::MatrixXd> svd(A, ComputeThinU | ComputeThinV );
        MatrixXd V = svd.matrixV();

        VectorXd qj_hat = V.col(V.cols()-1);

        // Get QStar
        Matrix4d QStar;
        QStar <<
            qj_hat(0),qj_hat(1),qj_hat(2),qj_hat(3),
            qj_hat(1),qj_hat(4),qj_hat(5),qj_hat(6),
            qj_hat(2),qj_hat(5),qj_hat(7),qj_hat(8),
            qj_hat(3),qj_hat(6),qj_hat(8),qj_hat(9);
        
        return QStar;
    }

    // Refer to QuadricSLAM-OpenSource
    g2o::ellipsoid InitializerQuadric::getEllipsoidFromQStar(Matrix4d& QStar)
    {
        // normalize if required
        Matrix4d normalized_dual_quadric(QStar);
        if (QStar(3,3) != 1.0) {
            normalized_dual_quadric = QStar/QStar(3,3);
        }

        // extract translation
        Vector3d translation(normalized_dual_quadric.block(0,3,3,1));

        // calculate the point quadric matrix
        Matrix4d point_quadric = normalized_dual_quadric.inverse();
        Matrix4d normalized_point_quadric = point_quadric;
        if (point_quadric(3,3) != 1.0) {
            normalized_point_quadric = point_quadric/point_quadric(3,3);
        }

        // extract shape
        auto lambdaa = normalized_point_quadric.block(0,0,3,3).eigenvalues();
        Vector3d shape = Eigen::sqrt(
            -1.0*normalized_point_quadric.determinant() \
            / normalized_point_quadric.block(0,0,3,3).determinant() \
            *  1.0/lambdaa.array()  ).abs();

        // extract rotation 
        Eigen::EigenSolver<Eigen::Matrix<double,3,3>> s(normalized_point_quadric.block(0,0,3,3));
        Matrix3d rotation_matrix = s.eigenvectors().real();

        // ensure rotation is right-handed
        if (!(fabs(1.0-rotation_matrix.determinant()) < 1e-8)) {
            rotation_matrix *= -1.0 * Matrix3d::Identity();
        }
        
        Quaterniond quat(rotation_matrix);
                
        Vector10d objectVec;
        objectVec << translation,quat.x(),quat.y(),quat.z(),quat.w(),shape;
        g2o::ellipsoid e; e.fromVector(objectVec);
        mbResult = true;
        return e;

    }

    // g2o::ellipsoid InitializerQuadric::getEllipsoidFromQStar(Matrix4d &QStar) {
    //     g2o::ellipsoid e;

    //     Matrix4d Q = QStar.inverse() * cbrt(QStar.determinant());

    //     SelfAdjointEigenSolver<Matrix4d> es(Q);    // ascending order by default
    //     MatrixXd D = es.eigenvalues().asDiagonal();
    //     MatrixXd V = es.eigenvectors();

    //     VectorXd eigens = es.eigenvalues();

    //     if( eigens(3) > 0  )  // normalize to - - - + 
    //     {
    //         Q=-Q;
    //         SelfAdjointEigenSolver<Matrix4d> es_2(Q);  
    //         D = es_2.eigenvalues().asDiagonal();
    //         V = es_2.eigenvectors();

    //         eigens = es_2.eigenvalues();
    //     }

    //     // Solve ellipsoid parameters from matrix Q
    //     // 求解 scale

    //     // 注意! 此处希望得到的 eigen_value 是 3x3的value, 而此处是对 4x4 求eigen之后取了前三个??? 注意经过降排序后不一定是3x3对应了!
    //     Matrix3d Q33 = Q.block(0,0,3,3);
    //     Vector3d lambda_mat = Q33.eigenvalues().array().inverse();
    //     double k = Q.determinant()/Q33.determinant();
    //     Vector3d value = -k*(lambda_mat);
    //     Vector3d s = Eigen::sqrt(value.array()).abs();

    //     // ****** Reference  : from quadricslam-opensource *****
    //     //   Vector3 shape = Eigen::sqrt(
    //     // -1.0*normalized_point_quadric.determinant() \
    //     // / normalized_point_quadric.block(0,0,3,3).determinant() \
    //     // *  1.0/lambdaa.array()  ).abs();

    //     Vector4d t = QStar.col(3);
    //     t = t/t(3);
    //     Vector3d translation = t.head(3);

    //     SelfAdjointEigenSolver<Matrix3d> es2(Q33);   
    //     MatrixXd D_Q33 = es2.eigenvalues().asDiagonal();
    //     MatrixXd rot = es2.eigenvectors();

    //     double r,p,y;
    //     rot_to_euler_zyx<double>(rot, r, p, y);
    //     Vector3d rpy(r,p,y);

    //     // generate ellipsoid
    //     Vector9d objectVec;
    //     objectVec << t(0),t(1),t(2),rpy(0),rpy(1),rpy(2),s(0),s(1),s(2);
    //     e.fromMinimalVector(objectVec);
        
    //     mbResult = true;
    //     return e;

    // }

    void InitializerQuadric::sortEigenValues(VectorXcd &eigens, MatrixXcd &V) {
        vector<pair<int, double>> pairs;
        for(int i=0;i<4;i++)
            pairs.push_back( make_pair( i, eigens(i).real() ) );
        sort(pairs.begin(), pairs.end(), cmp);

        // Construct a new matrix in order
        MatrixXcd V_new(4,4);
        Vector4cd eigens_new;

        for(int i=0;i<4;i++)
        {
            int oldID = pairs[i].first;
            V_new.col(i) = V.col(oldID);
            eigens_new(i) = complex<double>(pairs[i].second, 0);
        }

        eigens = eigens_new;
        V = V_new;
    }

    double InitializerQuadric::quadricErrorWithPlanes(MatrixXd &pose_mat, MatrixXd &detection_mat, Matrix3d &calib,
                                               g2o::ellipsoid &e) {
        Matrix4d QStar = e.generateQuadric();           // get Q^*
        Vector10d qj_hat;
        qj_hat << QStar(0,0),QStar(0,1),QStar(0,2),QStar(0,3),QStar(1,1),QStar(1,2),QStar(1,3),QStar(2,2),QStar(2,3),QStar(3,3);

        // Get planes vector
        MatrixXd planesHomo = getPlanesHomo(pose_mat, detection_mat, calib);
        MatrixXd planesVector = getVectorFromPlanesHomo(planesHomo);

        // Get error
        VectorXd result = planesVector.transpose() * qj_hat;
        return result.transpose() * result;
    }

    bool InitializerQuadric::getInitializeResult() {
        return mbResult;
    }

    g2o::ellipsoid InitializerQuadric::initializeQuadric(EllipsoidSLAM::Observations &obs, Matrix3d &calib) {
        MatrixXd pose_mat;
        MatrixXd detection_mat;

        // get pose matrix and detection matrix from observations
        getDetectionAndPoseMatFromObservations(obs, pose_mat, detection_mat);
        std::cout << "pose_mat : \n " << pose_mat << std::endl;
        std::cout << "detection_mat : \n " << detection_mat << std::endl;

        g2o::ellipsoid e = initializeQuadric(pose_mat, detection_mat, calib);

        if( getInitializeResult() )
        {
            e.miLabel = obs[0]->label;
        }

        return e;
    }

    void InitializerQuadric::getDetectionAndPoseMatFromObservations(EllipsoidSLAM::Observations &obs, MatrixXd &pose_mat,
                                                             MatrixXd &detection_mat) {
        int frameSize = obs.size();
        pose_mat.resize(frameSize, 7);  // x y z qx qy qz qw
        detection_mat.resize(frameSize, 5); // x1 y1 x2 y2 accuracy

        int id = 0;
        for (auto iter = obs.begin(); iter!=obs.end(); iter++)
        {
            Vector5d det_vec;
            det_vec << (*iter)->bbox, (*iter)->rate;
            Vector7d pos_vec = (*iter)->pFrame->cam_pose_Twc.toVector();

            pose_mat.row(id) = pos_vec;
            detection_mat.row(id) = det_vec;

            id ++;
        }
    }
}
