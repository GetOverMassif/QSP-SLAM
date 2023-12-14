#ifndef ELLIPSOIDSLAM_OPTIMIZER_H
#define ELLIPSOIDSLAM_OPTIMIZER_H

#include "Frame.h"
#include "Map.h"
#include "Initializer.h"

#include "src/symmetry/Symmetry.h"
#include "src/Relationship/Relationship.h"
#include "SupportingPlane.h"
#include "BasicEllipsoidEdges.h"

namespace ORB_SLAM2 {
    class Object
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        inline double dp_prior(int label)
        {
            // 需要知道总共的 label 种类. 然后有新观测则将其翻倍.
            int total_ob = measurementIDs.size();

            int label_ob;
            if(classVoter.find(label)!=classVoter.end())
                label_ob = classVoter[label];
            else 
                label_ob = 0;
            
            int label_ob_new = label_ob + 1;
            
            double dp = double(label_ob_new) / (total_ob+1);
            return dp;
        }

        int instance_id;
        std::map<int,int> classVoter;
        std::vector<int> measurementIDs;
        ellipsoid* pEllipsoid;
    };
    typedef std::vector<Object> Objects;

    class Relation;    // TOBECHECK: typedef出来的能否用class做声明?
    typedef std::vector<Relation> Relations;

    class Optimizer {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Optimizer();

        // Optimize with probabilistic data association
        void GlobalObjectGraphOptimizationWithPDA(std::vector<Frame *> &pFrames, Map *pMap, const Matrix3d& calib, int iRows, int iCols);

        // 使用点作为物体模型版本
        void GlobalObjectGraphOptimizationWithPDAPointModel(std::vector<Frame *> &pFrames, Map *pMap);

        void SetGroundPlane(Vector4d& normal);
        void GetOptimizedResult(Objects& objs, Measurements& mms);

        // 使用 QuadricSLAM 模型的入口. ( 该函数在 lab.cpp 被调用)
        std::vector<g2o::ellipsoid*> OptimizeUsingQuadricSLAM(std::vector<Frame *> &pFrames, Measurements& mms, Objects& objs, Trajectory& camTraj, Matrix3d& mCalib, int rows, int cols);
    private:
        void UpdateDataAssociation(Measurements& mms, Objects& objs, int model = 0);
        void OptimizeWithDataAssociation(std::vector<Frame *> &pFrames, int rows, int cols, Matrix3d &mCalib, 
                Measurements& mms, Objects& objs);
        void OptimizeWithDataAssociationUsingMultiplanes(std::vector<Frame *> &pFrames, 
                Measurements& mms, Objects& objs, Trajectory& camTraj, const Matrix3d& calib, int iRows, int iCols);
        void OptimizeWithDataAssociationUsingMultiplanesWithRelations(std::vector<Frame *> &pFrames, int rows, int cols, Matrix3d &mCalib, 
                Measurements& mms, Objects& objs, Trajectory& camTraj, Relations& rls);
        void OptimizeWithDataAssociationUsingMultiplanesPointModel(std::vector<Frame *> &pFrames, Measurements& mms, Objects& objs, Trajectory& camTraj);

        void LoadRelations(Relations& rls, SupportingPlanes& spls);
    private:
        std::map<int, std::vector<float>> mMapObjectConstrain;

        bool mbGroundPlaneSet;
        Vector4d mGroundPlaneNormal;

        bool mbRelationLoaded;
        Relations mRelations;
        SupportingPlanes mSupportingPlanes;

        // 保存优化结果
        Objects mObjects; 
        Measurements mMeasurements;
    };

    int GetTotalObjectIndex(std::vector<Frame *> &pFrames, int frame_index, int index_in_frame);

    bool checkVisibility(g2o::EdgeSE3EllipsoidProj *edge, g2o::VertexSE3Expmap *vSE3, 
    g2o::VertexEllipsoid *vEllipsoid, Eigen::Matrix3d &mCalib, int rows, int cols);
}

#endif //ELLIPSOIDSLAM_OPTIMIZER_H
