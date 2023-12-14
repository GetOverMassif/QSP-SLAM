#include "include/core/Optimizer.h"
#include "include/core/Ellipsoid.h"
#include "include/core/BasicEllipsoidEdges.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include <src/config/Config.h>

#include "include/core/NonparamDA.h"

#include "src/pca/EllipsoidExtractorEdges.h"
#include "include/core/ConstrainPlane.h"

#include "src/plane/PlaneVertexEdges.h"

using namespace cv;
using namespace g2o;

namespace ORB_SLAM2
{

bool isInImage(Eigen::Vector2d& uv, int rows, int cols){
    
    if( uv(0) >0 && uv(0) < cols )
        if( uv(1) >0 && uv(1) < rows )
            return true;

    return false;
}

// Update: 2020-6-10 , 1) and 2)
// [ Unused ]
// The function checks several conditions to decide whether the edge will be activated:
// 1) if the object is behind the camera
// 2) if the camera is inside the elipsoid, which generates an ill condition
// 3) if the projected ellipse lies in the image width and height
//    if one of the boundingbox vertces lies in the image, active it too.
bool checkVisibility(g2o::EdgeSE3EllipsoidProj *edge, g2o::VertexSE3Expmap *vSE3, 
    g2o::VertexEllipsoid *vEllipsoid, Eigen::Matrix3d &mCalib, int rows, int cols)
{
    g2o::ellipsoid e = vEllipsoid->estimate();
    Vector3d ellipsoid_center = e.toMinimalVector().head(3);    // Pwo
    Vector4d center_homo = real_to_homo_coord_vec<double>(ellipsoid_center);

    g2o::SE3Quat campose_Tcw = vSE3->estimate();
    Eigen::Matrix4d projMat = campose_Tcw.to_homogeneous_matrix(); // Tcw
    
    // project to image plane.
    Vector4d center_inCameraAxis_homo = projMat * center_homo;   // Pco =  Tcw * Pwo
    Vector3d center_inCameraAxis = homo_to_real_coord_vec<double>(center_inCameraAxis_homo);

    if( center_inCameraAxis_homo(2) < 0)    // if the object is behind the camera. z< 0
    {
        return false;
    }

    // check if the camera is inside the elipsoid, which generates an ill condition
    g2o::SE3Quat campose_Twc = campose_Tcw.inverse();
    Eigen::Vector3d X_cam = campose_Twc.translation();
    Eigen::Vector4d X_homo = real_to_homo_coord_vec<double>(X_cam);
    Eigen::Matrix4d Q_star = e.generateQuadric();
    Eigen::Matrix4d Q = Q_star.inverse();
    double point_in_Q = X_homo.transpose() * Q * X_homo;
    if(point_in_Q < 0)  // the center of the camera is inside the ellipsoid
        return false;

    // // check if the projected ellipse lies in the image width and height
    // Eigen::Matrix3Xd P = e.generateProjectionMatrix(campose_Tcw, mCalib);
    // Eigen::Vector3d uv = P * center_homo;
    // uv = uv/uv(2);
    // Eigen::Vector2d uv_2d(uv(0), uv(1));
    // if( isInImage(uv_2d, rows, cols) )
    //         return true;

    // // if one of the boundingbox vertces lies in the image, active it too.
    // Vector4d vec_proj = edge->getProject();
    // Vector2d point_lu(vec_proj(0), vec_proj(1));
    // Vector2d point_rd(vec_proj(2), vec_proj(3));
    // if( isInImage(point_lu,rows,cols) || isInImage(point_rd, rows, cols) )
    //     return true;

    // return false;

    return true;    // 修改，若没有满足上述两个条件，则说明是可以的

}

/*
*   [关系加载]
*   将 Relations / SupportingPlanes 加载到图中并返回边的指针数组.
*/
void LoadRelationsToGraph(Relations& relations, SupportingPlanes& supportingPlanes, 
            g2o::SparseOptimizer& graph, std::vector<g2o::VertexSE3Expmap*>& vSE3Vertex, std::vector<g2o::EdgePlaneSE3*>& vEdgePlaneSE3_)
{
    double config_plane_weight = Config::ReadValue<double>("DEBUG.PLANE.WEIGHT");
    cout << " * DEBUG-Plane weight : " << config_plane_weight << endl;

    // 仿造 mms, objs 的加载方式.
    int spl_num = supportingPlanes.size();
    std::map<int, g2o::VertexPlane3DOF *> mapPlaneVertices;
    std::vector<g2o::EdgePlaneSE3*> vEdgePlaneSE3;
    for(int sp_id=0; sp_id<spl_num; sp_id++ )        
    {
        SupportingPlane &spl = supportingPlanes[sp_id];
        int instance = spl.instance_id;
        g2o::plane* pCPlane = spl.pPlane;
        // Add planes vertices
        g2o::VertexPlane3DOF *vPlane = new g2o::VertexPlane3DOF();
        vPlane->setEstimate(*pCPlane);
        vPlane->setId(graph.vertices().size());
        vPlane->setFixed(false);
        graph.addVertex(vPlane);
        mapPlaneVertices.insert(make_pair(instance, vPlane)); 

        // 开始添加平面的观测
        std::set<int>& relation_ids = spl.vRelation;
        for( auto iter=relation_ids.begin(); iter!=relation_ids.end(); iter++ )
        {
            int rl_id = *iter;
            Relation& rl = relations[rl_id];
            g2o::plane* plane_local = rl.pPlane;

            // 寻找 frame vertex
            int frame_index = rl.pFrame->frame_seq_id;
            auto vSE3 = vSE3Vertex[frame_index];

            if( plane_local == NULL ) continue;

            g2o::EdgePlaneSE3* pEdge = new g2o::EdgePlaneSE3;
            pEdge->setId(graph.edges().size());
            pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vPlane ));
            pEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
            pEdge->setMeasurement(*plane_local);

            Vector3d inv_sigma;
            inv_sigma << 1,1,1;
            inv_sigma = inv_sigma * config_plane_weight;
            MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
            pEdge->setInformation(info);

            graph.addEdge(pEdge);
            vEdgePlaneSE3.push_back(pEdge);
               
        } // plane的观测: relations 遍历
    } // supporting planes 遍历

    // 保存输出
    vEdgePlaneSE3_ = vEdgePlaneSE3;
    return;
}

/*
*   该函数专门用于 Nonparam Data association 版本的优化计算
*/
void Optimizer::OptimizeWithDataAssociation(std::vector<Frame *> &pFrames, int rows, int cols, Matrix3d &mCalib, 
                Measurements& mms, Objects& objs) {
    // ************************ LOAD CONFIGURATION ************************
    double config_ellipsoid_3d_scale = Config::Get<double>("Optimizer.Edges.3DEllipsoid.Scale");
    bool mbSetGravityPrior = Config::Get<int>("Optimizer.Edges.GravityPrior.Open") == 1;  
    double dGravityPriorScale = Config::Get<double>("Optimizer.Edges.GravityPrior.Scale");
    bool mbOpen3DProb = true;  

    // OUTPUT
    std::cout << " -- Optimization parameters : " << std::endl;
    if(mbGroundPlaneSet)
        std::cout << " [ Using Ground Plane: " << mGroundPlaneNormal.transpose() << " ] " << std::endl;

    if(!mbGroundPlaneSet || !mbSetGravityPrior )   
        std::cout << " * Gravity Prior : closed." << std::endl;
    else
        std::cout << " * Gravity Prior : Open." << std::endl;
    
    cout<<" * Scale_3dedge: " << config_ellipsoid_3d_scale << endl;
    cout<<" * Scale_GravityPrior: " << dGravityPriorScale << endl;
    // ************************************************************************

    // Initialize variables.
    int total_frame_number = int(pFrames.size());
    int objects_num = int(objs.size());

    // initialize graph optimization.
    g2o::SparseOptimizer graph;
    g2o::BlockSolverX::LinearSolverType* linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    graph.setAlgorithm(solver);
    graph.setVerbose(false);        // Set output.

    std::map<int, g2o::VertexEllipsoid*> vEllipsoidVertexMaps;
    std::vector<g2o::EdgeSE3EllipsoidProj*> edges, edgesValid, edgesInValid;
    std::vector<bool> validVec; validVec.resize(total_frame_number);
    std::vector<g2o::EdgeEllipsoidGravityPlanePrior *> edgesEllipsoidGravityPlanePrior;     // Gravity prior
    std::vector<g2o::VertexSE3Expmap*> vSE3Vertex;

    std::vector<g2o::EdgeSE3Ellipsoid9DOF*> vEllipsoid3DEdges;

    // Add SE3 vertices for camera poses
    bool bSLAM_mode = false;   // Mapping Mode : Fix camera poses and mapping ellipsoids only
    for( int frame_index=0; frame_index< total_frame_number ; frame_index++) {
        g2o::SE3Quat curr_cam_pose_Twc = pFrames[frame_index]->cam_pose_Twc;
        Eigen::MatrixXd det_mat = pFrames[frame_index]->mmObservations;

        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setId(graph.vertices().size());
        graph.addVertex(vSE3);
        vSE3->setEstimate(pFrames[frame_index]->cam_pose_Tcw); // Tcw
        if(!bSLAM_mode)
            vSE3->setFixed(true);       // Fix all the poses in mapping mode.
        else 
            vSE3->setFixed(frame_index == 0);   
        vSE3Vertex.push_back(vSE3);

        // Add odom edges if in SLAM Mode
        if(bSLAM_mode && frame_index > 0){
            g2o::SE3Quat prev_cam_pose_Tcw = pFrames[frame_index-1]->cam_pose_Twc.inverse();
            g2o::SE3Quat curr_cam_pose_Tcw = curr_cam_pose_Twc.inverse();
            g2o::SE3Quat odom_val = curr_cam_pose_Tcw*prev_cam_pose_Tcw.inverse();;

            g2o::EdgeSE3Expmap* e = new g2o::EdgeSE3Expmap();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>( vSE3Vertex[frame_index-1] ));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>( vSE3Vertex[frame_index] ));
            e->setMeasurement(odom_val);

            e->setId(graph.edges().size());
            Vector6d inv_sigma;inv_sigma<<1,1,1,1,1,1;
            inv_sigma = inv_sigma*1.0;
            Matrix6d info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
            e->setInformation(info);
            graph.addEdge(e);
        }
    }

    // 接下来： 以 mms, objs 构造图.
    // Initialize objects vertices and add edges of camera-objects 2d observations 
    int objectid_in_edge = 0;
    int current_ob_id = 0;  
    int symplaneid_in_edge = 0;
    for(int object_id=0; object_id<objects_num; object_id++ )        
    {
        Object &obj = objs[object_id];
        int instance = obj.instance_id;

        // Add objects vertices
        g2o::VertexEllipsoid *vEllipsoid = new g2o::VertexEllipsoid();
        vEllipsoid->setEstimate(*obj.pEllipsoid);
        vEllipsoid->setId(graph.vertices().size());
        vEllipsoid->setFixed(false);
        graph.addVertex(vEllipsoid);
        vEllipsoidVertexMaps.insert(make_pair(instance, vEllipsoid)); 

        // Add gravity prior
        if(mbGroundPlaneSet && mbSetGravityPrior ){
            g2o::EdgeEllipsoidGravityPlanePrior *vGravityPriorEdge = new g2o::EdgeEllipsoidGravityPlanePrior;
            vGravityPriorEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));

            vGravityPriorEdge->setMeasurement(mGroundPlaneNormal);  
            Matrix<double,1,1> inv_sigma;
            inv_sigma << 1 * dGravityPriorScale;
            MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
            vGravityPriorEdge->setInformation(info);
            
            graph.addEdge(vGravityPriorEdge);
            edgesEllipsoidGravityPlanePrior.push_back(vGravityPriorEdge);
            
        }

        // Add camera-objects 2d constraints 
        // At least 3 observations are needed to activiate 2d edges. Since one ellipsoid has 9 degrees,
        // and every observation offers 4 constrains, only at least 3 observations could fully constrain an ellipsoid.
        bool bVvalid_2d_constraints = (obj.measurementIDs.size() > 2);
        if( bVvalid_2d_constraints ){
            for( int i=0; i<obj.measurementIDs.size(); i++ )
            {
                Measurement& measurement = mms[obj.measurementIDs[i]];
                int label = measurement.ob_2d.label;
                Vector4d measure_bbox = measurement.ob_2d.bbox;
                double measure_prob = measurement.ob_2d.rate;

                // find the corresponding frame vertex.
                int frame_index = measurement.ob_2d.pFrame->frame_seq_id;
                g2o::VertexSE3Expmap *vSE3 = vSE3Vertex[frame_index]; 
                // create 2d edge
                g2o::EdgeSE3EllipsoidProj *e = new g2o::EdgeSE3EllipsoidProj();
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoidVertexMaps[instance] ));
                e->setMeasurement(measure_bbox); 
                e->setId(graph.edges().size());    
                Vector4d inv_sigma;
                inv_sigma << 1, 1, 1, 1;   
                Matrix4d info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
                info = info * measure_prob;
                e->setInformation(info);
                // e->setRobustKernel(new g2o::RobustKernelHuber());     // Huber Kernel
                e->setKalib(mCalib);
                e->setLevel(0);
                edges.push_back(e);

                // Two conditions for valid 2d edges:
                // 1) [closed] visibility check: see the comments of function checkVisibility for detail.
                // 2) NaN check
                bool check_visibility = false;
                bool c1 = (!check_visibility) || checkVisibility(e, vSE3, vEllipsoidVertexMaps[instance], mCalib, rows, cols);

                e->computeError();
                double e_error = e->chi2();
                bool c2 = !isnan(e_error);  // NaN check

                if( c1 && c2 ){
                    graph.addEdge(e);
                    edgesValid.push_back(e);  // valid edges and invalid edges are for debug output
                }
                else
                    edgesInValid.push_back(e);   
            }
        } // bVvalid_2d_constraints

        // 添加 3d 约束
        for( int i=0; i<obj.measurementIDs.size(); i++ )
        {
            Measurement& measurement = mms[obj.measurementIDs[i]];
            int instance = measurement.instance_id;
            
            Observation3D& ob_3d = measurement.ob_3d;
            g2o::ellipsoid* pObj_ob = ob_3d.pObj;
            
            if( pObj_ob == NULL ) continue;

            int frame_index = measurement.ob_3d.pFrame->frame_seq_id;
            auto vEllipsoid = vEllipsoidVertexMaps[instance];  
            auto vSE3 = vSE3Vertex[frame_index];

            // create 3d edges
            g2o::EdgeSE3Ellipsoid9DOF* vEllipsoid3D = new g2o::EdgeSE3Ellipsoid9DOF; 
            vEllipsoid3D->setId(graph.edges().size()); 
            vEllipsoid3D->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
            vEllipsoid3D->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));                
            vEllipsoid3D->setMeasurement(*pObj_ob); 

            Vector9d inv_sigma;
            inv_sigma << 1,1,1,1,1,1,1,1,1;
            if(mbOpen3DProb)
                inv_sigma = inv_sigma * sqrt(pObj_ob->prob);
            Matrix9d info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal() * config_ellipsoid_3d_scale;
            vEllipsoid3D->setInformation(info);

            graph.addEdge(vEllipsoid3D);
            vEllipsoid3DEdges.push_back(vEllipsoid3D);
        }
    } // objs 遍历

    // output 
    std::cout << " -- GRAPH INFORMATION : " << std::endl;
    cout << " * Object Num : " << objects_num << endl;
    cout<<" * Vertices: "<<graph.vertices().size()<<endl;
    cout<<" * 2d Edges [Valid/Invalid] : " << edges.size() << " [" << edgesValid.size() <<"/" << edgesInValid.size() << "]" << endl;
    std::cout << " * 3d Edges : " << vEllipsoid3DEdges.size() << std::endl;
    cout<<" * Gravity edges: " << edgesEllipsoidGravityPlanePrior.size() << endl;
    cout << endl;

    int num_optimize = Config::Get<double>("Optimizer.Optimize.Num");
    if(graph.edges().size()>0){
        graph.initializeOptimization();
        graph.optimize( num_optimize );  //optimization step

        // Update estimated ellispoids to the map
        // 更新结果到 objs
        for( int i = 0 ; i< objs.size() ; i++)
        {
            g2o::ellipsoid* pEllipsoid = objs[i].pEllipsoid;
            if(pEllipsoid!=NULL)
            {
                int instance = objs[i].instance_id;
                (*pEllipsoid) = vEllipsoidVertexMaps[instance]->estimate();
            }
        }
    }
    else 
    {
        std::cout << " No observations valid." << std::endl;
    }

    // Output optimization information.
    // object list
    ofstream out_obj("./object_list_nonparam.txt");
    auto iter = vEllipsoidVertexMaps.begin();
    for(;iter!=vEllipsoidVertexMaps.end();iter++)
    {
        out_obj << iter->first << "\t" << iter->second->estimate().toMinimalVector().transpose() 
            << "\t" << iter->second->estimate().miLabel << std::endl;
    }
    out_obj.close();
}

Optimizer::Optimizer()
{
    mbGroundPlaneSet = false;
    mbRelationLoaded = false;
}

void Optimizer::SetGroundPlane(Vector4d& normal){
    mbGroundPlaneSet = true;
    mGroundPlaneNormal = normal;
}

// 输出函数
void OutputOptimizationEdgeErrors(Objects& objs, std::map<int, std::vector<g2o::EdgeSE3EllipsoidPlane*>>& mapInsEdgePlanes, 
    std::map<int, std::vector<g2o::EdgeSE3EllipsoidPlanePartial*>>& mapInsEdgePlanesPartial)
{
    ofstream out_obj("./optimization_edges.txt");
    for( int i=0;i<objs.size();i++)
    {
        Object& obj = objs[i];
        int instance = obj.instance_id;

        std::vector<g2o::EdgeSE3EllipsoidPlane*>& vEdgesValid = mapInsEdgePlanes[instance];
        std::vector<g2o::EdgeSE3EllipsoidPlanePartial*>& vEdgesPartial = mapInsEdgePlanesPartial[instance];

        int num_valid = vEdgesValid.size();
        int num_partial = vEdgesPartial.size();
        int num_total = num_valid + num_partial;

        out_obj << "Object " << instance << ", PlaneEdges " << num_total << "( V " << num_valid << ", P " << num_partial << " )" << std::endl;
        out_obj << " -> Valid edges : " << std::endl;
        for( int n =0;n<num_valid; n++)
        {
            g2o::EdgeSE3EllipsoidPlane* e = vEdgesValid[n];
            double error = e->error()(0,0);
            out_obj << "   v " << n << " : " << error << std::endl;
        }

        out_obj << std::endl;
        out_obj << " -> Partial edges : " << std::endl;
        for( int n =0;n<num_partial; n++)
        {
            g2o::EdgeSE3EllipsoidPlanePartial* e = vEdgesPartial[n];
            double error = e->error()(0,0);
            out_obj << "   p " << n << " : " << error << std::endl;
        }
        out_obj << std::endl;
    }

    out_obj.close();
}

const char* GetLabelText_Optimizer(int id)
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
    if(id >= 0)
        return coco_classes[id];
    else 
        return "Unknown";
}

// 根据语义标签，若该物体是对称的，则赋予对称性质. 
// 目前该性质仅仅用于：
// 1) 判断旋转约束是否生效 
// 2) evo中是否评估其与gt的旋转error.
bool IsSymmetry(int label)
{
    static const std::set<std::string> symLabels = 
    {
        "vase", "bowl", "cup", "potted plant", "bottle"
    };
    const char* txtLabel = GetLabelText_Optimizer(label);
    if( symLabels.find(std::string(txtLabel)) != symLabels.end() )
        return true;
    else 
        return false;
}

void OutputInstanceObservationNum(std::map<int,int>& map)
{
    ofstream out_obj("./instance_observation_num.txt");

    for(auto pair : map)
    {
        out_obj << pair.first << " " << pair.second << std::endl;
    }
    out_obj.close();

    return ;
}

/*
*   [新版本] 基于切平面完成椭球体的全局优化!
*   10-4 Ours 论文发表使用的版本。
*/
void Optimizer::OptimizeWithDataAssociationUsingMultiplanes(std::vector<Frame *> &pFrames,
                Measurements& mms, Objects& objs, Trajectory& camTraj, const Matrix3d& calib, int iRows, int iCols) {
    // ************* 系统调试 ***************
    bool bUsePlanesAs3DConstrains = true;
    bool bUseNormalConstrain = true; // 是否使用三维切平面的法向量约束

    double config_plane_angle_sigma = Config::Get<double>("Optimizer.Edges.3DConstrain.PlaneAngle.Sigma");
    
    // 注意 ： 开启了3d 平面的局部过滤. 30 pixel
    // 注意 ： 开启了2d 平面的局部过滤. param设置 30pixel. 只有非局部平面才添加约束.
    // *************************************

    // ************************ LOAD CONFIGURATION ************************
    double config_ellipsoid_3d_scale = Config::ReadValue<double>("Optimizer.Edges.3DEllipsoid.Scale");
    double config_ellipsoid_2d_scale = Config::ReadValue<double>("Optimizer.Edges.2D.Scale");
    // if(config_ellipsoid_2d_scale <= 0.01)
    //     config_ellipsoid_2d_scale = 1.0;
    bool mbSetGravityPrior = Config::Get<int>("Optimizer.Edges.GravityPrior.Open") == 1;  
    double dGravityPriorScale = Config::Get<double>("Optimizer.Edges.GravityPrior.Scale");
    bool mbOpenPartialObservation = Config::Get<int>("Optimizer.PartialObservation.Open") == 1;
    bool mbOpen3DProb = true;  

    std::cout << " -- Image : " << iCols << " x " << iRows << std::endl;

    // OUTPUT
    std::cout << " -- Optimization parameters : " << std::endl;
    if(mbGroundPlaneSet)
        std::cout << " [ Using Ground Plane: " << mGroundPlaneNormal.transpose() << " ] " << std::endl;

    if(!mbGroundPlaneSet || !mbSetGravityPrior )   
        std::cout << " * Gravity Prior : closed." << std::endl;
    else
        std::cout << " * Gravity Prior : Open." << std::endl;
    
    cout<<" * Scale_3dedge: " << config_ellipsoid_3d_scale << endl;
    cout<<" * Scale_2dedge: " << config_ellipsoid_2d_scale << endl;
    cout<<" * Scale_GravityPrior: " << dGravityPriorScale << endl;
    cout<<" * config_plane_angle_sigma: " << config_plane_angle_sigma << endl;

    double config_odometry_weight = Config::ReadValue<double>("DEBUG.ODOM.WEIGHT");
    // double exp_config_odometry_weight = pow(10, config_odometry_weight);    //exp
    double exp_config_odometry_weight = config_odometry_weight; // normal
    cout << " * DEBUG-Odometry weight : " << exp_config_odometry_weight << endl;

    // 添加一个关闭 2D 约束的开关.
    bool mbClose2DConstrain = Config::Get<int>("Optimizer.Edges.2DConstrain.Close") > 0;
    if(mbClose2DConstrain)
        cout << "********** CLOSE 2D Constrain. **********" << endl;
    bool bOpen3DEllipsoidEdges = !(Config::Get<int>("Optimizer.Edges.3DConstrain.Close") > 0);
    if(!bOpen3DEllipsoidEdges)
        cout << "********** CLOSE 3D Constrain. **********" << endl;
    // ************************************************************************

    // Initialize variables.
    int total_frame_number = int(pFrames.size());
    int objects_num = int(objs.size());

    // initialize graph optimization.
    g2o::SparseOptimizer graph;
    g2o::BlockSolverX::LinearSolverType* linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    graph.setAlgorithm(solver);
    graph.setVerbose(false);        // Set output.

    std::map<int, g2o::VertexEllipsoidXYZABCYaw*> vEllipsoidVertexMaps;
    std::vector<g2o::EdgeSE3EllipsoidProj*> edges, edgesValid, edgesInValid;
    std::vector<bool> validVec; validVec.resize(total_frame_number);
    std::vector<g2o::EdgeEllipsoidGravityPlanePrior *> edgesEllipsoidGravityPlanePrior;     // Gravity prior
    std::vector<g2o::VertexSE3Expmap*> vSE3Vertex;

    std::vector<g2o::EdgeSE3Ellipsoid7DOF*> vEllipsoid3DEdges;
    std::vector<g2o::EdgeSE3Expmap*> vEdgesOdom;
    std::vector<g2o::EdgeSE3EllipsoidPlane*> vEdgeEllipsoidPlane;
    std::vector<g2o::EdgeSE3EllipsoidPlane*> vEdgeEllipsoidPlane3D;
    std::vector<g2o::EdgeSE3EllipsoidPlaneWithNormal*> vEdgeEllipsoidPlane3DWithNormal;
    std::vector<g2o::EdgeSE3EllipsoidPlanePartial*> vEdgeEllipsoidPlanePartial;
    std::vector<g2o::EdgeSE3EllipsoidCenterFull*> vEdgeSE3EllipsoidCenterFull;

    std::map<int, std::vector<g2o::EdgeSE3EllipsoidPlane*>> mapInsEdgePlanes;       // 8-27 check : 该部分统计并没有考虑新的带 normal 的法向量
    std::map<int, std::vector<g2o::EdgeSE3EllipsoidPlanePartial*>> mapInsEdgePlanesPartial; 

    // Add SE3 vertices for camera poses
    bool bSLAM_mode = (Config::Get<int>("Optimizer.SLAM.mode") == 1);   // Mapping Mode : Fix camera poses and mapping ellipsoids only
    std::cout << " [ SLAM Mode : " << bSLAM_mode << " ] " << std::endl;
    for( int frame_index=0; frame_index< total_frame_number ; frame_index++) {
        g2o::SE3Quat curr_cam_pose_Twc = pFrames[frame_index]->cam_pose_Twc;

        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setId(graph.vertices().size());
        graph.addVertex(vSE3);
        vSE3->setEstimate(pFrames[frame_index]->cam_pose_Tcw); // Tcw
        if(!bSLAM_mode)
            vSE3->setFixed(true);       // Fix all the poses in mapping mode.
        else 
            vSE3->setFixed(frame_index == 0);   
        vSE3Vertex.push_back(vSE3);

        // Add odom edges if in SLAM Mode
        if(bSLAM_mode && frame_index > 0){
            g2o::SE3Quat prev_cam_pose_Tcw = pFrames[frame_index-1]->cam_pose_Twc.inverse();
            g2o::SE3Quat curr_cam_pose_Tcw = curr_cam_pose_Twc.inverse();

            // 最后的 inverse: 将 wc 转为 cw
            // g2o::SE3Quat odom_val = (curr_cam_pose_Tcw*prev_cam_pose_Tcw.inverse()).inverse();; //  odom_wc * To = T1
            g2o::SE3Quat odom_val = curr_cam_pose_Tcw*prev_cam_pose_Tcw.inverse(); 

            g2o::EdgeSE3Expmap* e = new g2o::EdgeSE3Expmap();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>( vSE3Vertex[frame_index-1] ));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>( vSE3Vertex[frame_index] ));
            e->setMeasurement(odom_val);

            e->setId(graph.edges().size());
            Vector6d inv_sigma;inv_sigma<<1,1,1,1,1,1;
            inv_sigma = inv_sigma*exp_config_odometry_weight;
            Matrix6d info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
            e->setInformation(info);
            graph.addEdge(e);

            vEdgesOdom.push_back(e);
        }
    }

    // 接下来： 以 mms, objs 构造图.
    // Initialize objects vertices and add edges of camera-objects 2d observations 
    int objectid_in_edge = 0;
    int current_ob_id = 0;  
    int symplaneid_in_edge = 0;

    bool bUseProbThresh = Config::ReadValue<double>("Dynamic.Optimizer.UseProbThresh") > 0.01;
    double config_prob_thresh = Config::ReadValue<double>("Dynamic.Optimizer.EllipsoidProbThresh");
    int count_jump_prob_thresh = 0;
    if(bUseProbThresh)
        std::cout << "Use prob threshold, please make sure this is at least the second time running optimization!!" << std::endl;

    int count_partial_3d_planes = 0;
    int count_pointmodel_num = 0;
    std::vector<std::vector<g2o::ConstrainPlane*>> vwcPlanes;
    std::map<int,int> mmInstanceObservationNum;
    for(int object_id=0; object_id<objects_num; object_id++ )        
    {
        Object &obj = objs[object_id];
        int instance = obj.instance_id;
        int label = obj.pEllipsoid->miLabel;
        // 查语义表格判断该物体是否具有对称性质
        bool bSemanticSymmetry = IsSymmetry(label);

        if(bUseProbThresh)
        {
            // 因为obj中的概率需要在优化完之后做更新.
            if(obj.pEllipsoid->prob < config_prob_thresh) {
                count_jump_prob_thresh++;
                continue;
            }
        }

        // Add objects vertices
        g2o::VertexEllipsoidXYZABCYaw *vEllipsoid = new g2o::VertexEllipsoidXYZABCYaw();
        vEllipsoid->setEstimate(*obj.pEllipsoid);
        vEllipsoid->setId(graph.vertices().size());
        vEllipsoid->setFixed(false);
        graph.addVertex(vEllipsoid);
        vEllipsoidVertexMaps.insert(make_pair(instance, vEllipsoid)); 

        // // Add gravity prior
        // if(mbGroundPlaneSet && mbSetGravityPrior ){
        //     g2o::EdgeEllipsoidGravityPlanePrior *vGravityPriorEdge = new g2o::EdgeEllipsoidGravityPlanePrior;
        //     vGravityPriorEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));

        //     vGravityPriorEdge->setMeasurement(mGroundPlaneNormal);  
        //     Matrix<double,1,1> inv_sigma;
        //     inv_sigma << 1 * dGravityPriorScale;
        //     MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
        //     vGravityPriorEdge->setInformation(info);
            
        //     graph.addEdge(vGravityPriorEdge);
        //     edgesEllipsoidGravityPlanePrior.push_back(vGravityPriorEdge);
            
        // }

        // std::vector<g2o::ConstrainPlane*> vCPlanesWorld;
        // MatrixXd world_constrain_planes; world_constrain_planes.resize(0,4);
        // 添加 3d 约束 : 来自三维立方体构造时所用的三维平面.
        for( int i=0; i<obj.measurementIDs.size(); i++ )
        {
            Measurement& measurement = mms[obj.measurementIDs[i]];
            int instance = measurement.instance_id;
            
            Observation3D& ob_3d = measurement.ob_3d;
            g2o::ellipsoid* pObj_ob = ob_3d.pObj;
            
            if( pObj_ob == NULL ) continue;

            // if( i < 3 ) std::cout << "pObj_ob->prob : " << pObj_ob->prob << std::endl;

            int frame_index = measurement.ob_3d.pFrame->frame_seq_id;
            auto vEllipsoid = vEllipsoidVertexMaps[instance];  
            auto vSE3 = vSE3Vertex[frame_index];

            std::vector<g2o::ConstrainPlane*> vCPlanes = pObj_ob->mvCPlanes;
            // MatrixXd mPlanesParam = pObj_ob->cplanes;
            int plane_num = vCPlanes.size();

            g2o::SE3Quat &Twc = measurement.ob_2d.pFrame->cam_pose_Twc;

            if(!mbClose2DConstrain){
                for( int i=0;i<plane_num;i++)
                {
                    g2o::ConstrainPlane* pCPlane = vCPlanes[i];
                    Vector4d planeVec = pCPlane->pPlane->param.head(4); // local coordinate

                    // 为边缘平面添加特制的约束
                    if(pCPlane->valid){
                        g2o::EdgeSE3EllipsoidPlane* pEdge = new g2o::EdgeSE3EllipsoidPlane;
                        pEdge->setId(graph.edges().size());
                        pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
                        pEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
                        pEdge->setMeasurement(planeVec);

                        Matrix<double,1,1> inv_sigma;
                        inv_sigma << 1 * config_ellipsoid_2d_scale;
                        MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
                        pEdge->setInformation(info);
                        pEdge->setRobustKernel( new g2o::RobustKernelHuber() );

                        graph.addEdge(pEdge);
                        vEdgeEllipsoidPlane.push_back(pEdge);
                        mapInsEdgePlanes[instance].push_back(pEdge);
                        
                        mmInstanceObservationNum[instance]++;
                    }

                    // update : 2020-7-3 
                    // 二维约束中的局部平面统一全部忽略.
                    // else
                    // {
                    //     // 局部平面
                    //     g2o::EdgeSE3EllipsoidPlanePartial* pEdge = new g2o::EdgeSE3EllipsoidPlanePartial;
                    //     pEdge->setId(graph.edges().size());
                    //     pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
                    //     pEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
                    //     pEdge->setMeasurement(planeVec);

                    //     Matrix<double,1,1> inv_sigma;
                    //     inv_sigma << 1 * pObj_ob->prob * config_ellipsoid_2d_scale;
                    //     MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
                    //     pEdge->setInformation(info);
                    //     pEdge->setRobustKernel( new g2o::RobustKernelHuber() );

                    //     if(mbOpenPartialObservation)    // 注意需要开放才生效
                    //         graph.addEdge(pEdge);
                    //     vEdgeEllipsoidPlanePartial.push_back(pEdge);
                    //     mapInsEdgePlanesPartial[instance].push_back(pEdge);
                    // } // valid plane
                } // plane 遍历
            } // 2D Constrain flag


            // 更新: 2020-6-17日, 添加3d-ellipsoid之间的约束.
            // if(bOpen3DEllipsoidEdges)
            // {
            //     // 局部3d观测:pObj_ob
            //     g2o::EdgeSE3EllipsoidCenterFull* pEdge = new g2o::EdgeSE3EllipsoidCenterFull;
            //     pEdge->setId(graph.edges().size());
            //     pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
            //     pEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
            //     pEdge->setMeasurement(*pObj_ob);

            //     Matrix<double,1,1> inv_sigma;
            //     inv_sigma << 1 * pObj_ob->prob;
            //     MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
            //     pEdge->setInformation(info);

            //     graph.addEdge(pEdge);
            //     vEdgeSE3EllipsoidCenterFull.push_back(pEdge);
            //     // mapInsEdgePlanes[instance].push_back(pEdge);
            // }

            // 更新： 切换成完整约束
            bool bPointModel = pObj_ob->bPointModel;    // 参与不能是点模型.
            if(bPointModel) count_pointmodel_num++; // 统计
            if(bOpen3DEllipsoidEdges && !bPointModel){
                // bool bUsePlanesAs3DConstrains = Config::Get<double>("Optimizer.Edges.3DEllipsoid.PlanesAsConstrains") > 0;

                // 使用切平面的3d约束
                // ****************** 7-3 更新代码: 添加未被遮挡的平面 ********************
                // 要不先测试简单的，所有平面都放进来. 取消三维约束.
                if(bUsePlanesAs3DConstrains)
                {
                    // std::vector<g2o::plane*> vecPlanes = pObj_ob->GetCubePlanes();
                    // 开始判断是否在界外
                    
                    std::vector<g2o::plane*> vecPlanes = pObj_ob->GetCubePlanesInImages(g2o::SE3Quat(),calib,iRows, iCols, 30);
                    // 看怎么封装好一点，把边缘的都给我去掉.

                    count_partial_3d_planes += (6 - vecPlanes.size());

                    // 将界内的边构建并加入到图优化中
                    // 先尝试普通平面边，之后再tm尝试带法向量约束的边.

                    for(int i=0;i<vecPlanes.size();i++)
                    {
                        // 取一个平面
                        g2o::plane* ppl = vecPlanes[i];
                        Vector4d planeVec = ppl->param;

                        if(bUseNormalConstrain)
                        {
                            int flag_valid_angle =  !bSemanticSymmetry ? 1 : 0;
                            g2o::EdgeSE3EllipsoidPlaneWithNormal* pEdge = new g2o::EdgeSE3EllipsoidPlaneWithNormal;
                            pEdge->setId(graph.edges().size());
                            pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
                            pEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
                            pEdge->setMeasurement(planeVec);

                            Matrix<double,2,1> inv_sigma;
                            inv_sigma << 1, 1/(config_plane_angle_sigma * 1 / 180.0 * M_PI) * flag_valid_angle;   // 距离, 角度标准差 ; 暂时不管
                            inv_sigma = inv_sigma * config_ellipsoid_3d_scale;
                            MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
                            pEdge->setInformation(info);
                            pEdge->setRobustKernel( new g2o::RobustKernelHuber() );

                            graph.addEdge(pEdge);
                            vEdgeEllipsoidPlane3DWithNormal.push_back(pEdge);

                            mmInstanceObservationNum[instance]++;
                        }
                        else // 不使用法向量约束
                        {
                            g2o::EdgeSE3EllipsoidPlane* pEdge = new g2o::EdgeSE3EllipsoidPlane;
                            pEdge->setId(graph.edges().size());
                            pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
                            pEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
                            pEdge->setMeasurement(planeVec);

                            Matrix<double,1,1> inv_sigma;
                            inv_sigma << 1 * config_ellipsoid_3d_scale;
                            MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
                            pEdge->setInformation(info);
                            pEdge->setRobustKernel( new g2o::RobustKernelHuber() );

                            graph.addEdge(pEdge);
                            vEdgeEllipsoidPlane3D.push_back(pEdge);
                            // mapInsEdgePlanes[instance].push_back(pEdge);
                        }
                    }
                }
                else 
                {
                    // 完整3d约束
                    g2o::EdgeSE3Ellipsoid7DOF* vEllipsoid3D = new g2o::EdgeSE3Ellipsoid7DOF; 
                    vEllipsoid3D->setId(graph.edges().size()); 
                    vEllipsoid3D->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
                    vEllipsoid3D->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));                
                    vEllipsoid3D->setMeasurement(*pObj_ob); 

                    Vector7d inv_sigma;
                    inv_sigma << 1,1,1,1,1,1,1;
                    // if(mbOpen3DProb)
                    //     inv_sigma = inv_sigma * sqrt(pObj_ob->prob);
                    // Matrix7d info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal() * config_ellipsoid_3d_scale * pObj_ob->prob;
                    Matrix7d info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal() * config_ellipsoid_3d_scale * pObj_ob->prob;
                    vEllipsoid3D->setInformation(info);
                    vEllipsoid3D->setRobustKernel( new g2o::RobustKernelHuber() );

                    graph.addEdge(vEllipsoid3D);
                    vEllipsoid3DEdges.push_back(vEllipsoid3D);
                }
            }

            bool bOpenYawConstrain = false;
            // 旋转角度观测约束.
        
        } // 观测遍历

    } // objs 遍历

    // 处理关系
    bool bOpenPlaneOptimization = Config::ReadValue<double>("Dynamic.Optimizer.OptimizeRelationPlane") > 0;
    if(bOpenPlaneOptimization){
        std::vector<g2o::EdgePlaneSE3*> vEdgeRelations;
        if(mbRelationLoaded){
            // 该函数加载所有平面观测到图优化中
            // cout << "CLOSE RELATION OPTIMIZATION." << endl;
            LoadRelationsToGraph(mRelations, mSupportingPlanes, graph, vSE3Vertex, vEdgeRelations);
            mbRelationLoaded = false;

            // Output Information of Relation Planes
            cout << " ----- Relation Planes -----" << endl;
            cout<<  "   - SupportingPlanes: " << mSupportingPlanes.size() << endl;
            cout<<  "   - Relations: " << mRelations.size() << endl;
            cout<<  "   - Edge Splane-Plane: " << vEdgeRelations.size() << endl;

            cout << endl << endl;
        }
    }

    // output 
    int valid_plane_num = vEdgeEllipsoidPlane.size();
    int partial_plane_num = vEdgeEllipsoidPlanePartial.size();
    int total_plane_num = valid_plane_num + partial_plane_num;
    std::cout << std::endl << " ---- GRAPH INFORMATION ---- : " << std::endl;
    cout << " * Object Num : " << objects_num << endl;
    cout<<  " * Vertices: "<<graph.vertices().size()<<endl;
    cout << "   - SE3: " << vSE3Vertex.size() << endl;
    cout << "   - Object: " << vEllipsoidVertexMaps.size() << endl;
    cout << " * Edges: " << graph.edges().size() << endl;

    if(!mbOpenPartialObservation)
        cout << "   - Ellipsoid-Plane: " << total_plane_num << "( partial " << partial_plane_num << " [CLOSED])" << endl;
    else
        cout << "   - Ellipsoid-Plane: " << total_plane_num << "( partial " << partial_plane_num << " )" << endl;
    if(bOpen3DEllipsoidEdges){
        cout << "   - Ellipsoid-center: " << vEdgeSE3EllipsoidCenterFull.size() << std::endl;
        cout << "   - Ellipsoid-3D: " << vEllipsoid3DEdges.size() << std::endl;
        cout << "   - Ellipsoid-Plane-3D: " << vEdgeEllipsoidPlane3D.size() << "(filter partial:" << count_partial_3d_planes << ")" << std::endl;
        cout << "   - Ellipsoid-Plane-3DWithNormal: " << vEdgeEllipsoidPlane3DWithNormal.size() << "(filter partial:" << count_partial_3d_planes << ")" << std::endl;
    }
    cout << "   - Odom: " << vEdgesOdom.size() << endl;
    cout<<  "   - Gravity: " << edgesEllipsoidGravityPlanePrior.size() << endl;
    cout << " * Object Measurements : " << mms.size() << endl;
    cout << "   -- Point-Model: " << count_pointmodel_num << endl;

    cout << endl;

    if(bUseProbThresh){
        std::cout << " * ProbThresh : " << config_prob_thresh << std::endl;
        std::cout << " * Jump objects : " << count_jump_prob_thresh << std::endl;
    }

    // nan check
    bool bOpenNanCheck = false;
    if(bOpenNanCheck)
    {
        std::cout << "Begin NaN checking ... " << std::endl;
        // odom  vEdgesOdom
        // 2d vEdgeEllipsoidPlane
        // 3d vEdgeEllipsoidPlane3DWithNormal

        int nan_count = 0;
        for(auto edge : vEdgesOdom)
        {
            edge->computeError();
            double chi2 = edge->chi2();
            if(std::isnan(chi2))
            {
                // 输出
                nan_count++;
            }
        }
        std::cout << "odom nan : " << nan_count << std::endl;

        nan_count = 0;
        for(auto edge : vEdgeEllipsoidPlane)
        {
            edge->computeError();
            double chi2 = edge->chi2();
            if(std::isnan(chi2))
            {
                // 输出
                nan_count++;
            }
        }
        std::cout << "2d plane nan : " << nan_count << std::endl;

        nan_count = 0;
        for(auto edge : vEdgeEllipsoidPlane3DWithNormal)
        {
            edge->computeError();
            double chi2 = edge->chi2();
            if(std::isnan(chi2))
            {
                // 输出
                nan_count++;
            }
        }
        std::cout << "3d plane with normal nan : " << nan_count << std::endl;
        
        std::cout << "[NAN_CHECK]Press any key to continue .. " << std::endl;
        getchar();
    }

    int num_optimize = Config::Get<double>("Optimizer.Optimize.Num");
    if(graph.edges().size()>0){
        std::cout << "Begin Optimization..." << std::endl;
        graph.initializeOptimization();
        graph.optimize( num_optimize );  //optimization step
        std::cout << "Optimization done." << std::endl;

        // Update estimated ellispoids to the map
        // 更新结果到 objs
        for( int i = 0 ; i< objs.size() ; i++)
        {
            g2o::ellipsoid* pEllipsoid = objs[i].pEllipsoid;
            if(pEllipsoid!=NULL)
            {
                int instance = objs[i].instance_id;
                if(vEllipsoidVertexMaps.find(instance) == vEllipsoidVertexMaps.end()) continue;
                auto pEVertex = vEllipsoidVertexMaps[instance];
                
                bool colorSet = pEllipsoid->isColorSet();
                Vector4d old_color;
                if(colorSet) old_color = pEllipsoid->getColorWithAlpha();
                (*pEllipsoid) = pEVertex->estimate();
                pEllipsoid->miInstanceID = instance;
                if(colorSet) pEllipsoid->setColor(old_color.head(3), old_color[3]);   // 保持颜色不变.
            }
        }

        // 更新 Frames 轨迹; 也应该用临时结构体，而非直接改frame
        camTraj.resize(total_frame_number);
        for( int i=0; i< total_frame_number; i++ )
        {
            //  直接修改到帧中
            // 2020-6-2 让其生效. 引入 Save/Load 实现 debugging
            // Frame* pF = pFrames[i];
            // pF->cam_pose_Tcw = vSE3Vertex[i]->estimate();
            // pF->cam_pose_Twc = pF->cam_pose_Tcw.inverse();

            // 暂存版本
            SE3QuatWithStamp* pPoseTimestamp = new SE3QuatWithStamp;
            pPoseTimestamp->pose = vSE3Vertex[i]->estimate().inverse();  // Tcw -> Twc
            pPoseTimestamp->timestamp = pFrames[i]->timestamp;
            camTraj[i] = pPoseTimestamp;
        }
    }
    else 
    {
        std::cout << " No observations valid." << std::endl;
    }

    // Output : 以物体为核心, 输出所有约束边对应的error
    OutputOptimizationEdgeErrors(objs, mapInsEdgePlanes, mapInsEdgePlanesPartial);
    OutputInstanceObservationNum(mmInstanceObservationNum);

    // Output optimization information.
    // object list
    ofstream out_obj("./object_list_nonparam.txt");
    auto iter = vEllipsoidVertexMaps.begin();
    for(;iter!=vEllipsoidVertexMaps.end();iter++)
    {
        out_obj << iter->first << "\t" << iter->second->estimate().toMinimalVector().transpose() 
            << "\t" << iter->second->estimate().miLabel << std::endl;
    }
    out_obj.close();

    return;
}

// 注意： 目前不会修改传入的各帧!
std::vector<g2o::ellipsoid*> Optimizer::OptimizeUsingQuadricSLAM(std::vector<Frame *> &pFrames, Measurements& mms, Objects& objs, Trajectory& camTraj, Matrix3d& mCalib, int rows, int cols)
{
    // ************************ LOAD CONFIGURATION ************************
    double config_odometry_weight = Config::ReadValue<double>("DEBUG.ODOM.WEIGHT");
    double exp_config_odometry_weight = config_odometry_weight; // normal
    cout << " * DEBUG-Odometry weight : " << exp_config_odometry_weight << " ( 10^" << config_odometry_weight << ")" << endl;

    bool bUseProbThresh = Config::ReadValue<double>("Dynamic.Optimizer.UseProbThresh") > 0.01;
    double config_prob_thresh = Config::ReadValue<double>("Dynamic.Optimizer.EllipsoidProbThresh");

    double config_ellipsoid_2d_scale = Config::ReadValue<double>("Optimizer.Edges.2D.Scale");
    // ************************************************************************

    // Initialize variables.
    int total_frame_number = int(pFrames.size());
    int objects_num = int(objs.size());

    // initialize graph optimization.
    g2o::SparseOptimizer graph;
    g2o::BlockSolverX::LinearSolverType* linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    graph.setAlgorithm(solver);
    graph.setVerbose(false);        // Set output.

    std::map<int, g2o::VertexEllipsoid*> vEllipsoidVertexMaps;
    std::vector<g2o::EdgeSE3EllipsoidProj*> edges, edgesValid, edgesInValid;
    std::vector<bool> validVec; validVec.resize(total_frame_number);
    std::vector<g2o::EdgeEllipsoidGravityPlanePrior *> edgesEllipsoidGravityPlanePrior;     // Gravity prior
    std::vector<g2o::VertexSE3Expmap*> vSE3Vertex;

    std::vector<g2o::EdgeSE3Expmap*> vEdgesOdom;
    std::vector<g2o::EdgeSE3EllipsoidProj*> vEdgeSE3Ellipsoid;

    std::map<int, std::vector<g2o::EdgeSE3EllipsoidPlane*>> mapInsEdgePlanes;
    std::map<int, std::vector<g2o::EdgeSE3EllipsoidPlanePartial*>> mapInsEdgePlanesPartial;
    std::map<int, g2o::VertexSE3Expmap*> mSE3Vertex;

    // Add SE3 vertices for camera poses
    // bool bSLAM_mode = (Config::Get<int>("Optimizer.SLAM.mode") == 1);   // Mapping Mode : Fix camera poses and mapping ellipsoids only
    
    // DEBUG: 保持 SLAM mode 关闭. 只衡量建图过程.
    bool bSLAM_mode = false;
    std::cout << " [ SLAM Mode : " << bSLAM_mode << " ] " << std::endl;
    for( int frame_index=0; frame_index< total_frame_number ; frame_index++) {
        g2o::SE3Quat curr_cam_pose_Twc = pFrames[frame_index]->cam_pose_Twc;

        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setId(graph.vertices().size());
        graph.addVertex(vSE3);
        vSE3->setEstimate(pFrames[frame_index]->cam_pose_Tcw); // Tcw
        if(!bSLAM_mode)
            vSE3->setFixed(true);       // Fix all the poses in mapping mode.
        else 
            vSE3->setFixed(frame_index == 0);   
        // vSE3Vertex.push_back(vSE3);
        mSE3Vertex.insert(make_pair(pFrames[frame_index]->frame_seq_id,vSE3));

        // Add odom edges if in SLAM Mode
        // 注意该部分在frame的id不能用作索引时有bug，考虑到现在不使用位姿优化功能，注释掉该部分
    //     if(bSLAM_mode && frame_index > 0){
    //         g2o::SE3Quat prev_cam_pose_Tcw = pFrames[frame_index-1]->cam_pose_Twc.inverse();
    //         g2o::SE3Quat curr_cam_pose_Tcw = curr_cam_pose_Twc.inverse();

    //         // 最后的 inverse: 将 wc 转为 cw
    //         // g2o::SE3Quat odom_val = (curr_cam_pose_Tcw*prev_cam_pose_Tcw.inverse()).inverse();; //  odom_wc * To = T1
    //         g2o::SE3Quat odom_val = curr_cam_pose_Tcw*prev_cam_pose_Tcw.inverse(); 

    //         g2o::EdgeSE3Expmap* e = new g2o::EdgeSE3Expmap();
    //         e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>( mSE3Vertex[frame_index-1] ));
    //         e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>( mSE3Vertex[frame_index] ));
    //         e->setMeasurement(odom_val);

    //         e->setId(graph.edges().size());
    //         Vector6d inv_sigma;inv_sigma<<1,1,1,1,1,1;
    //         inv_sigma = inv_sigma*exp_config_odometry_weight;
    //         Matrix6d info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
    //         e->setInformation(info);
    //         graph.addEdge(e);

    //         vEdgesOdom.push_back(e);
    //     }
    }

    // 接下来： 以 mms, objs 构造图.
    // Initialize objects vertices and add edges of camera-objects 2d observations 
    int objectid_in_edge = 0;
    int current_ob_id = 0;  
    int symplaneid_in_edge = 0;

    int count_jump_prob_thresh = 0;
    for(int object_id=0; object_id<objects_num; object_id++ )        
    {
        Object &obj = objs[object_id];
        int instance = obj.instance_id;

        if(obj.pEllipsoid==NULL)  continue;     // 初始化失败的物体直接跳过去.

        if(bUseProbThresh)
        {
            // 因为obj中的概率需要在优化完之后做更新.
            if(obj.pEllipsoid->prob < config_prob_thresh) {
                count_jump_prob_thresh++;
                continue;
            }
        }

        // Add objects vertices
        g2o::VertexEllipsoid *vEllipsoid = new g2o::VertexEllipsoid();
        vEllipsoid->setEstimate(*obj.pEllipsoid);
        vEllipsoid->setId(graph.vertices().size());
        vEllipsoid->setFixed(false);
        graph.addVertex(vEllipsoid);
        vEllipsoidVertexMaps.insert(make_pair(instance, vEllipsoid)); 

        // 添加 3d 约束 : 来自三维立方体构造时所用的三维平面.
        for( int i=0; i<obj.measurementIDs.size(); i++ )
        {
            Measurement& measurement = mms[obj.measurementIDs[i]];
            int instance = measurement.instance_id;
            
            Observation& ob_2d = measurement.ob_2d;

            Vector4d bbox = ob_2d.bbox;

            int frame_index = ob_2d.pFrame->frame_seq_id;
            auto vEllipsoid = vEllipsoidVertexMaps[instance];  
            auto vSE3 = mSE3Vertex[frame_index];

            g2o::SE3Quat &Twc = measurement.ob_2d.pFrame->cam_pose_Twc;
            
            // 此处使用点模型的距离直接互相约束
            // 局部观测得到的物体 pObj_ob
            // 世界系下的物体Vertex vEllipsoid
            g2o::EdgeSE3EllipsoidProj* pEdge = new g2o::EdgeSE3EllipsoidProj();
            pEdge->setId(graph.edges().size());
            pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
            pEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
            pEdge->setMeasurement(bbox);

            Matrix<double,4,1> inv_sigma;
            inv_sigma << 1,1,1,1;
            inv_sigma = inv_sigma * config_ellipsoid_2d_scale;
            MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
            pEdge->setInformation(info);

            pEdge->setKalib(mCalib);

            // Two conditions for valid 2d edges:
            // 1) [closed] visibility check: see the comments of function checkVisibility for detail.
            // 2) NaN check
            bool check_visibility = true;
            bool c1 = (!check_visibility) || checkVisibility(pEdge, vSE3, vEllipsoid, mCalib, rows, cols);

            pEdge->computeError();
            double e_error = pEdge->chi2();
            bool c2 = !isnan(e_error);  // NaN check

            if( c1 && c2 ){
                graph.addEdge(pEdge);
                edgesValid.push_back(pEdge);  // valid edges and invalid edges are for debug output
            }
            else
                edgesInValid.push_back(pEdge);   

            vEdgeSE3Ellipsoid.push_back(pEdge);
        }

    } // objs 遍历

    // output 
    std::cout << std::endl << " ---- GRAPH INFORMATION ---- : " << std::endl;
    cout << " * Object Num : " << objects_num << endl;
    cout<<  " * Vertices: "<<graph.vertices().size()<<endl;
    cout << "   - SE3: " << mSE3Vertex.size() << endl;
    cout << "   - Object: " << vEllipsoidVertexMaps.size() << endl;
    cout << " * Edges: " << graph.edges().size() << endl;

    cout << "   - Ellipsoid-SE3: " << vEdgeSE3Ellipsoid.size() << endl;
    cout << "   --- Valid / Invalid : " << edgesValid.size() << " / " << edgesInValid.size() << endl;
    
    cout << "   - Odom: " << vEdgesOdom.size() << endl;
    cout << endl;

    if(bUseProbThresh){
        std::cout << " * ProbThresh : " << config_prob_thresh << std::endl;
        std::cout << " * Jump objects : " << count_jump_prob_thresh << std::endl;
    }

    int num_optimize = Config::Get<double>("Optimizer.Optimize.Num");
    if(graph.edges().size()>0){
        graph.initializeOptimization();
        graph.optimize( num_optimize );  //optimization step

        cout << "Optimization done." << endl;
        // Update estimated ellispoids to the map
        // 更新结果到 objs
        bool bUpdateInput = false;
        if(bUpdateInput){
            for( int i = 0 ; i< objs.size() ; i++)
            {
                g2o::ellipsoid* pEllipsoid = objs[i].pEllipsoid;
                if(pEllipsoid!=NULL)
                {
                    int instance = objs[i].instance_id;
                    if(vEllipsoidVertexMaps.find(instance) == vEllipsoidVertexMaps.end()) continue;
                    auto pEVertex = vEllipsoidVertexMaps[instance];
                    
                    bool colorSet = pEllipsoid->isColorSet();
                    Vector3d old_color;
                    if(colorSet) old_color = pEllipsoid->getColor();
                    (*pEllipsoid) = pEVertex->estimate();
                    pEllipsoid->miInstanceID = instance;
                    if(colorSet) pEllipsoid->setColor(old_color);   // 保持颜色不变.
                }
            }

            // cout << "Update pose in frames ... " << endl;
            // // 更新 Frames 轨迹; 也应该用临时结构体，而非直接改frame
            // camTraj.resize(total_frame_number);
            // for( int i=0; i< total_frame_number; i++ )
            // {
            //     //  直接修改到帧中
            //     // Frame* pF = pFrames[i];
            //     // pF->cam_pose_Tcw = vSE3Vertex[i]->estimate();
            //     // pF->cam_pose_Twc = pF->cam_pose_Tcw.inverse();

            //     // 暂存版本
            //     SE3QuatWithStamp* pPoseTimestamp = new SE3QuatWithStamp;
            //     pPoseTimestamp->pose = vSE3Vertex[i]->estimate().inverse();  // Tcw -> Twc
            //     pPoseTimestamp->timestamp = pFrames[i]->timestamp;
            //     camTraj[i] = pPoseTimestamp;
            // }

        }
    }
    else 
    {
        std::cout << " No observations valid." << std::endl;
    }

    // Output optimization information.
    // object list
    cout << "output objecct_list ..." << endl;
    ofstream out_obj("./object_list_quadricslam.txt");
    auto iter = vEllipsoidVertexMaps.begin();
    for(;iter!=vEllipsoidVertexMaps.end();iter++)
    {
        out_obj << iter->first << "\t" << iter->second->estimate().toMinimalVector().transpose() 
            << "\t" << iter->second->estimate().miLabel << std::endl;
    }
    out_obj.close();

    std::vector<g2o::ellipsoid*> vpObjs; vpObjs.reserve(vEllipsoidVertexMaps.size());
    for(auto& pair : vEllipsoidVertexMaps)
    {
        g2o::ellipsoid* pE = new g2o::ellipsoid(pair.second->estimate());
        vpObjs.push_back(pE);
    }
    return vpObjs;
}

/*
*   Baseline: 使用点模型的优化!
*/
void Optimizer::OptimizeWithDataAssociationUsingMultiplanesPointModel(std::vector<Frame *> &pFrames, Measurements& mms, Objects& objs, Trajectory& camTraj) {
    // ************************ LOAD CONFIGURATION ************************
    double config_ellipsoid_3d_scale = Config::ReadValue<double>("Optimizer.Edges.3DEllipsoid.Scale");
    bool mbSetGravityPrior = Config::Get<int>("Optimizer.Edges.GravityPrior.Open") == 1;  
    double dGravityPriorScale = Config::Get<double>("Optimizer.Edges.GravityPrior.Scale");
    bool mbOpenPartialObservation = Config::Get<int>("Optimizer.PartialObservation.Open") == 1;
    bool mbOpen3DProb = true;  

    // OUTPUT
    std::cout << " -- Optimization parameters : " << std::endl;
    if(mbGroundPlaneSet)
        std::cout << " [ Using Ground Plane: " << mGroundPlaneNormal.transpose() << " ] " << std::endl;

    if(!mbGroundPlaneSet || !mbSetGravityPrior )   
        std::cout << " * Gravity Prior : closed." << std::endl;
    else
        std::cout << " * Gravity Prior : Open." << std::endl;
    
    cout<<" * Scale_3dedge: " << config_ellipsoid_3d_scale << endl;
    cout<<" * Scale_GravityPrior: " << dGravityPriorScale << endl;

    double config_odometry_weight = Config::ReadValue<double>("DEBUG.ODOM.WEIGHT");
    // double exp_config_odometry_weight = pow(10, config_odometry_weight);    //exp
    double exp_config_odometry_weight = config_odometry_weight; // normal
    cout << " * DEBUG-Odometry weight : " << exp_config_odometry_weight << " ( 10^" << config_odometry_weight << ")" << endl;
    // ************************************************************************

    // Initialize variables.
    int total_frame_number = int(pFrames.size());
    int objects_num = int(objs.size());

    // initialize graph optimization.
    g2o::SparseOptimizer graph;
    g2o::BlockSolverX::LinearSolverType* linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    graph.setAlgorithm(solver);
    graph.setVerbose(false);        // Set output.

    std::map<int, g2o::VertexEllipsoidXYZ*> vEllipsoidVertexMaps;
    std::vector<g2o::EdgeSE3EllipsoidProj*> edges, edgesValid, edgesInValid;
    std::vector<bool> validVec; validVec.resize(total_frame_number);
    std::vector<g2o::EdgeEllipsoidGravityPlanePrior *> edgesEllipsoidGravityPlanePrior;     // Gravity prior
    std::vector<g2o::VertexSE3Expmap*> vSE3Vertex;

    std::vector<g2o::EdgeSE3Ellipsoid9DOF*> vEllipsoid3DEdges;
    std::vector<g2o::EdgeSE3Expmap*> vEdgesOdom;
    std::vector<g2o::EdgeSE3EllipsoidPlane*> vEdgeEllipsoidPlane;
    std::vector<g2o::EdgeSE3EllipsoidPlanePartial*> vEdgeEllipsoidPlanePartial;
    std::vector<g2o::EdgeSE3EllipsoidCenter*> vEdgeSE3EllipsoidCenter;

    std::map<int, std::vector<g2o::EdgeSE3EllipsoidPlane*>> mapInsEdgePlanes;
    std::map<int, std::vector<g2o::EdgeSE3EllipsoidPlanePartial*>> mapInsEdgePlanesPartial;

    // Add SE3 vertices for camera poses
    bool bSLAM_mode = (Config::Get<int>("Optimizer.SLAM.mode") == 1);   // Mapping Mode : Fix camera poses and mapping ellipsoids only
    std::cout << " [ SLAM Mode : " << bSLAM_mode << " ] " << std::endl;
    for( int frame_index=0; frame_index< total_frame_number ; frame_index++) {
        g2o::SE3Quat curr_cam_pose_Twc = pFrames[frame_index]->cam_pose_Twc;

        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setId(graph.vertices().size());
        graph.addVertex(vSE3);
        vSE3->setEstimate(pFrames[frame_index]->cam_pose_Tcw); // Tcw
        if(!bSLAM_mode)
            vSE3->setFixed(true);       // Fix all the poses in mapping mode.
        else 
            vSE3->setFixed(frame_index == 0);   
        vSE3Vertex.push_back(vSE3);

        // Add odom edges if in SLAM Mode
        if(bSLAM_mode && frame_index > 0){
            g2o::SE3Quat prev_cam_pose_Tcw = pFrames[frame_index-1]->cam_pose_Twc.inverse();
            g2o::SE3Quat curr_cam_pose_Tcw = curr_cam_pose_Twc.inverse();

            // 最后的 inverse: 将 wc 转为 cw
            // g2o::SE3Quat odom_val = (curr_cam_pose_Tcw*prev_cam_pose_Tcw.inverse()).inverse();; //  odom_wc * To = T1
            g2o::SE3Quat odom_val = curr_cam_pose_Tcw*prev_cam_pose_Tcw.inverse(); 

            g2o::EdgeSE3Expmap* e = new g2o::EdgeSE3Expmap();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>( vSE3Vertex[frame_index-1] ));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>( vSE3Vertex[frame_index] ));
            e->setMeasurement(odom_val);

            e->setId(graph.edges().size());
            Vector6d inv_sigma;inv_sigma<<1,1,1,1,1,1;
            inv_sigma = inv_sigma*exp_config_odometry_weight;
            Matrix6d info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
            e->setInformation(info);
            graph.addEdge(e);

            vEdgesOdom.push_back(e);
        }
    }

    // 接下来： 以 mms, objs 构造图.
    // Initialize objects vertices and add edges of camera-objects 2d observations 
    int objectid_in_edge = 0;
    int current_ob_id = 0;  
    int symplaneid_in_edge = 0;

    bool bUseProbThresh = Config::ReadValue<double>("Dynamic.Optimizer.UseProbThresh") > 0.01;
    double config_prob_thresh = Config::ReadValue<double>("Dynamic.Optimizer.EllipsoidProbThresh");
    int count_jump_prob_thresh = 0;
    if(bUseProbThresh)
        std::cout << "Use prob threshold, please make sure this is at least the second time running optimization!!" << std::endl;

    std::vector<std::vector<g2o::ConstrainPlane*>> vwcPlanes;
    for(int object_id=0; object_id<objects_num; object_id++ )        
    {
        Object &obj = objs[object_id];
        int instance = obj.instance_id;

        if(bUseProbThresh)
        {
            // 因为obj中的概率需要在优化完之后做更新.
            if(obj.pEllipsoid->prob < config_prob_thresh) {
                count_jump_prob_thresh++;
                continue;
            }
        }

        // Add objects vertices
        g2o::VertexEllipsoidXYZ *vEllipsoid = new g2o::VertexEllipsoidXYZ();
        vEllipsoid->setEstimate(*obj.pEllipsoid);
        vEllipsoid->setId(graph.vertices().size());
        vEllipsoid->setFixed(false);
        graph.addVertex(vEllipsoid);
        vEllipsoidVertexMaps.insert(make_pair(instance, vEllipsoid)); 

        // 添加 3d 约束 : 来自三维立方体构造时所用的三维平面.
        for( int i=0; i<obj.measurementIDs.size(); i++ )
        {
            Measurement& measurement = mms[obj.measurementIDs[i]];
            int instance = measurement.instance_id;
            
            Observation3D& ob_3d = measurement.ob_3d;
            g2o::ellipsoid* pObj_ob = ob_3d.pObj;
            
            if( pObj_ob == NULL ) continue;

            // if( i < 3 ) std::cout << "pObj_ob->prob : " << pObj_ob->prob << std::endl;

            int frame_index = measurement.ob_3d.pFrame->frame_seq_id;
            auto vEllipsoid = vEllipsoidVertexMaps[instance];  
            auto vSE3 = vSE3Vertex[frame_index];

            g2o::SE3Quat &Twc = measurement.ob_2d.pFrame->cam_pose_Twc;
            
            // 此处使用点模型的距离直接互相约束
            // 局部观测得到的物体 pObj_ob
            // 世界系下的物体Vertex vEllipsoid
            g2o::EdgeSE3EllipsoidCenter* pEdge = new g2o::EdgeSE3EllipsoidCenter();
            pEdge->setId(graph.edges().size());
            pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
            pEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
            pEdge->setMeasurement(*pObj_ob);

            Matrix<double,1,1> inv_sigma;
            inv_sigma << 1 * config_ellipsoid_3d_scale;
            MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
            pEdge->setInformation(info);

            graph.addEdge(pEdge);
            vEdgeSE3EllipsoidCenter.push_back(pEdge);
        }

    } // objs 遍历

    // output 
    int valid_plane_num = vEdgeEllipsoidPlane.size();
    int partial_plane_num = vEdgeEllipsoidPlanePartial.size();
    int total_plane_num = valid_plane_num + partial_plane_num;
    std::cout << std::endl << " ---- GRAPH INFORMATION ---- : " << std::endl;
    cout << " * Object Num : " << objects_num << endl;
    cout<<  " * Vertices: "<<graph.vertices().size()<<endl;
    cout << "   - SE3: " << vSE3Vertex.size() << endl;
    cout << "   - Object: " << vEllipsoidVertexMaps.size() << endl;
    cout << " * Edges: " << graph.edges().size() << endl;

    cout << "   - Ellipsoid-Center: " << vEdgeSE3EllipsoidCenter.size() << endl;
    
    cout << "   - Odom: " << vEdgesOdom.size() << endl;
    cout<<  "   - Gravity: " << edgesEllipsoidGravityPlanePrior.size() << endl;

    cout << endl;

    if(bUseProbThresh){
        std::cout << " * ProbThresh : " << config_prob_thresh << std::endl;
        std::cout << " * Jump objects : " << count_jump_prob_thresh << std::endl;
    }

    int num_optimize = Config::Get<double>("Optimizer.Optimize.Num");
    if(graph.edges().size()>0){
        graph.initializeOptimization();
        graph.optimize( num_optimize );  //optimization step

        // Update estimated ellispoids to the map
        // 更新结果到 objs
        for( int i = 0 ; i< objs.size() ; i++)
        {
            g2o::ellipsoid* pEllipsoid = objs[i].pEllipsoid;
            if(pEllipsoid!=NULL)
            {
                int instance = objs[i].instance_id;
                if(vEllipsoidVertexMaps.find(instance) == vEllipsoidVertexMaps.end()) continue;
                auto pEVertex = vEllipsoidVertexMaps[instance];
                
                bool colorSet = pEllipsoid->isColorSet();
                Vector3d old_color;
                if(colorSet) old_color = pEllipsoid->getColor();
                (*pEllipsoid) = pEVertex->estimate();
                pEllipsoid->miInstanceID = instance;
                if(colorSet) pEllipsoid->setColor(old_color);   // 保持颜色不变.
            }
        }

        // 更新 Frames 轨迹; 也应该用临时结构体，而非直接改frame
        for( int i=0; i< total_frame_number; i++ )
        {
            //  直接修改到帧中
            // Frame* pF = pFrames[i];
            // pF->cam_pose_Tcw = vSE3Vertex[i]->estimate();
            // pF->cam_pose_Twc = pF->cam_pose_Tcw.inverse();

            // 暂存版本
            SE3QuatWithStamp* pPoseTimestamp = new SE3QuatWithStamp;
            pPoseTimestamp->pose = vSE3Vertex[i]->estimate().inverse();  // Tcw -> Twc
            pPoseTimestamp->timestamp = pFrames[i]->timestamp;
            camTraj[i] = pPoseTimestamp;
        }
    }
    else 
    {
        std::cout << " No observations valid." << std::endl;
    }

    // Output : 以物体为核心, 输出所有约束边对应的error
    OutputOptimizationEdgeErrors(objs, mapInsEdgePlanes, mapInsEdgePlanesPartial);

    // Output optimization information.
    // object list
    ofstream out_obj("./object_list_nonparam.txt");
    auto iter = vEllipsoidVertexMaps.begin();
    for(;iter!=vEllipsoidVertexMaps.end();iter++)
    {
        out_obj << iter->first << "\t" << iter->second->estimate().toMinimalVector().transpose() 
            << "\t" << iter->second->estimate().miLabel << std::endl;
    }
    out_obj.close();
}

int GetTotalObjectIndex(std::vector<Frame *> &pFrames, int frame_index, int index_in_frame)
{
    int total_id = 0;
    for(int i=0;i<frame_index;i++)
    {
        total_id+= pFrames[i]->mpLocalObjects.size();
    }
    return total_id+index_in_frame;
}

void Optimizer::GetOptimizedResult(Objects& objs, Measurements& mms)
{
    objs = mObjects;
    mms = mMeasurements;
}


/*********
 * 全新版本， 引入了 物体-平面 支撑关系
 */
// void Optimizer::OptimizeWithDataAssociationUsingMultiplanesWithRelations(std::vector<Frame *> &pFrames, int rows, int cols, Matrix3d &mCalib, 
//                 Measurements& mms, Objects& objs, Trajectory& camTraj, Relations& rls) {
//     // ************************ LOAD CONFIGURATION ************************
//     double config_ellipsoid_3d_scale = Config::Get<double>("Optimizer.Edges.3DEllipsoid.Scale");
//     bool mbSetGravityPrior = Config::Get<int>("Optimizer.Edges.GravityPrior.Open") == 1;  
//     double dGravityPriorScale = Config::Get<double>("Optimizer.Edges.GravityPrior.Scale");
//     bool mbOpen3DProb = true;  

//     // OUTPUT
//     std::cout << " -- Optimization parameters : " << std::endl;
//     if(mbGroundPlaneSet)
//         std::cout << " [ Using Ground Plane: " << mGroundPlaneNormal.transpose() << " ] " << std::endl;

//     if(!mbGroundPlaneSet || !mbSetGravityPrior )   
//         std::cout << " * Gravity Prior : closed." << std::endl;
//     else
//         std::cout << " * Gravity Prior : Open." << std::endl;
    
//     cout<<" * Scale_3dedge: " << config_ellipsoid_3d_scale << endl;
//     cout<<" * Scale_GravityPrior: " << dGravityPriorScale << endl;

//     double config_odometry_weight = Config::ReadValue<double>("DEBUG.ODOM.WEIGHT");
//     double exp_config_odometry_weight = pow(10, config_odometry_weight);
//     cout << " * DEBUG-Odometry weight : " << exp_config_odometry_weight << " ( 10^" << config_odometry_weight << ")" << endl;
//     // ************************************************************************

//     // Initialize variables.
//     int total_frame_number = int(pFrames.size());
//     int objects_num = int(objs.size());

//     // initialize graph optimization.
//     g2o::SparseOptimizer graph;
//     g2o::BlockSolverX::LinearSolverType* linearSolver;
//     linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
//     g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
//     g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
//     graph.setAlgorithm(solver);
//     graph.setVerbose(true);        // Set output.

//     std::map<int, g2o::VertexEllipsoidXYZABC*> vEllipsoidVertexMaps;
//     std::map<g2o::ellipsoid*, g2o::VertexEllipsoidXYZABC*> vEllipsoidVertexMapsEllipsoidPointer;
//     std::vector<g2o::EdgeSE3EllipsoidProj*> edges, edgesValid, edgesInValid;
//     std::vector<bool> validVec; validVec.resize(total_frame_number);
//     std::vector<g2o::EdgeEllipsoidGravityPlanePrior *> edgesEllipsoidGravityPlanePrior;     // Gravity prior
//     std::vector<g2o::VertexSE3Expmap*> vSE3Vertex;

//     std::vector<g2o::EdgeSE3Ellipsoid9DOF*> vEllipsoid3DEdges;
//     std::vector<g2o::EdgeSE3Expmap*> vEdgesOdom;
//     std::vector<g2o::EdgeSE3EllipsoidPlane*> vEdgeEllipsoidPlane;

//     // Add SE3 vertices for camera poses
//     bool bSLAM_mode = true;   // Mapping Mode : Fix camera poses and mapping ellipsoids only
//     for( int frame_index=0; frame_index< total_frame_number ; frame_index++) {
//         g2o::SE3Quat curr_cam_pose_Twc = pFrames[frame_index]->cam_pose_Twc;

//         g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
//         vSE3->setId(graph.vertices().size());
//         graph.addVertex(vSE3);
//         vSE3->setEstimate(pFrames[frame_index]->cam_pose_Tcw); // Tcw
//         if(!bSLAM_mode)
//             vSE3->setFixed(true);       // Fix all the poses in mapping mode.
//         else 
//             vSE3->setFixed(frame_index == 0);   
//         vSE3Vertex.push_back(vSE3);

//         // Add odom edges if in SLAM Mode
//         if(bSLAM_mode && frame_index > 0){
//             g2o::SE3Quat prev_cam_pose_Tcw = pFrames[frame_index-1]->cam_pose_Twc.inverse();
//             g2o::SE3Quat curr_cam_pose_Tcw = curr_cam_pose_Twc.inverse();

//             // 最后的 inverse: 将 wc 转为 cw
//             // g2o::SE3Quat odom_val = (curr_cam_pose_Tcw*prev_cam_pose_Tcw.inverse()).inverse();; //  odom_wc * To = T1
//             g2o::SE3Quat odom_val = curr_cam_pose_Tcw*prev_cam_pose_Tcw.inverse(); 

//             g2o::EdgeSE3Expmap* e = new g2o::EdgeSE3Expmap();
//             e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>( vSE3Vertex[frame_index-1] ));
//             e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>( vSE3Vertex[frame_index] ));
//             e->setMeasurement(odom_val);

//             e->setId(graph.edges().size());
//             Vector6d inv_sigma;inv_sigma<<1,1,1,1,1,1;
//             inv_sigma = inv_sigma*exp_config_odometry_weight;
//             Matrix6d info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
//             e->setInformation(info);
//             graph.addEdge(e);

//             vEdgesOdom.push_back(e);
//         }
//     }

//     // 接下来： 以 mms, objs 构造图.
//     // Initialize objects vertices and add edges of camera-objects 2d observations 
//     int objectid_in_edge = 0;
//     int current_ob_id = 0;  
//     int symplaneid_in_edge = 0;

//     std::vector<MatrixXd> vwcPlanes;
//     for(int object_id=0; object_id<objects_num; object_id++ )        
//     {
//         Object &obj = objs[object_id];
//         int instance = obj.instance_id;

//         // Add objects vertices
//         g2o::VertexEllipsoidXYZABC *vEllipsoid = new g2o::VertexEllipsoidXYZABC();
//         vEllipsoid->setEstimate(*obj.pEllipsoid);
//         vEllipsoid->setId(graph.vertices().size());
//         vEllipsoid->setFixed(false);
//         graph.addVertex(vEllipsoid);
//         vEllipsoidVertexMaps.insert(make_pair(instance, vEllipsoid)); 
//         vEllipsoidVertexMapsEllipsoidPointer.insert(make_pair(obj.pEllipsoid, vEllipsoid));

//         // // Add gravity prior
//         // if(mbGroundPlaneSet && mbSetGravityPrior ){
//         //     g2o::EdgeEllipsoidGravityPlanePrior *vGravityPriorEdge = new g2o::EdgeEllipsoidGravityPlanePrior;
//         //     vGravityPriorEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));

//         //     vGravityPriorEdge->setMeasurement(mGroundPlaneNormal);  
//         //     Matrix<double,1,1> inv_sigma;
//         //     inv_sigma << 1 * dGravityPriorScale;
//         //     MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
//         //     vGravityPriorEdge->setInformation(info);
            
//         //     graph.addEdge(vGravityPriorEdge);
//         //     edgesEllipsoidGravityPlanePrior.push_back(vGravityPriorEdge);
            
//         // }

//         MatrixXd world_constrain_planes; world_constrain_planes.resize(0,4);
//         // 添加 3d 约束 : 来自三维立方体构造时所用的三维平面.
//         for( int i=0; i<obj.measurementIDs.size(); i++ )
//         {
//             Measurement& measurement = mms[obj.measurementIDs[i]];
//             int instance = measurement.instance_id;
            
//             Observation3D& ob_3d = measurement.ob_3d;
//             g2o::ellipsoid* pObj_ob = ob_3d.pObj;
            
//             if( pObj_ob == NULL ) continue;

//             int frame_index = measurement.ob_3d.pFrame->frame_seq_id;
//             auto vEllipsoid = vEllipsoidVertexMaps[instance];  
//             auto vSE3 = vSE3Vertex[frame_index];

//             MatrixXd mPlanesParam = pObj_ob->cplanes;
//             int plane_num = mPlanesParam.rows();

//             g2o::SE3Quat &Twc = measurement.ob_2d.pFrame->cam_pose_Twc;
//             for( int i=0;i<plane_num;i++)
//             {
//                 Vector4d planeVec = mPlanesParam.row(i).head(4); // local coordinate
//                 g2o::EdgeSE3EllipsoidPlane* pEdge = new g2o::EdgeSE3EllipsoidPlane;
//                 pEdge->setId(graph.edges().size());
//                 pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
//                 pEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
//                 pEdge->setMeasurement(planeVec);

//                 Matrix<double,1,1> inv_sigma;
//                 inv_sigma << 1;
//                 MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
//                 pEdge->setInformation(info);

//                 graph.addEdge(pEdge);
//                 vEdgeEllipsoidPlane.push_back(pEdge);

//                 g2o::plane pl(planeVec); pl.transform(Twc);
//                 Vector4d plVecWorld = pl.param;
//                 VectorXd plwv = plVecWorld;
//                 addVecToMatirx(world_constrain_planes, plwv);
//             }
//         }

//         vwcPlanes.push_back(world_constrain_planes);
//     } // objs 遍历

//     // 本函数的核心部分:
//     //     添加Relations进来约束
//     int rls_num = rls.size();
//     for( int i= 0;i<rls_num; i++)
//     {
//         Relation& rl = rls[i];
//         g2o::SE3Quat& Twc = rl.pFrame->cam_pose_Twc;  // 由index在 vSE3Vertex 索引
//         g2o::ellipsoid* pE = rl.pEllipsoid; // 索引 Vertex.
//         g2o::plane* pPlane = rl.pPlane;
//         Frame* pFrame = rl.pFrame;
//         int frame_index = pFrame->frame_seq_id;

//         // object instance
//         int total_index = GetTotalObjectIndex(pFrames, frame_index, rl.obj_id);
//         Measurement& meas = mms[total_index];
//         int instance = meas.instance_id;
//         auto vEllipsoid = vEllipsoidVertexMaps[instance];  

//         // Frame pose Index
//         auto vSE3 = vSE3Vertex[frame_index];

        

//     }

//     // output 
//     std::cout << std::endl << " ---- GRAPH INFORMATION ---- : " << std::endl;
//     cout << " * Object Num : " << objects_num << endl;
//     cout<<  " * Vertices: "<<graph.vertices().size()<<endl;
//     cout << "   - SE3: " << vSE3Vertex.size() << endl;
//     cout << "   - Object: " << vEllipsoidVertexMaps.size() << endl;
//     cout << " * Edges: " << graph.edges().size() << endl;
//     cout << "   - Ellipsoid-Plane: " << vEdgeEllipsoidPlane.size() << endl;
//     cout << "   - Odom: " << vEdgesOdom.size() << endl;
//     cout<<  "   - Gravity: " << edgesEllipsoidGravityPlanePrior.size() << endl;

//     cout << endl;

//     if(graph.edges().size()>0){
//         graph.initializeOptimization();
//         graph.optimize( 10 );  //optimization step

//         // Update estimated ellispoids to the map
//         // 更新结果到 objs
//         for( int i = 0 ; i< objs.size() ; i++)
//         {
//             g2o::ellipsoid* pEllipsoid = objs[i].pEllipsoid;
//             if(pEllipsoid!=NULL)
//             {
//                 int instance = objs[i].instance_id;
//                 (*pEllipsoid) = vEllipsoidVertexMaps[instance]->estimate();
//                 pEllipsoid->cplanes = vwcPlanes[i]; // 配置世界系下的约束平面
//             }
//         }

//         // 更新 Frames 轨迹; 也应该用临时结构体，而非直接改frame
//         for( int i=0; i< total_frame_number; i++ )
//         {
//             //  直接修改到帧中
//             // Frame* pF = pFrames[i];
//             // pF->cam_pose_Twc = vSE3Vertex[i]->estimate();
//             // pF->cam_pose_Tcw = pF->cam_pose_Twc.inverse();

//             // 暂存版本
//             SE3QuatWithStamp* pPoseTimestamp = new SE3QuatWithStamp;
//             pPoseTimestamp->pose = vSE3Vertex[i]->estimate().inverse();  // Tcw -> Twc
//             pPoseTimestamp->timestamp = pFrames[i]->timestamp;
//             camTraj[i] = pPoseTimestamp;
//         }
//     }
//     else 
//     {
//         std::cout << " No observations valid." << std::endl;
//     }

//     // Output optimization information.
//     // object list
//     ofstream out_obj("./object_list_nonparam.txt");
//     auto iter = vEllipsoidVertexMaps.begin();
//     for(;iter!=vEllipsoidVertexMaps.end();iter++)
//     {
//         out_obj << iter->first << "\t" << iter->second->estimate().toMinimalVector().transpose() 
//             << "\t" << iter->second->estimate().miLabel << std::endl;
//     }
//     out_obj.close();
// }

} // namespace
