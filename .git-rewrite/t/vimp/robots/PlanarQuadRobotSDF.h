/**
 * @file PlanarQuadRobotSDF.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Planar quadrotor with an internal sdf.
 * @version 0.1
 * @date 2023-07-01
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "robots/RobotSDFBase.h"
#include "dynamics/PlanarQuadDynamics.h"

// namespace vimp{

// class PlanarQuadRobotSDF: public RobotSDFBase{
// using Base = RobotSDFBase;
// public:
//     PlanarQuadRobotSDF():RobotSDFBase(6, 1){}

//     virtual ~RobotSDFBase(){}

//     void update_sdf(const SDF& sdf){
//         Base::_sdf = sdf;
//     };

//     void default_sdf(){
//         /// map and sdf
//         MatrixXd field{Base::_m_io.load_csv("/home/hzyu/git/VIMP/vimp/data/vimp/2d_pR/field_multiobs_entropy_map2.csv")};

//         // layout of SDF: Bottom-left is (0,0), length is +/- cell_size per grid.
//         gtsam::Point2 origin(-20, -10);
//         double cell_size = 0.1;

//         Base::_psdf = std::make_shared<gpmp2::PlanarSDF>(gpmp2::PlanarSDF(origin, cell_size, field));
//     };

//     void generate_pr_sdf(const gpmp2::PlanarSDF& sdf, double r){
//         /// Robot model
//         gpmp2::PointRobot pR(_ndof, _nlinks);
//         gpmp2::BodySphereVector body_spheres;
//         body_spheres.push_back(gpmp2::BodySphere(0, r, gtsam::Point3(0.0, 0.0, 0.0)));
//         Base::_robot = gpmp2::PointRobotModel(pR, body_spheres);

//         Base::_psdf_factor = std::make_shared<pRSDF>(pRSDF(gtsam::symbol('x', 0), Base::_robot, sdf, 0.0, _eps));
//     }
    
//     inline void update_sdf(const gpmp2::PlanarSDF& sdf){
//         Base::_psdf = std::make_shared<gpmp2::PlanarSDF>(sdf);
//         Base::_psdf_factor = std::make_shared<pRSDF>(pRSDF(gtsam::symbol('x', 0), Base::_robot, sdf, 0.0, _eps));
//     }

//     std::tuple<VectorXd, MatrixXd> hinge_jacobian_nonlinear_dyn(const VectorXd& pose) override{
//         MatrixXd Jacobian;
//         VectorXd vec_err = _psdf_factor->evaluateError(pose, Jacobian);
//         return std::make_tuple(vec_err, Jacobian);
//     }

//     public:
//         /// 2D point robot         
//         double _eps, _r;

// };


// }