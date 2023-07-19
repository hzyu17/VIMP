/**
 * @file PlanarQuadrotorSDFExample.h
 * @author your name (hyu419@gatech.edu)
 * @brief Planar quadrotor with a planar sdf.
 * @version 0.1
 * @date 2023-06-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#define STRING(x) #x
#define XSTRING(x) STRING(x)

std::string source_root{XSTRING(SOURCE_ROOT)};

#include "robots/RobotSDFBase.h"
#include <gpmp2/kinematics/PointRobotModel.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactor.h>
#include <gtsam/inference/Symbol.h>
namespace vimp{

using pRSDF = gpmp2::ObstaclePlanarSDFFactor<gpmp2::PointRobotModel>;
using Base = RobotSDFBase<gpmp2::PointRobotModel, gpmp2::PlanarSDF, pRSDF>;
class PlanarQuadrotorSDFExample: public Base{

public:
    PlanarQuadrotorSDFExample(){}
    PlanarQuadrotorSDFExample(double epsilon, double radius, const std::string& field_file, const std::string& sdf_file=""): 
        Base(2, 1, source_root+"/data/vimp/2d_pR/field_multiobs_entropy_map2.csv", ""), 
        _eps(epsilon), 
        _r(radius)
        {   
            if (!field_file.empty()){
                Base::update_field_file(field_file);
            }
            default_sdf();
            generate_pr_sdf(*(Base::_psdf), radius);
        }

        void default_sdf(){
            /// map and sdf
            MatrixXd field{_m_io.load_csv(Base::_field_file)};

            // layout of SDF: Bottom-left is (0,0), length is +/- cell_size per grid.
            Point2 origin(-20, -10);
            double cell_size = 0.1;

            Base::_psdf = std::make_shared<gpmp2::PlanarSDF>(gpmp2::PlanarSDF(origin, cell_size, field));

        }

        void generate_pr_sdf(const gpmp2::PlanarSDF& sdf, double r){
            /// Robot model
            gpmp2::PointRobot pR(_ndof, _nlinks);
            gpmp2::BodySphereVector body_spheres;
            body_spheres.push_back(gpmp2::BodySphere(0, r, Point3(0.0, 0.0, 0.0)));
            Base::_robot = gpmp2::PointRobotModel(pR, body_spheres);

            Base::_psdf_factor = std::make_shared<pRSDF>(pRSDF(gtsam::symbol('x', 0), Base::_robot, sdf, 0.0, _eps));
        }
        
        inline void update_sdf(const gpmp2::PlanarSDF& sdf){
            Base::_psdf = std::make_shared<gpmp2::PlanarSDF>(sdf);
            Base::_psdf_factor = std::make_shared<pRSDF>(pRSDF(gtsam::symbol('x', 0), Base::_robot, sdf, 0.0, _eps));
        }

        std::tuple<VectorXd, MatrixXd> hinge_jacobian(const VectorXd& pose_PR) override{
            // The jacobian w.r.t. the point robot model.
            MatrixXd Jacobian_PR;
            VectorXd vec_err = _psdf_factor->evaluateError(pose_PR, Jacobian_PR);

            return std::make_tuple(vec_err, Jacobian_PR);
        }

        public:
            /// 2D point robot         
            double _eps, _r;

};

}