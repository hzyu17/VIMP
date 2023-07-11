/**
 * @file PlanarPointRobotSDFMultiObsExample.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief An example experiment settings of a planar point robot in multi obstacle env. 
 * Imported from the tested code in gpmp2.
 * @version 0.1
 * @date 2022-07-27
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include <gtsam/inference/Symbol.h>
#include <gpmp2/kinematics/PointRobotModel.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactor.h>
#include "robots/RobotSDFBase.h"

using namespace Eigen;

using pRSDF = gpmp2::ObstaclePlanarSDFFactor<gpmp2::PointRobotModel>;

namespace vimp{
using Base = RobotSDFBase<gpmp2::PointRobotModel, gpmp2::PlanarSDF, pRSDF>;
class PlanarPRSDFExample: public Base{
    public:
        PlanarPRSDFExample(double epsilon, double radius): Base(2, 1), 
                                                           _eps(epsilon), 
                                                           _r(radius)
        {
            default_sdf();
            generate_pr_sdf(*(Base::_psdf), radius);
        }

        void default_sdf(){
            /// map and sdf
            MatrixXd field{_m_io.load_csv("/home/hyu419/git/VIMP/vimp/data/vimp/2d_pR/field_multiobs_entropy_map2.csv")};

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

        public:
            /// 2D point robot         
            double _eps, _r;

};
}