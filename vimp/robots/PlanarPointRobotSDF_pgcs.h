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

#include <gtsam/inference/Symbol.h>
#include <gpmp2/kinematics/PointRobotModel.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactor.h>
#include "RobotSDFBase.h"

using namespace Eigen;

using pRSDF = gpmp2::ObstaclePlanarSDFFactor<gpmp2::PointRobotModel>;

namespace vimp{

class PlanarPRSDFExample: public RobotSDFBase<gpmp2::PointRobotModel, gpmp2::PlanarSDF, pRSDF>{
    public:
        PlanarPRSDFExample(double epsilon): _ndof(2), _nlinks(1), _eps(epsilon), _r(0.0){
            default_sdf();
            generate_pr_sdf(*_psdf, 0.0);
        }

        void default_sdf(){
            /// map and sdf
            MatrixXd field{_m_io.load_csv("/home/hongzhe/git/VIMP/vimp/data/vimp/2d_pR/field_multiobs_entropy_map2.csv")};

            // layout of SDF: Bottom-left is (0,0), length is +/- cell_size per grid.
            Point2 origin(-20, -10);
            double cell_size = 0.1;

            _psdf = std::make_shared<gpmp2::PlanarSDF>(gpmp2::PlanarSDF(origin, cell_size, field));

        }

        void generate_pr_sdf(const gpmp2::PlanarSDF& sdf, double r){
            /// Robot model
            gpmp2::PointRobot pR(_ndof, _nlinks);
            gpmp2::BodySphereVector body_spheres;
            body_spheres.push_back(gpmp2::BodySphere(0, r, Point3(0.0, 0.0, 0.0)));
            _robot = gpmp2::PointRobotModel(pR, body_spheres);

            _psdf_factor = std::make_shared<pRSDF>(pRSDF(gtsam::symbol('x', 0), _robot, sdf, 0.0, _eps));
        }
        
        inline void update_sdf(const gpmp2::PlanarSDF& sdf){
            _psdf = std::make_shared<gpmp2::PlanarSDF>(sdf);
            _psdf_factor = std::make_shared<pRSDF>(pRSDF(gtsam::symbol('x', 0), _robot, sdf, 0.0, _eps));
        }

        public:
            /// 2D point robot
            int _ndof = 2;
            int _nlinks = 1;            
            double _eps, _r;

};
}