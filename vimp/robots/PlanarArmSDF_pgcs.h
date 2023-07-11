/**
 * @file PlanarArmSDF_pgcs.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief An example experiment settings of a planar arm in multi obstacle env. 
 * Imported from the tested code in gpmp2.
 * @version 0.1
 * @date 2022-07-27
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <gtsam/inference/Symbol.h>
#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactor.h>
#include "RobotSDFBase.h"

using namespace Eigen;
using namespace gpmp2;

using ArmSDF = gpmp2::ObstaclePlanarSDFFactor<gpmp2::ArmModel>;

namespace vimp{

class PlanarArmSDFExample: public RobotSDFBase<gpmp2::ArmModel, gpmp2::PlanarSDF, ArmSDF>{
    public:
        PlanarArmSDFExample(double epsilon):_eps(epsilon){
            default_sdf();
            generate_arm_sdf(*_psdf, 0.01);
        }

        PlanarArmSDFExample(double epsilon, double r): _eps(epsilon), _r(r){
            // default sdf
            default_sdf();
            generate_arm_sdf(*_psdf, r);
        }

        void default_sdf(){
            /// map and sdf
            MatrixXd field{_m_io.load_csv("/home/hyu419/git/VIMP/vimp/data/pgcs/2d_Arm/field_two_obs.csv")};

            // layout of SDF: Bottom-left is (0,0), length is +/- cell_size per grid.
            Point2 origin(-20, -10);
            double cell_size = 0.1;

            _psdf = std::make_shared<gpmp2::PlanarSDF>(gpmp2::PlanarSDF(origin, cell_size, field));

        }

        void generate_arm_sdf(const gpmp2::PlanarSDF& sdf, double r){
            // 2 link simple example
                gtsam::Pose3 arm_base(Rot3(), Point3(0.0, 0.0, 0.0));
                Vector2d a(0.5, 0.5), alpha(0, 0), d(0, 0);
                Arm abs_arm(2, a, alpha, d, arm_base);

                // body info, three spheres
                BodySphereVector body_spheres;

                body_spheres.push_back(BodySphere(0, r, Point3(-0.5,  0.0,  0.0)));
                body_spheres.push_back(BodySphere(0, r, Point3(-0.4,  0.0,  0.0)));
                body_spheres.push_back(BodySphere(0, r, Point3(-0.3,  0.0,  0.0)));
                body_spheres.push_back(BodySphere(0, r, Point3(-0.2,  0.0,  0.0)));
                body_spheres.push_back(BodySphere(0, r, Point3(-0.2,  0.0,  0.0)));
                body_spheres.push_back(BodySphere(0, r, Point3(-0.1,  0.0,  0.0)));

                body_spheres.push_back(BodySphere(1, r, Point3(-0.5, 0.0,  0.0)));
                body_spheres.push_back(BodySphere(1, r, Point3(-0.4, 0.0,  0.0)));
                body_spheres.push_back(BodySphere(1, r, Point3(-0.3, 0.0,  0.0)));
                body_spheres.push_back(BodySphere(1, r, Point3(-0.2, 0.0,  0.0)));
                body_spheres.push_back(BodySphere(1, r, Point3(-0.1, 0.0,  0.0)));
                body_spheres.push_back(BodySphere(1, r, Point3( 0.0, 0.0,  0.0)));

                _robot = gpmp2::ArmModel{abs_arm, body_spheres};

            _psdf_factor = std::make_shared<ArmSDF>(ArmSDF(gtsam::symbol('x', 0), _robot, sdf, 0.0, _eps));
        }

        inline void update_sdf(const gpmp2::PlanarSDF& sdf){
            _psdf = std::make_shared<gpmp2::PlanarSDF>(sdf);
            _psdf_factor = std::make_shared<ArmSDF>(ArmSDF(gtsam::symbol('x', 0), _robot, sdf, 0.0, _eps));
        }


        public:       
            double _eps, _r;
            int _ndof = 2, _nlinks = 1;

};
}