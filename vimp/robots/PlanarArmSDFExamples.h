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
using Base = RobotSDFBase<gpmp2::ArmModel, gpmp2::PlanarSDF, ArmSDF>;
class PlanarArmSDFExample: public Base{
    public:
        PlanarArmSDFExample(double epsilon, double radius,
                            const std::string& field_file="", const std::string& sdf_file="", 
                            double cell_size=0.01, double ori_x=-1.0, double ori_y=-1.0):
        Base(2, 1, "/home/hyu419/git/VIMP/vimp/data/pgcs/2d_Arm/field_two_obs.csv", ""), 
        _eps(epsilon), 
        _r(radius),
        _cell_size(cell_size)
        {
            Eigen::Vector2d origin(ori_x, ori_y);
            _origin = origin;
            if (!field_file.empty()){
                Base::update_field_file(field_file);
            }
            default_sdf();
            generate_arm_sdf(*(Base::_psdf), radius);
        }

        virtual void default_sdf(){
            /// map and sdf
            MatrixXd field{_m_io.load_csv(Base::_field_file)};

            // layout of SDF: Bottom-left is (0,0), length is +/- cell_size per grid.
            // Point2 origin(-20, -10);
            // double cell_size = 0.1;

            // Point2 origin(-1.0, -1.0);
            // double cell_size = 0.01;

            _psdf = std::make_shared<gpmp2::PlanarSDF>(gpmp2::PlanarSDF(_origin, _cell_size, field));

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


        protected:       
            double _eps, _r;
            double _cell_size;
            Eigen::Vector2d _origin;
};

// class PlanarArmSDFMap1: public PlanarArmSDFExample{

// public: 
// PlanarArmSDFMap1(double epsilon, double radius, 
//                  const std::string& field_file="/home/hyu419/git/VIMP/vimp/data/vimp/2d_Arm/field_one_obs.csv", 
//                  const std::string& sdf_file=""):
//                  PlanarArmSDFExample(epsilon, radius, 0.01, Eigen::Vector2d::Zero(), field_file){
//                     Eigen::Vector2d new_origin(-1.0, -1.0);
//                     _origin = new_origin;
//                     default_sdf();
//                  }

//         void default_sdf() override{
//             /// map and sdf
//             MatrixXd field{_m_io.load_csv(Base::_field_file)};

//             // layout of SDF: Bottom-left is (0,0), length is +/- cell_size per grid.
//             // Point2 origin(-20, -10);
//             // double cell_size = 0.1;

//             // Point2 origin(-1.0, -1.0);
//             // double cell_size = 0.01;

//             _psdf = std::make_shared<gpmp2::PlanarSDF>(gpmp2::PlanarSDF(_origin, _cell_size, field));

//         }
// private:
//     Eigen::Vector2d _origin;
// };
}