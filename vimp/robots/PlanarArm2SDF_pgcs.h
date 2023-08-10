/**
 * @file PlanarArm2SDF_pgcs.h
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

namespace vimp{

using ArmSDF = gpmp2::ObstaclePlanarSDFFactor<gpmp2::ArmModel>;
using BaseClass = RobotSDFBase<gpmp2::ArmModel, gpmp2::PlanarSDF, ArmSDF>;

class PlanarArm2SDFExample: public BaseClass{
    public:

        PlanarArm2SDFExample(){}

        PlanarArm2SDFExample(double epsilon, double radius,
                            const std::string& map_name="2darm_map1", 
                            const std::string& sdf_file=""):
        BaseClass(2, 1, 2, map_name), 
        _eps(epsilon), 
        _r(radius)
        {   
            MatrixXd field = _m_io.load_csv(BaseClass::_field_file);  
            BaseClass::_sdf = gpmp2::PlanarSDF(BaseClass::_origin, BaseClass::_cell_size, field);
            BaseClass::_psdf = std::make_shared<gpmp2::PlanarSDF>(BaseClass::_sdf);

            generate_arm_sdf(BaseClass::_sdf, radius);
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

                _psdf_factor = std::make_shared<ArmSDF>(ArmSDF(gtsam::symbol('x', 0), BaseClass::_robot, sdf, 0.0, _eps));
        }

        inline void update_sdf(const gpmp2::PlanarSDF& sdf){
            _psdf = std::make_shared<gpmp2::PlanarSDF>(sdf);
            _psdf_factor = std::make_shared<ArmSDF>(ArmSDF(gtsam::symbol('x', 0), BaseClass::_robot, sdf, 0.0, _eps));
        }


        protected:       
            double _eps, _r;
            double _cell_size;
            Eigen::Vector2d _origin;
};

}