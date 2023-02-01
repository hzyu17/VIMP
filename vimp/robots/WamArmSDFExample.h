/**
 * @file WamArmSDFExample.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The Wam robot with SDF example
 * @version 0.1
 * @date 2023-01-19
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/obstacle/SignedDistanceField.h>

using namespace gpmp2;

namespace vimp{

    class WamArmSDFExample{
        public:
            WamArmSDFExample(const string& sdf_file){

                _sdf = SignedDistanceField();
                _sdf.loadSDF(sdf_file);

                // WAM arm exmaple, only joint position, no rotation
                gtsam::Vector7 a = (gtsam::Vector7() << 0.0, 0.0, 0.045, -0.045, 0.0, 0.0, 0.0).finished();
                gtsam::Vector7 alpha = (gtsam::Vector7() << -M_PI/2.0, M_PI/2.0, -M_PI/2.0, M_PI/2.0, -M_PI/2.0, M_PI/2.0, 0.0).finished();
                gtsam::Vector7 d = (gtsam::Vector7() << 0.0, 0.0, 0.55, 0.0, 0.3, 0.0, 0.06).finished();
                Arm arm(7, a, alpha, d);
            
                // body spheres    
                BodySphereVector body_spheres;
                body_spheres.push_back(BodySphere(0, 0.15, gtsam::Point3(0.0,  0.0,  0.0)));

                body_spheres.push_back(BodySphere(1, 0.06, gtsam::Point3(0.0,  0.0,  0.2)));
                body_spheres.push_back(BodySphere(1, 0.06, gtsam::Point3(0.0,  0.0,  0.3)));
                body_spheres.push_back(BodySphere(1, 0.06, gtsam::Point3(0.0,  0.0,  0.4)));
                body_spheres.push_back(BodySphere(1, 0.06, gtsam::Point3(0.0,  0.0,  0.5)));

                body_spheres.push_back(BodySphere(2, 0.06, gtsam::Point3(0.0,  0.0,  0.0)));

                body_spheres.push_back(BodySphere(3, 0.06, gtsam::Point3(0.0,  0.0,  0.1)));
                body_spheres.push_back(BodySphere(3, 0.06, gtsam::Point3(0.0,  0.0,  0.2)));
                body_spheres.push_back(BodySphere(3, 0.06, gtsam::Point3(0.0,  0.0,  0.3)));

                body_spheres.push_back(BodySphere(5, 0.06, gtsam::Point3(0.0,  0.0,  0.1)));

                body_spheres.push_back(BodySphere(6, 0.04, gtsam::Point3(0.1, -0.025,  0.08)));
                body_spheres.push_back(BodySphere(6, 0.04, gtsam::Point3(0.1,  0.025,  0.08)));
                body_spheres.push_back(BodySphere(6, 0.04, gtsam::Point3(-0.1,  0.0,  0.08)));
                body_spheres.push_back(BodySphere(6, 0.04, gtsam::Point3(0.15, -0.025,  0.13)));
                body_spheres.push_back(BodySphere(6, 0.04, gtsam::Point3(0.15,  0.025,  0.13)));
                body_spheres.push_back(BodySphere(6, 0.04, gtsam::Point3(-0.15,  0.0,  0.13)));

                _arm = ArmModel{arm, body_spheres};

            }

            ArmModel arm_model(){
                return _arm;
            }

            SignedDistanceField sdf() const { return _sdf; }
            int ndof() const {return _ndof; }
            int nlinks() const {return _nlinks; }

        private:
            ArmModel _arm;
            SignedDistanceField _sdf;

            /// Arm robot
            int _ndof = 1;
            int _nlinks = 7;
    };

}// namespace