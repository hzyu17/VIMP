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

#include "RobotSDFBase.h"
#include <gtsam/inference/Symbol.h>
#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/obstacle/SignedDistanceField.h>
#include <gpmp2/obstacle/ObstacleSDFFactorArm.h>

using namespace gpmp2;
using ArmModel = gpmp2::ArmModel;
using ObsArmSDF = gpmp2::ObstacleSDFFactorArm;
using SDF = gpmp2::SignedDistanceField;
using sym = gtsam::Symbol;

namespace vimp{

    class WamArmSDFExample:public RobotSDFBase<ArmModel, SDF, ObsArmSDF>{
        public:
            WamArmSDFExample(const string& sdf_file, double eps): 
                RobotSDFBase<ArmModel, SDF, ObsArmSDF>(),
                _eps(eps)
            {
                this->_sdf.loadSDF(sdf_file);
                this->_psdf = std::make_shared<SDF>(this->_sdf);
                generateArm();
                this->_psdf_factor = std::make_shared<ObsArmSDF>(ObsArmSDF(sym('x', 0), this->_robot, this->_sdf, 0.0, _eps));
            }

            WamArmSDFExample(double eps, double radius): _eps(eps), _radius(radius){
                default_sdf();
                generateArm();
                this->_psdf_factor = std::make_shared<ObsArmSDF>(ObsArmSDF(sym('x', 0), this->_robot, this->_sdf, 0.0, _eps));
            }

            inline void update_sdf(const SDF& sdf){
                this->_psdf = std::make_shared<SDF>(sdf);
                this->_psdf_factor = std::make_shared<ObsArmSDF>(ObsArmSDF(gtsam::symbol('x', 0), this->_robot, sdf, 0.0, _eps));
            }

            void generateArm(){
                // WAM arm exmaple, only joint position, no rotation
                gtsam::Vector7 a = (gtsam::Vector7() << 0.0, 0.0, 0.045, -0.045, 0.0, 0.0, 0.0).finished();
                gtsam::Vector7 alpha = (gtsam::Vector7() << -M_PI/2.0, M_PI/2.0, -M_PI/2.0, M_PI/2.0, -M_PI/2.0, M_PI/2.0, 0.0).finished();
                gtsam::Vector7 d = (gtsam::Vector7() << 0.0, 0.0, 0.55, 0.0, 0.3, 0.0, 0.06).finished();
                Arm arm(7, a, alpha, d);
            
                // body spheres    
                BodySphereVector body_spheres;
                // body_spheres.push_back(BodySphere(0, 0.15, gtsam::Point3(0.0,  0.0,  0.0)));

                // body_spheres.push_back(BodySphere(1, 0.06, gtsam::Point3(0.0,  0.0,  0.2)));
                // body_spheres.push_back(BodySphere(1, 0.06, gtsam::Point3(0.0,  0.0,  0.3)));
                // body_spheres.push_back(BodySphere(1, 0.06, gtsam::Point3(0.0,  0.0,  0.4)));
                // body_spheres.push_back(BodySphere(1, 0.06, gtsam::Point3(0.0,  0.0,  0.5)));

                // body_spheres.push_back(BodySphere(2, 0.06, gtsam::Point3(0.0,  0.0,  0.0)));

                // body_spheres.push_back(BodySphere(3, 0.06, gtsam::Point3(0.0,  0.0,  0.1)));
                // body_spheres.push_back(BodySphere(3, 0.06, gtsam::Point3(0.0,  0.0,  0.2)));
                // body_spheres.push_back(BodySphere(3, 0.06, gtsam::Point3(0.0,  0.0,  0.3)));

                // body_spheres.push_back(BodySphere(5, 0.06, gtsam::Point3(0.0,  0.0,  0.1)));

                // body_spheres.push_back(BodySphere(6, 0.04, gtsam::Point3(0.1, -0.025,  0.08)));
                // body_spheres.push_back(BodySphere(6, 0.04, gtsam::Point3(0.1,  0.025,  0.08)));
                // body_spheres.push_back(BodySphere(6, 0.04, gtsam::Point3(-0.1,  0.0,  0.08)));
                // body_spheres.push_back(BodySphere(6, 0.04, gtsam::Point3(0.15, -0.025,  0.13)));
                // body_spheres.push_back(BodySphere(6, 0.04, gtsam::Point3(0.15,  0.025,  0.13)));
                // body_spheres.push_back(BodySphere(6, 0.04, gtsam::Point3(-0.15,  0.0,  0.13)));

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

                this->_robot = gpmp2::ArmModel{arm, body_spheres};
            }

            /**
             * Obstacle factor: planar case, returns the Vector of h(x) and the Jacobian matrix.
             * */
            std::tuple<Eigen::VectorXd, Eigen::MatrixXd> hinge_jac(const Eigen::VectorXd& pose){
                Eigen::MatrixXd Jacobian;
                Eigen::VectorXd vec_err = this->_psdf_factor->evaluateError(pose, Jacobian);

                return std::make_tuple(vec_err, Jacobian);
            }

            void default_sdf(){
                this->_sdf = SDF();
                this->_sdf.loadSDF("/home/hongzhe/git/VIMP/matlab_helpers/PGCS-examples/3dSDFs/WAMDeskDataset.bin");
                this->_psdf = std::make_shared<SDF>(this->_sdf);
            }

            
            // SDF sdf() const { return this->_sdf; }
            int ndof() const {return _ndof; }
            int nlinks() const {return _nlinks; }

        public:
           
            /// Arm robot
            int _ndof = 1;
            int _nlinks = 7;
            double _eps, _radius;
    };

}// namespace