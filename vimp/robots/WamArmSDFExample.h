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

#include <gtsam/inference/Symbol.h>
#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/obstacle/SignedDistanceField.h>
#include <gpmp2/obstacle/ObstacleSDFFactorArm.h>

using namespace gpmp2;
using ObsArmSDF = gpmp2::ObstacleSDFFactorArm;
using SDF = gpmp2::SignedDistanceField;
using sym = gtsam::Symbol;

namespace vimp{

    class WamArmSDFExample{
        public:
            WamArmSDFExample(const string& sdf_file, double eps): _eps(eps){
                _sdf = SDF();
                _sdf.loadSDF(sdf_file);
                generateArm();
                _parm_sdf = std::make_shared<ObsArmSDF>(ObsArmSDF(sym('x', 0), _arm, _sdf, 0.0, _eps));
            }

            WamArmSDFExample(double eps): _eps(eps){
                default_sdf();
                generateArm();
                _parm_sdf = std::make_shared<ObsArmSDF>(ObsArmSDF(sym('x', 0), _arm, _sdf, 0.0, _eps));
            }

            void update_arm_sdf(){
                _parm_sdf = std::make_shared<ObsArmSDF>(ObsArmSDF(sym('x', 0), _arm, _sdf, 0.0, _eps));
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

                double r = 0.0;
                body_spheres.push_back(BodySphere(0, 0.15, gtsam::Point3(0.0,  0.0,  0.0)));

                body_spheres.push_back(BodySphere(1, 0.06, gtsam::Point3(0.0,  0.0,  0.2)));
                body_spheres.push_back(BodySphere(1, 0.06, gtsam::Point3(0.0,  0.0,  0.3)));
                body_spheres.push_back(BodySphere(1, 0.06, gtsam::Point3(0.0,  0.0,  0.4)));
                body_spheres.push_back(BodySphere(1, 0.06, gtsam::Point3(0.0,  0.0,  0.5)));

                body_spheres.push_back(BodySphere(2, r, gtsam::Point3(0.0,  0.0,  0.0)));

                body_spheres.push_back(BodySphere(3, r, gtsam::Point3(0.0,  0.0,  0.1)));
                body_spheres.push_back(BodySphere(3, r, gtsam::Point3(0.0,  0.0,  0.2)));
                body_spheres.push_back(BodySphere(3, r, gtsam::Point3(0.0,  0.0,  0.3)));

                body_spheres.push_back(BodySphere(5, r, gtsam::Point3(0.0,  0.0,  0.1)));

                body_spheres.push_back(BodySphere(6, r, gtsam::Point3(0.1, -0.025,  0.08)));
                body_spheres.push_back(BodySphere(6, r, gtsam::Point3(0.1,  0.025,  0.08)));
                body_spheres.push_back(BodySphere(6, r, gtsam::Point3(-0.1,  0.0,  0.08)));
                body_spheres.push_back(BodySphere(6, r, gtsam::Point3(0.15, -0.025,  0.13)));
                body_spheres.push_back(BodySphere(6, r, gtsam::Point3(0.15,  0.025,  0.13)));
                body_spheres.push_back(BodySphere(6, r, gtsam::Point3(-0.15,  0.0,  0.13)));

                _arm = gpmp2::ArmModel{arm, body_spheres};
            }

            /**
             * Obstacle factor: planar case, returns the Vector of h(x) and the Jacobian matrix.
             * */
            std::tuple<Eigen::VectorXd, Eigen::MatrixXd> hinge_jac(const Eigen::VectorXd& pose){
                Eigen::MatrixXd Jacobian;
                Eigen::VectorXd vec_err = _parm_sdf->evaluateError(pose, Jacobian);

                return std::make_tuple(vec_err, Jacobian);
            }

            void default_sdf(){
                _sdf = SDF();
                _sdf.loadSDF("/home/hongzhe/git/VIMP/matlab_helpers/PGCS-examples/3dSDFs/WAMDeskDataset.bin");
            }

            void update_sdf(const SDF& sdf){ 
                _sdf = sdf; 
                update_arm_sdf();
            }
            
            SDF sdf() const { return _sdf; }
            int ndof() const {return _ndof; }
            int nlinks() const {return _nlinks; }
            gpmp2::ArmModel arm_model(){ return _arm; }

        private:
            gpmp2::ArmModel _arm;
            SDF _sdf;
            
            /// Arm robot
            int _ndof = 1;
            int _nlinks = 7;

            // obstacle factor
            std::shared_ptr<ObsArmSDF> _parm_sdf;
            double _eps;
    };

}// namespace