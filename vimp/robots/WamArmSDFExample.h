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

#define STRING(x) #x
#define XSTRING(x) STRING(x)

using namespace gpmp2;
using ArmModel = gpmp2::ArmModel;
using ObsArmSDF = gpmp2::ObstacleSDFFactorArm;
using SDF = gpmp2::SignedDistanceField;
using sym = gtsam::Symbol;

namespace vimp{
using Base = RobotSDFBase<ArmModel, SDF, ObsArmSDF>;
    class WamArmSDFExample:public Base{
        public:
            WamArmSDFExample(){}
            
            WamArmSDFExample(const std::string& sdf_file, double eps): 
                Base(1, 7),
                _eps(eps)
            {
                Base::_sdf.loadSDF(sdf_file);
                Base::_psdf = std::make_shared<SDF>(Base::_sdf);
                generateArm();
                Base::_psdf_factor = std::make_shared<ObsArmSDF>(ObsArmSDF(sym('x', 0), Base::_robot, Base::_sdf, 0.0, _eps));
            }

            WamArmSDFExample(double eps, double radius): Base(1, 7), _eps(eps), _radius(radius){
                default_sdf();
                generateArm();
                Base::_psdf_factor = std::make_shared<ObsArmSDF>(ObsArmSDF(sym('x', 0), Base::_robot, Base::_sdf, 0.0, _eps));
            }

            WamArmSDFExample(double eps, double radius, const std::string & map_name, const std::string & sdf_file): 
                Base(1, 7, 3, map_name), 
                _eps(eps), 
                _radius(radius)
            {
                std::cout << "sdf file: " << sdf_file << std::endl;
                std::cout << "map name: " << map_name << std::endl;
                if (!sdf_file.empty()){
                    std::cout << "Load SDF file: " << sdf_file << std::endl;
                    Base::_sdf.loadSDF(sdf_file);
                    Base::_psdf = std::make_shared<SDF>(Base::_sdf);
                }
                else{
                    std::runtime_error("Empty sdf map file!");
                }

                generateArm();
                // default_sdf();
                Base::_psdf_factor = std::make_shared<ObsArmSDF>(ObsArmSDF(sym('x', 0), Base::_robot, Base::_sdf, 0.0, _eps));
            }

            inline void update_sdf(const SDF& sdf){
                Base::_psdf = std::make_shared<SDF>(sdf);
                Base::_psdf_factor = std::make_shared<ObsArmSDF>(ObsArmSDF(gtsam::symbol('x', 0), Base::_robot, sdf, 0.0, _eps));
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

                Base::_robot = gpmp2::ArmModel{arm, body_spheres};
            }

            void default_sdf(){
                Base::_sdf = SDF();
                std::string source_root{XSTRING(SOURCE_ROOT)};
                std::string sdf_file{source_root+"/maps/WAM/WAMDeskDataset.bin"};
                Base::_sdf.loadSDF(sdf_file);
                Base::_psdf = std::make_shared<SDF>(Base::_sdf);
            }
            
            // SDF sdf() const { return Base::_sdf; }
            int ndof() const {return _ndof; }
            int nlinks() const {return _nlinks; }

        public:
           
            /// Arm robot
            double _eps, _radius;
    };

}// namespace