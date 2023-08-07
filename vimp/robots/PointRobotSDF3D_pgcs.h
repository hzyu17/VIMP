/**
 * @file PointRobotSDF3D_pgcs.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief An example experiment settings of a point robot in 3D multi obstacle env. 
 * Imported from the tested code in gpmp2.
 * @version 0.1
 * @date 2022-07-27
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#define STRING(x) #x
#define XSTRING(x) STRING(x)

std::string source_root{XSTRING(SOURCE_ROOT)};

#include <gtsam/inference/Symbol.h>
#include <gpmp2/kinematics/PointRobotModel.h>
#include <gpmp2/obstacle/ObstacleSDFFactor.h>
#include "RobotSDFBase.h"

using namespace Eigen;

using PRModel = gpmp2::PointRobotModel;
using SDF = gpmp2::SignedDistanceField;
using pRSDF = gpmp2::ObstacleSDFFactor<PRModel>;

namespace vimp{
using Base = RobotSDFBase<PRModel, SDF, pRSDF>;
class PointRobot3DSDFExample:public RobotSDFBase<PRModel, SDF, pRSDF>{
    public:
        virtual ~PointRobot3DSDFExample(){}
        
        PointRobot3DSDFExample(double epsilon, 
                               double radius, 
                               const std::string& map_name, 
                               const std::string& sdf_file=source_root+"/../matlab_helpers/PGCS-examples/3dSDFs/pRSDF3D.bin"):
        Base(3, 1, 3, ""),
        _eps(epsilon), 
        _r(radius)
        {
            if (!sdf_file.empty()){
                Base::_sdf.loadSDF(sdf_file);
                Base::_psdf = std::make_shared<SDF>(Base::_sdf);
            }
            else{
                std::runtime_error("Empty sdf map file!");
            }
            generate_pr_sdf(Base::_sdf, radius);
        }

        void generate_pr_sdf(const SDF& sdf, double r){
            /// Robot model
            gpmp2::PointRobot pR(_ndof, _nlinks);
            gpmp2::BodySphereVector body_spheres;
            body_spheres.push_back(gpmp2::BodySphere(0, r, Point3(0.0, 0.0, 0.0)));
            _robot = PRModel(pR, body_spheres);

            _psdf_factor = std::make_shared<pRSDF>(pRSDF(gtsam::symbol('x', 0), _robot, sdf, 0.0, _eps));
        }

        inline void update_sdf(const SDF& sdf){
            _psdf = std::make_shared<SDF>(sdf);
            _psdf_factor = std::make_shared<pRSDF>(pRSDF(gtsam::symbol('x', 0), _robot, sdf, 0.0, _eps));
        }

        public:
            /// 3D point robot
            int _ndof = 3;
            int _nlinks = 1;            
            double _eps, _r;

    };
} // namespace vimp