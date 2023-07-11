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

#include <gtsam/inference/Symbol.h>
#include <gpmp2/kinematics/PointRobotModel.h>
#include <gpmp2/obstacle/ObstacleSDFFactor.h>
#include "RobotSDFBase.h"

using namespace Eigen;

using PRModel = gpmp2::PointRobotModel;
using SDF = gpmp2::SignedDistanceField;
using pRSDF = gpmp2::ObstacleSDFFactor<PRModel>;

namespace vimp{

class PointRobot3DSDFExample:public RobotSDFBase<PRModel, SDF, pRSDF>{
    public:
        virtual ~PointRobot3DSDFExample(){}
        
        PointRobot3DSDFExample(double epsilon, double radius):RobotSDFBase<PRModel, SDF, pRSDF>(3, 1),
                                                              _eps(epsilon), 
                                                              _r(radius)
        {
            default_sdf();
            generate_pr_sdf(*_psdf, radius);
        }

        virtual void default_sdf(){
            SDF sdf = SDF();
            sdf.loadSDF("/home/hyu419/git/VIMP/matlab_helpers/PGCS-examples/3dSDFs/pRSDF3D.bin");
            _psdf = std::make_shared<SDF>(sdf);
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