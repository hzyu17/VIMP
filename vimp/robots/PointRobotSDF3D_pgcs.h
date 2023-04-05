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
#include <gpmp2/obstacle/SignedDistanceField.h>
#include "../helpers/data_io.h"

using namespace Eigen;

using SDF = gpmp2::SignedDistanceField;
using pRSDF3D = gpmp2::ObstacleSDFFactor<gpmp2::PointRobotModel>;

namespace vimp{

class PointRobotSDFPGCS{
    public:
        PointRobotSDFPGCS(double epsilon): _eps(epsilon){
            default_sdf();
            generate_pr_sdf(*_psdf, 0.0);
        }

        PointRobotSDFPGCS(double epsilon, double r): _eps(epsilon), _r(r){
            // default sdf
            default_sdf();

            // point robot with sdf
            generate_pr_sdf(*_psdf, r);
        }

        void default_sdf(){
            SDF sdf = SDF();
            sdf.loadSDF("/home/hongzhe/git/VIMP/matlab_helpers/PGCS-examples/3d_pR/pRSDF3D.bin");
            _psdf = std::make_shared<SDF>(sdf);
        }

        void generate_pr_sdf(const SDF& sdf, double r){
            /// Robot model
            gpmp2::PointRobot pR(_ndof, _nlinks);
            gpmp2::BodySphereVector body_spheres;
            body_spheres.push_back(gpmp2::BodySphere(0, r, Point3(0.0, 0.0, 0.0)));
            _pR_model = gpmp2::PointRobotModel(pR, body_spheres);

            _p_planar_sdf_factor = std::make_shared<pRSDF3D>(pRSDF3D(gtsam::symbol('x', 0), _pR_model, sdf, 0.0, _eps));
        }

        /**
         * Obstacle factor: planar case, returns the Vector of h(x) and the Jacobian matrix.
         * */
        std::tuple<VectorXd, MatrixXd> hinge_jac(const VectorXd& pose){
            MatrixXd Jacobian;
            VectorXd vec_err = _p_planar_sdf_factor->evaluateError(pose, Jacobian);

            return std::make_tuple(vec_err, Jacobian);
        }

        inline void update_sdf(const SDF& sdf){
            _psdf = std::make_shared<SDF>(sdf);        
        }

        inline gpmp2::PointRobotModel pRmodel() const { return _pR_model; }
        inline std::shared_ptr<SDF> sdf() const { return _psdf; }
        inline int ndof() const { return _ndof; }
        inline int nlinks() const { return _nlinks; }

        public:
            gpmp2::PointRobotModel _pR_model;
            std::shared_ptr<SDF> _psdf;

            std::shared_ptr<pRSDF3D> _p_planar_sdf_factor;

            /// 3D point robot
            int _ndof = 3;
            int _nlinks = 1;            

            double _eps, _r;

    };
} // namespace vimp