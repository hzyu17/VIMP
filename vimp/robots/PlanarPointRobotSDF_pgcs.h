/**
 * @file PlanarPointRobotSDFMultiObsExample.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief An example experiment settings of a planar point robot in multi obstacle env. 
 * Imported from the tested code in gpmp2.
 * @version 0.1
 * @date 2022-07-27
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <gtsam/inference/Symbol.h>
#include <gpmp2/kinematics/PointRobotModel.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactor.h>
#include "../helpers/data_io.h"

using namespace Eigen;

using pRSDF = gpmp2::ObstaclePlanarSDFFactor<gpmp2::PointRobotModel>;

namespace vimp{

class PlanarPointRobotSDFPGCS{
    public:
        PlanarPointRobotSDFPGCS(double epsilon): _eps(epsilon){
            default_sdf();
            default_pR(*_psdf);
        }

        PlanarPointRobotSDFPGCS(double epsilon, double r): _eps(epsilon), _r(r){
            // default sdf
            default_sdf();

            /// Robot model
            gpmp2::PointRobot pR(_ndof, _nlinks);
            gpmp2::BodySphereVector body_spheres;
            body_spheres.push_back(gpmp2::BodySphere(0, _r, Point3(0.0, 0.0, 0.0)));
            _pR_model = gpmp2::PointRobotModel(pR, body_spheres);

            _p_planar_sdf_factor = std::make_shared<pRSDF>(pRSDF(gtsam::symbol('x', 0), _pR_model, *_psdf, 0.0, _eps));

        }

        void default_sdf(){
            /// map and sdf
            _field = _matrix_io.load_csv("/home/hongzhe/git/VIMP/vimp/data/vimp/2d_pR/field_multiobs_entropy_map2.csv");

            // layout of SDF: Bottom-left is (0,0), length is +/- cell_size per grid.
            Point2 origin(-20, -10);
            double cell_size = 0.1;

            _psdf = std::make_shared<gpmp2::PlanarSDF>(gpmp2::PlanarSDF(origin, cell_size, _field));

        }

        void default_pR(const gpmp2::PlanarSDF& sdf){
            /// Robot model
            gpmp2::PointRobot pR(_ndof, _nlinks);
            gpmp2::BodySphereVector body_spheres;
            body_spheres.push_back(gpmp2::BodySphere(0, 0.0, Point3(0.0, 0.0, 0.0)));
            _pR_model = gpmp2::PointRobotModel(pR, body_spheres);

            _p_planar_sdf_factor = std::make_shared<pRSDF>(pRSDF(gtsam::symbol('x', 0), _pR_model, sdf, 0.0, _eps));
        }

        /**
         * Obstacle factor: planar case, returns the Vector of h(x) and the Jacobian matrix.
         * */
        std::tuple<VectorXd, MatrixXd> hinge_jac(const VectorXd& pose){
            MatrixXd Jacobian;
            VectorXd vec_err = _p_planar_sdf_factor->evaluateError(pose, Jacobian);

            return std::make_tuple(vec_err, Jacobian);
        }

        inline void update_sdf(const gpmp2::PlanarSDF& sdf){
            _psdf = std::make_shared<gpmp2::PlanarSDF>(sdf);
            _p_planar_sdf_factor = std::make_shared<pRSDF>(pRSDF(gtsam::symbol('x', 0), _pR_model, sdf, 0.0, _eps));
        }

        inline gpmp2::PointRobotModel pRmodel() const { return _pR_model; }
        inline std::shared_ptr<gpmp2::PlanarSDF> sdf() const { return _psdf; }
        inline int ndof() const {return _ndof;}
        inline int nlinks() const {return _nlinks;}
        inline MatrixXd field() const {return _field;}

        public:
            gpmp2::PointRobotModel _pR_model;
            std::shared_ptr<gpmp2::PlanarSDF> _psdf;
            MatrixXd _field;
            MatrixIO _matrix_io;

            std::shared_ptr<pRSDF> _p_planar_sdf_factor;

            /// 2D point robot
            int _ndof = 2;
            int _nlinks = 1;            

            double _eps, _r;

};
}