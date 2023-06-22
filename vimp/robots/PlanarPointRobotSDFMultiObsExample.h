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

#include <gpmp2/kinematics/PointRobotModel.h>
#include <gtsam/inference/Symbol.h>
#include <gpmp2/obstacle/PlanarSDF.h>
#include "../helpers/MatrixIO.h"

using namespace std;
using namespace Eigen;
using namespace vimp;
using namespace gtsam;

namespace vimp{


class PlanarPointRobotSDFMultiObsExample{
    public:
        PlanarPointRobotSDFMultiObsExample(){
            /// map and sdf
            MatrixXd map_ground_truth = _matrix_io.load_csv("/home/hongzhe/git/VIMP/vimp/data/2d_pR/map_multiobs_entropy_map3.csv");
            _field = _matrix_io.load_csv("/home/hongzhe/git/VIMP/vimp/data/2d_pR/field_multiobs_entropy_map3.csv");

            // layout of SDF: Bottom-left is (0,0), length is +/- cell_size per grid.
            Point2 origin(-20, -10);
            double cell_size = 0.1;

            _sdf = gpmp2::PlanarSDF(origin, cell_size, _field);

            /// Robot model
            gpmp2::PointRobot pR(_ndof, _nlinks);
            double r = 1.5;
            gpmp2::BodySphereVector body_spheres;
            body_spheres.push_back(gpmp2::BodySphere(0, r, Point3(0.0, 0.0, 0.0)));
            _pR_model = gpmp2::PointRobotModel(pR, body_spheres);

        }

        public:
            gpmp2::PointRobotModel _pR_model = gpmp2::PointRobotModel();
            gpmp2::PlanarSDF _sdf = gpmp2::PlanarSDF();
            MatrixXd _field;
            MatrixIO _matrix_io = MatrixIO();

            /// 2D point robot
            int _ndof = 2;
            int _nlinks = 1;

        public:
            gpmp2::PointRobotModel pRmodel() const { return _pR_model; }
            gpmp2::PlanarSDF sdf() const { return _sdf; }
            int ndof() const {return _ndof;}
            int nlinks() const {return _nlinks;}
            MatrixXd field() const {return _field;}
        
};
}