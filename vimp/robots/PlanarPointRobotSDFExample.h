/**
 * @file PlanarPointRobotSDFExample.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Create a commonly used planar point robot example class.
 * @version 0.1
 * @date 2022-07-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <gpmp2/kinematics/PointRobotModel.h>
#include <gtsam/inference/Symbol.h>
#include <gpmp2/obstacle/PlanarSDF.h>
#include "helpers/MatrixHelper.h"

using namespace std;
using namespace gpmp2;
using namespace Eigen;
using namespace gtsam;

namespace vimp{


class PlanarPointRobotSDFExample{
    public:
        PlanarPointRobotSDFExample(){
        /// map and sdf
        MatrixXd map_ground_truth = (MatrixXd(7, 7) <<
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 1, 1, 1, 0, 0,
                0, 0, 1, 1, 1, 0, 0,
                0, 0, 1, 1, 1, 0, 0,
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0).finished();
        MatrixXd field = (MatrixXd(7, 7) <<
                2.8284, 2.2361, 2.0000, 2.0000, 2.0000, 2.2361, 2.8284,
                2.2361, 1.4142, 1.0000, 1.0000, 1.0000, 1.4142, 2.2361,
                2.0000, 1.0000, -1.0000, -1.0000, -1.0000, 1.0000, 2.0000,
                2.0000, 1.0000, -1.0000, -2.0000, -1.0000, 1.0000, 2.0000,
                2.0000, 1.0000, -1.0000, -1.0000, -1.0000, 1.0000, 2.0000,
                2.2361, 1.4142, 1.0000, 1.0000, 1.0000, 1.4142, 2.2361,
                2.8284, 2.2361, 2.0000, 2.0000, 2.0000, 2.2361, 2.8284).finished();

        string filename_map{"/home/hzyu/git/VIMP/vimp/data/2d_pR/map_ground_truth.csv"};
        string filename_sdf{"/home/hzyu/git/VIMP/vimp/data/2d_pR/map_sdf.csv"};

        _matrix_io.saveData<MatrixXd>(filename_map, map_ground_truth);
        _matrix_io.saveData<MatrixXd>(filename_sdf, field);

        // layout of SDF: Bottom-left is (0,0), length is +/- 1 per point.
        Point2 origin(0, 0);
        double cell_size = 1.0;

        _sdf = PlanarSDF(origin, cell_size, field);

        /// Robot model
        PointRobot pR(_ndof, _nlinks);
        double r = 1.0;
        BodySphereVector body_spheres;
        body_spheres.push_back(BodySphere(0, r, Point3(0.0, 0.0, 0.0)));
        _pR_model = PointRobotModel(pR, body_spheres);

        }

        public:
            PointRobotModel _pR_model = PointRobotModel();
            PlanarSDF _sdf = PlanarSDF();
            gvi::MatrixIO _matrix_io = gvi::MatrixIO();

            /// 2D point robot
            int _ndof = 2;
            int _nlinks = 1;

        public:
            PointRobotModel pRmodel() const { return _pR_model; }
            PlanarSDF sdf() const { return _sdf; }
            int ndof() const {return _ndof;}
            int nlinks() const {return _nlinks;}
        
};
}