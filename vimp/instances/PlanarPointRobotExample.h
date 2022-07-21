/**
 * @file PlanarPointRobotExample.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Create a commonly used planar point robot example class.
 * @version 0.1
 * @date 2022-07-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <vimp/instances/PriorColPlanarPR.h>
#include <gtsam/inference/Symbol.h>
#include <vimp/helpers/data_io.h>

using namespace std;
using namespace gpmp2;
using namespace Eigen;
using namespace vimp;

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

MatrixIO matrix_io{};
string filename_map{"data/2d_pR/map_ground_truth.csv"};
string filename_sdf{"data/2d_pR/map_sdf.csv"};

matrix_io.saveData(filename_map, map_ground_truth);
matrix_io.saveData(filename_sdf, field);
// layout of SDF: Bottom-left is (0,0), length is +/- 1 per point.
Point2 origin(0, 0);
double cell_size = 1.0;

PlanarSDF sdf = PlanarSDF(origin, cell_size, field);
double cost_sigma = 1.0;
double epsilon = 1.0;

/// 2D point robot
int n_total_states = 20, N = n_total_states - 1;

/// parameters
const int ndof = 2, nlinks = 1;
const int dim_conf = ndof * nlinks;
const int dim_theta = 2 * dim_conf; // theta = [conf, vel_conf]
/// dimension of the joint optimization problem
const int ndim = dim_theta * n_total_states;

/// Robot model
PointRobot pR(ndof, nlinks);
double r = 1.0;
BodySphereVector body_spheres;
body_spheres.push_back(BodySphere(0, r, Point3(0.0, 0.0, 0.0)));
PointRobotModel pRModel(pR, body_spheres);

/// start and goal
double start_x = 1.0, start_y = 1.5, goal_x = 5.5, goal_y = 4.5;
VectorXd start_theta(dim_theta);
start_theta << start_x, start_y, 0, 0;
VectorXd goal_theta(dim_theta);
goal_theta << goal_x, goal_y, 0, 0;
