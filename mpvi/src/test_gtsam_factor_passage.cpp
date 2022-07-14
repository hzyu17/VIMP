/**
 * @file test_gtsam_factor_passage.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief 
 * @version 0.1
 * @date 2022-07-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */


/// Test the passage of factors defined in gtsam in the classes defined in VIMP optimizers.

#include "../include/OptimizerPriorPlanarPointRobot.h"
#include <gtsam/inference/Symbol.h>
// #include "../include/matplotlibcpp.h"
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h>


int main(){

    /// 2D point robot
    // double total_time = 10.0;
    int num_support_states = 1, N = num_support_states + 1;
    // int num_interp = 2;

    /// parameters
    const int ndof = 2, nlinks = 1;
    const int dim_conf = ndof * nlinks;
    const int dim_theta = 2 * dim_conf;
    const int ndim = dim_theta * (N+1);

    /// Robot model
    PointRobot pR(ndof, nlinks);
    double r = 0.5;
    BodySphereVector body_spheres;
    body_spheres.push_back(BodySphere(0, r, Point3(0.0, 0.0, 0.0)));
    PointRobotModel pRModel(pR, body_spheres);

    // initialize the mean by linear interpolation
    double start_x = 1.0, start_y = 0.0, goal_x = 5.5, goal_y = 0.0;
    VectorXd start_conf(dim_theta);
    start_conf << start_x, start_y, 0, 0;
    VectorXd goal_conf(dim_theta);
    goal_conf << goal_x, goal_y, 0, 0;
    
    /// Noise models
    SharedNoiseModel Qc_model = noiseModel::Isotropic::Sigma(dim_conf, 1.0);
    cout << "Qc" << endl << getQc(Qc_model) << endl;
    SharedNoiseModel K_0 = noiseModel::Isotropic::Sigma(dim_conf, 1.0);

    /// Vector of Pks
    vector<MatrixXd> vec_Pks;

    /// Vector of factored optimizers
    vector<OptimizerFactorPriorPRGH> vec_factorized_opts;

    for (int i = 0; i < N+1; i++) {
        VectorXd conf{start_conf + double(i) * (goal_conf - start_conf) / num_support_states};

        cout << "conf" << endl << conf << endl;
        /// Pk matrices
        MatrixXd Pk{MatrixXd::Zero(dim_theta, ndim)};
        Pk.block<dim_theta, dim_theta>(0, i * dim_theta) = MatrixXd::Identity(dim_theta, dim_theta);
        /// Pk.block<dim_conf, dim_conf>(dim_conf, (i + 1) * dim_conf) = MatrixXd::Identity(dim_conf, dim_conf);
        
        cout << "Pk" << endl << Pk << endl;
        vec_Pks.emplace_back(Pk);

        cout << "conf.segment<dim_conf>(0)" << endl << gtsam::Vector2{conf.segment<dim_conf>(0)} << endl;

        /// unary factor
        UnaryFactorTranslation2D prior_k{symbol('x', i), gtsam::Vector2{conf.segment<dim_conf>(0)}, K_0};
        cout << prior_k.get_Qc() << endl;
        prior_k.printKeys();

        /// Factored optimizer
        VectorXd test_vec = (VectorXd(4) << 1.0, 1.0, 0.0, 0.0).finished();
        double temp {errorWrapperPrior(test_vec, prior_k)};
        cout << "temp" << endl << temp << endl;
        vec_factorized_opts.emplace_back(OptimizerFactorPriorPRGH{dim_theta, errorWrapperPrior, prior_k});

    }

    /// The joint optimizer
    OptimizerPriorPointRobot optimizer{vec_Pks, vec_factorized_opts};

    return 0;
}