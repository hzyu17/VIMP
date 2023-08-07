#include "instances/PlanarFactorPR.h"
#include "robots/PlanarPointRobotSDFMultiObsExample.h"
#include <gtest/gtest.h>

using namespace gpmp2;
using namespace Eigen;
using namespace vimp;
using namespace std;


int dim_state = 4, num_states = 1; 

/// Obs factor
double cost_sigma = 0.5, epsilon = 4.0;

// An example pr and sdf
vimp::PlanarPointRobotSDFMultiObsExample planar_pr_sdf("map3");
gpmp2::PointRobotModel pRModel = std::move(planar_pr_sdf.pRmodel());
gpmp2::PlanarSDF sdf = std::move(planar_pr_sdf.sdf());

// collision factor
gpmp2::ObstaclePlanarSDFFactorPointRobot collision_k{gtsam::symbol('x', 0), pRModel, sdf, cost_sigma, epsilon};

// Eigen helper
EigenWrapper ei;

TEST(ColCost, sdf_map){

    // Test point
    VectorXd conf_test(2);
    conf_test.setZero();
    conf_test << 11.3321059864421, 9.16117246531728;

    // signed distance ground truth
    double signed_distance = planar_pr_sdf.sdf().getSignedDistance(conf_test);
    double signed_distance_gt = 4.5321;
    ASSERT_LE(abs(signed_distance - signed_distance_gt), 1e-5);

    // error vector ground truth
    VectorXd vec_err = collision_k.evaluateError(conf_test);  
    VectorXd err_vec_gt(1);
    err_vec_gt(0) = 0.9679;
    ASSERT_LE((err_vec_gt - vec_err).norm(), 1e-5);

}


TEST(ColCost, collision_cost){
    
    /// Vector of base factored optimizers
    vector<std::shared_ptr<GVIFactorizedBase>> vec_factor_opts;

    
    MatrixXd Pk_col{MatrixXd::Zero(2, 4)};
    Pk_col.block(0, 0, 2, 2) = std::move(MatrixXd::Identity(2, 2));

    /// Factored optimizer
    std::shared_ptr<PlanarSDFFactorPR> p_obs{new PlanarSDFFactorPR{2, dim_state, cost_sdf_pR, collision_k, num_states, 0, 1.0, 10.0}};
    vec_factor_opts.emplace_back(p_obs);

    /// The joint optimizer
    GVIGH<GVIFactorizedBase> optimizer{vec_factor_opts, dim_state, num_states};

    /// Set initial value to the linear interpolation
    VectorXd joint_init_theta(4);
    joint_init_theta.setZero();
    joint_init_theta << 11.3321059864421, 9.16117246531728, 0, 0;

    optimizer.set_mu(joint_init_theta);

    optimizer.set_GH_degree(10);

    // *********** In the case of a very small covariance, the expected value should equal the mean value.
    MatrixXd precision = MatrixXd::Identity(4, 4) * 100000.0;
    SpMat precision_sp = precision.sparseView();

    optimizer.set_precision(precision_sp);

    double cost = optimizer.cost_value_no_entropy();
    cout << "cost_w_o_entropy " << endl << cost << endl;

    VectorXd vec_err(1);
    vec_err = collision_k.evaluateError(joint_init_theta.segment(0, 2));  

    double temperature = 10.0;
    double  cost_expected = vec_err(0) * vec_err(0) / cost_sigma / temperature;
    ASSERT_LE(abs(cost - cost_expected), 1e-5);


    // *********** In the case of a normal covariance, compared with the ground truth from matlab.
    precision = MatrixXd::Identity(4, 4);
    precision_sp = precision.sparseView();

    optimizer.set_precision(precision_sp);

    double cost_new = optimizer.cost_value_no_entropy();
    cout << "cost_new " << endl << cost_new << endl;

    cost_expected = 0.3710;

    ASSERT_LE(abs(cost_new - cost_expected), 1e-2);

}

// *** Test the prior cost with a fixed Gaussian target
TEST(PriorCost, fixed_cost){
    /// Vector of base factored optimizers
    vector<std::shared_ptr<GVIFactorizedBase>> vec_factors;

    int dim_state = 2;
    int n_states = 1;
    double boundary_penalties = 1e4;

    MatrixXd K0_fixed{MatrixXd::Identity(dim_state, dim_state) * boundary_penalties};

    VectorXd theta_0(2);
    theta_0.setZero();
    theta_0 << 11.3333333333333, 9.33333333333333;

    FixedPriorGP fixed_gp{K0_fixed, theta_0};
    vec_factors.emplace_back(new FixedGpPrior{dim_state, 
                                              dim_state, 
                                              cost_fixed_gp, 
                                              fixed_gp, 
                                              n_states, 
                                              0, 
                                              10.0, 
                                              100.0});

    /// The joint optimizer
    GVIGH<GVIFactorizedBase> optimizer{vec_factors, dim_state, num_states};
    optimizer.set_mu(theta_0);

    optimizer.set_GH_degree(6);

    // initial precision matrix for the optimization
    MatrixXd precision = MatrixXd::Identity(dim_state, dim_state) * boundary_penalties;

    optimizer.set_precision(precision.sparseView());

    double cost = optimizer.cost_value_no_entropy();
    cout << "cost fixed linear factor" << endl << cost << endl;

    double cost_expected = 2.0000e-08; 

    ASSERT_LE(abs(cost - cost_expected), 1e-6);

}


// *** Tests for the linear dynamics prior
// ======================= global varibles ======================= 
int dim_conf = 2;
int state_dim = 4;
int n_states = 2;
int joint_state_dim = n_states * state_dim;
double initial_precision_factor = 1000.0;

double coeff_Qc = 0.8;
double delt_t = 0.1667;

MatrixXd Qc{MatrixXd::Identity(dim_conf, dim_conf)*coeff_Qc};

VectorXd theta_0{(VectorXd(4) << 0, 0, 11.3333333333333, 9.33333333333333).finished()};
VectorXd theta_1{(VectorXd(4) << 1.88888888888889, 1.55555555555556, 11.3333333333333, 9.33333333333333).finished()};

MinimumAccGP lin_gp{Qc, 0, delt_t, theta_0};

vector<std::shared_ptr<GVIFactorizedBase>> vec_factors{std::make_shared<GVIFactorizedBase>(
                                                            LinearGpPrior{joint_state_dim,  state_dim, 
                                                            cost_linear_gp,  lin_gp, n_states, 0, 10.0, 100.0}
                                                            )
                                                       };

/// The joint optimizer
GVIGH<GVIFactorizedBase> optimizer{vec_factors, state_dim, n_states};

// initial precision matrix for the optimization
MatrixXd precision{MatrixXd::Identity(joint_state_dim, joint_state_dim) * initial_precision_factor};

VectorXd joint_mean{(VectorXd(8) << theta_0, theta_1).finished()};


// ======================= end global varibles ======================= 

// *** Test block extraction of the factors from joint level mean and covariances.
TEST(PriorCost, factorization){
    optimizer.set_mu(joint_mean);
    optimizer.set_GH_degree(6);
    
    optimizer.set_precision(precision.sparseView());

    SpMat joint_cov = optimizer.covariance();

    VectorXd extracted_mean = vec_factors[0]->extract_mu_from_joint(joint_mean);
    SpMat extracted_covariance = vec_factors[0]->extract_cov_from_joint(joint_cov);
    // should equal
    ASSERT_LE((extracted_mean - joint_mean).norm(), 1e-5);
    ASSERT_LE((extracted_covariance - joint_cov).norm(), 1e-5);

}


// *** Test linear dynamics prior cost
TEST(PriorCost, dynamics_prior_cost){
    
    optimizer.set_mu(joint_mean);
    optimizer.set_GH_degree(6);
    
    optimizer.set_precision(precision.sparseView());

    double cost_prior = optimizer.cost_value_no_entropy();
    cout << "cost linear prior" << endl << cost_prior << endl;

    double cost_prior_expected = 0.6537; 

    ASSERT_LE(abs(cost_prior - cost_prior_expected), 1e-4);
}
