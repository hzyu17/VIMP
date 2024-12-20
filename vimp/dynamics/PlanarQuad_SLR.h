/**
 * @file PlanarQuad_SLR.h
 * @author Zinuo Chang (zchang40@gatech.edu)
 * @brief 
 * @version 0.1
 * @date 2024-12-06
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "helpers/CudaOperation.h"
#include "quadrature/SparseGaussHermite_Cuda.h"

using namespace Eigen;

using GHFunction = std::function<MatrixXd(const VectorXd&)>;
using GH = SparseGaussHermite_Cuda<GHFunction>;

// const double g = 9.81;
// const double m = 1.0; // Example mass
// const double l = 1.0; // Example length
// const double J = 1.0; // Example inertia

std::tuple<std::vector<MatrixXd>, std::vector<VectorXd>> linearization_SLR(const MatrixXd& trajectory, const SpMat& covariace, std::shared_ptr<GH> gh_ptr, int dim_state, int n_states){
    std::vector<MatrixXd> sigmapts_vec(n_states);
    std::vector<VectorXd> mean_vec(n_states);

    VectorXd weights = gh_ptr->weights();
    MatrixXd y_bar = MatrixXd::Zero(dim_state, n_states);
    MatrixXd P_xy = MatrixXd::Zero(dim_state, dim_state*n_states);

    for (int i = 0; i < n_states; i++)
    {
        // MatrixXd covariace_i = MatrixXd::Identity(dim_state, dim_state) * 1.0;
        MatrixXd covariace_i = covariace.block(i*dim_state, i*dim_state, dim_state, dim_state);
        gh_ptr->update_P(covariace_i);
        gh_ptr->update_mean(trajectory.row(i));
        gh_ptr->update_sigmapoints();

        sigmapts_vec[i] = gh_ptr->sigmapts();
    }

    int sigma_rows = sigmapts_vec[0].rows();
    std::cout << "sigma_rows: " << sigma_rows << std::endl;

    MatrixXd sigmapts_mat(sigma_rows, n_states*dim_state);

    for (int i = 0; i < n_states; i++)
        sigmapts_mat.block(0, i * dim_state, sigma_rows, dim_state) = sigmapts_vec[i];

    CudaOperation_SLR cuda_SLR(sigmapts_mat, weights, trajectory.transpose(), dim_state, n_states);
    cuda_SLR.expectationIntegration(y_bar);
    cuda_SLR.covarianceIntegration(P_xy);

    std::vector<MatrixXd> hA(n_states);
    std::vector<VectorXd> ha(n_states);

    for (int i = 0; i < n_states; i++)
    {
        MatrixXd covariace_i = covariace.block(i*dim_state, i*dim_state, dim_state, dim_state);
        // std::cout << "covariace_i: " << std::endl << covariace_i.inverse() << std::endl;
        hA[i] = P_xy.block(0, i*dim_state, dim_state, dim_state).transpose() * (covariace_i.inverse());
        ha[i] = y_bar.col(i) - hA[i] * trajectory.row(i).transpose();
    }
    // std::cout << "n_states: " << n_states << ", traj_row: " << trajectory.rows() << std::endl;

    // std::cout << "x_bar: " << std::endl << trajectory.row(n_states / 2) << std::endl;
    // std::cout << "y_bar: " << std::endl << y_bar.col(n_states / 2).transpose() << std::endl;
    // std::cout << "P_xy: " << std::endl << P_xy.block(0, (n_states / 2) * dim_state, dim_state, dim_state) << std::endl;

    // std::cout << "hA: " << std::endl << hA[n_states / 2] << std::endl;

    return std::make_tuple(hA, ha);
}