/**
 * @file test_GH.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test the Gauss-Hermite estimator using a known 1d experiment.
 * @version 0.1
 * @date 2023-07-18
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "gvimp/GaussHermite.h"

using namespace Eigen;
using namespace vimp;

MatrixXd phi(const VectorXd& vec_x){
    double x = vec_x(0);
    double mu_p = 20, f = 400, b = 0.1, sig_r_sq = 0.09;
    double sig_p_sq = 9;

    // y should be sampled. for single trial just give it a value.
    double y = f*b/mu_p + 0.05;

    MatrixXd res(1, 1);
    res(0, 0) = ((x - mu_p)*(x - mu_p) / sig_p_sq / 2 + (y - f*b/x)*(y - f*b/x) / sig_r_sq / 2); 

    return res;

}

MatrixXd xmu_phi(const VectorXd& vec_x){
    double x = vec_x(0);
    double mu_p = 20, f = 400, b = 0.1, sig_r_sq = 0.09;
    double sig_p_sq = 9;

    // y should be sampled. for single trial just give it a value.
    double y = f*b/mu_p + 0.05;

    MatrixXd res(1, 1);
    res(0, 0) = (x - mu_p) * ((x - mu_p)*(x - mu_p) / sig_p_sq / 2 + (y - f*b/x)*(y - f*b/x) / sig_r_sq / 2); 

    return res;
}

int main(){

    int deg = 6, dim = 1;
    VectorXd mean(1);
    mean.setZero();
    mean(0) = 20.0;
    MatrixXd cov(1, 1);
    cov.setZero();
    cov(0, 0) = 9.0;

    using Function = std::function<MatrixXd(const VectorXd&)>;    
    GaussHermite<Function> gh(deg, dim, mean, cov);

    MatrixXd phi_mu = phi(mean);
    MatrixXd phi_mu_GT(1,1);
    phi_mu_GT(0,0) = 0.013888888888889;
    std::cout << "phi_func value at mu_p" << std::endl << phi_mu(0,0) << std::endl;
    std::cout << "phi_func value at mu_p Ground Truth" << std::endl << phi_mu_GT(0,0) << std::endl;
    
    MatrixXd xmu_phi_mu = xmu_phi(mean);
    MatrixXd xmu_phi_mu_GT(1,1);
    xmu_phi_mu_GT(0,0) = 0.0;
    std::cout << "phi_func value at mu_p" << std::endl << xmu_phi_mu(0,0) << std::endl;
    std::cout << "phi_func value at mu_p Ground Truth" << std::endl << xmu_phi_mu_GT(0,0) << std::endl;


    // Integrations
    // integration of phi
    std::shared_ptr<Function> p_phi;
    p_phi = std::make_shared<Function>(phi);
    MatrixXd E_Phi = gh.Integrate(phi);
    double E_Phi_GT = 1.1129;
    std::cout << "E_Phi" << std::endl << E_Phi(0,0) << std::endl;
    std::cout << "E_Phi Ground Truth" << std::endl << E_Phi_GT << std::endl;

    // integration of (x-mu)*phi
    std::shared_ptr<Function> p_xmu_phi;
    p_xmu_phi = std::make_shared<Function>(xmu_phi);
    MatrixXd E_xmu_phi = gh.Integrate(xmu_phi);
    double E_xmu_phi_GT = -1.2144;
    std::cout << "E_xmu_phi" << std::endl << E_xmu_phi(0,0) << std::endl;
    std::cout << "E_xmu_phi Ground Truth" << std::endl << E_xmu_phi_GT << std::endl;

    return 0;
}