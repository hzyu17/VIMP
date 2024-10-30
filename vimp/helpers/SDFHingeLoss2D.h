#include <iostream>
#include <Eigen/Dense>
#include <vector>

// Hinge loss gradient function
std::pair<double, double> hinge_loss_gradient(double dist, double eps_obs, double slope) {
    if (dist > eps_obs) {
        return {0.0, 0.0};
    }
    if (dist <= eps_obs) {
        return {slope * dist, -slope};
    }
    return {0.0, 0.0}; // default case
}

// Hinge SDF loss gradient function
std::pair<double, Eigen::Vector2d> hingesdfloss_gradient(const Eigen::Vector2d& pt, const SDF2D& sdf_2d, double eps_obs, double slope = 1.0) {
    double dist = sdf_2d.getSignedDistance(pt);
    Eigen::Vector2d g_sdf = sdf_2d.getGradient(pt);

    auto [h, g_h] = hinge_loss_gradient(dist, eps_obs, slope);

    return {h, g_h * g_sdf};
}

// Vector collision loss gradient function
std::pair<double, Eigen::MatrixXd> vec_colloss_gradient(const Eigen::MatrixXd& sig_obs, const Eigen::MatrixXd& vec_pts, const SDF2D& sdf_2d, double eps_obs, double slope) {
    int n_pts = vec_pts.rows();
    int pt_dim = vec_pts.cols();

    Eigen::VectorXd vec_hinge = Eigen::VectorXd::Zero(n_pts);
    Eigen::MatrixXd vec_grad_hinge_x = Eigen::MatrixXd::Zero(n_pts, pt_dim);

    for (int i_pt = 0; i_pt < n_pts; ++i_pt) {
        Eigen::Vector2d pt = vec_pts.row(i_pt);

        auto [h, g_h_pt] = hingesdfloss_gradient(pt, sdf_2d, eps_obs, slope);
        vec_hinge(i_pt) = h;
        vec_grad_hinge_x.row(i_pt) = g_h_pt;
    }

    Eigen::MatrixXd Sig_obs = sig_obs * Eigen::MatrixXd::Identity(n_pts, n_pts);
    double collision_cost = vec_hinge.transpose() * Sig_obs * vec_hinge;

    Eigen::MatrixXd vec_grad_collision_x = 2.0 * (Sig_obs * vec_hinge).asDiagonal() * vec_grad_hinge_x;

    return {collision_cost, vec_grad_collision_x};
}