#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/CXX11/Tensor>

using namespace Eigen;

const double g = 9.81;
const double m = 1.0; // Example mass
const double l = 1.0; // Example length
const double J = 1.0; // Example inertia

std::tuple<std::vector<MatrixXd>, std::vector<MatrixXd>, std::vector<VectorXd>> planarquad_linearization_deterministic(const MatrixXd& zt) {
    int nt = zt.rows();
    int nx = zt.cols();

    std::vector<MatrixXd> hA(nt, MatrixXd::Zero(nx, nx));
    std::vector<MatrixXd> hB(nt, MatrixXd::Zero(nx, 2));
    std::vector<VectorXd> ha(nt, VectorXd::Zero(nx));

    MatrixXd hBi(6, 2);
    hBi << 0.0, 0.0, 
           0.0, 0.0, 
           0.0, 0.0, 
           0.0, 0.0, 
           1.0 / std::sqrt(2), 1.0 / std::sqrt(2), 
           1.0 / std::sqrt(2), -1.0 / std::sqrt(2);

    for (int i = 0; i < nt; i++) {
        VectorXd fzi(6);
        fzi << zt(i, 3) * std::cos(zt(i, 2)) - zt(i, 4) * std::sin(zt(i, 2)),
               zt(i, 3) * std::sin(zt(i, 2)) + zt(i, 4) * std::cos(zt(i, 2)),
               zt(i, 5),
               zt(i, 4) * zt(i, 5) - g * std::sin(zt(i, 2)),
              -zt(i, 3) * zt(i, 5) - g * std::cos(zt(i, 2)),
               0.0;

        hA[i] << 0.0, 0.0, -zt(i, 3) * std::sin(zt(i, 2)) - zt(i, 4) * std::cos(zt(i, 2)), std::cos(zt(i, 2)), -std::sin(zt(i, 2)), 0.0,
                 0.0, 0.0, zt(i, 3) * std::cos(zt(i, 2)) - zt(i, 4) * std::sin(zt(i, 2)), std::sin(zt(i, 2)), std::cos(zt(i, 2)), 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                 0.0, 0.0, -g * std::cos(zt(i, 2)), 0.0, zt(i, 5), zt(i, 4),
                 0.0, 0.0, g * std::sin(zt(i, 2)), -zt(i, 5), 0.0, -zt(i, 3),
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        hB[i] = hBi;
        ha[i] = fzi - hA[i] * zt.row(i).transpose();
    }

    return std::make_tuple(hA, hB, ha);
}

/**
 * @brief The forward kinematics to compute the locations of the collision-checking balls.
 * 
 * @param x : states (half): [px, pz, \phi]
 * @param L : planar quad length
 * @param n_balls : number of collision checking balls
 * @param radius : radius
 * @return std::tuple<Eigen::MatrixXd, Eigen::Tensor<double, 3>, Eigen::VectorXd> 
 */
std::tuple<Eigen::MatrixXd, Eigen::Tensor<double, 3>, Eigen::VectorXd>
vec_balls(const Eigen::VectorXd& x, double L, int n_balls, double radius) {
    int nx = x.size();
    Eigen::MatrixXd v_pts = Eigen::MatrixXd::Zero(n_balls, 2);
    Eigen::Tensor<double, 3> v_gradient_ball_states(n_balls, 2, nx);
    v_gradient_ball_states.setZero();
    Eigen::VectorXd v_radius = Eigen::VectorXd::Constant(n_balls, radius);

    double pos_x = x(0);
    double pos_z = x(1);
    double phi = x(2);

    double l_pt_x = pos_x - (L - radius * 1.5) * std::cos(phi) / 2.0;
    double l_pt_z = pos_z - (L - radius * 1.5) * std::sin(phi) / 2.0;

    for (int i = 0; i < n_balls; ++i) {
        double pt_xi = l_pt_x + L * std::cos(phi) / n_balls * i;
        double pt_zi = l_pt_z + L * std::sin(phi) / n_balls * i;
        v_pts(i, 0) = pt_xi;
        v_pts(i, 1) = pt_zi;

        // v_gradient_ball_states(i, 0, 0) = 1.0;
        // v_gradient_ball_states(i, 0, 2) = (L - radius * 1.5) * std::sin(phi) / 2.0 - L * std::sin(phi) / n_balls * i;

        // v_gradient_ball_states(i, 1, 1) = 1.0;
        // v_gradient_ball_states(i, 1, 2) = -(L - radius * 1.5) * std::cos(phi) / 2.0 + L * std::cos(phi) / n_balls * i;
    }

    return std::make_tuple(v_pts, v_gradient_ball_states, v_radius);
}

int main() {
    // Example input
    MatrixXd zt(3, 6);
    zt << 0, 0, 0, 1, 2, 3,
          1, 2, 3, 4, 5, 6,
          2, 4, 6, 8, 10, 12;

    auto [hA, hB, ha] = planarquad_linearization_deterministic(zt);

    // Print the results for verification
    for (size_t i = 0; i < hA.size(); i++) {
        std::cout << "hA[" << i << "]:\n" << hA[i] << "\n\n";
        std::cout << "hB[" << i << "]:\n" << hB[i] << "\n\n";
        std::cout << "ha[" << i << "]:\n" << ha[i].transpose() << "\n\n";
    }

    return 0;
}