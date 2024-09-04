#include <vector>
#include <cmath>
#include <Eigen/Dense>

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