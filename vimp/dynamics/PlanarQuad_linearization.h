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

    MatrixXd matrix_noAngle = zt;
    // matrix_noAngle.col(2).setZero();
    // matrix_noAngle.col(5).setZero();

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
        fzi << matrix_noAngle(i, 3) * std::cos(matrix_noAngle(i, 2)) - matrix_noAngle(i, 4) * std::sin(matrix_noAngle(i, 2)),
               matrix_noAngle(i, 3) * std::sin(matrix_noAngle(i, 2)) + matrix_noAngle(i, 4) * std::cos(matrix_noAngle(i, 2)),
               matrix_noAngle(i, 5),
               matrix_noAngle(i, 4) * matrix_noAngle(i, 5) - g * std::sin(matrix_noAngle(i, 2)),
              -matrix_noAngle(i, 3) * matrix_noAngle(i, 5) - g * std::cos(matrix_noAngle(i, 2)),
               0.0;

        hA[i] << 0.0, 0.0, -matrix_noAngle(i, 3) * std::sin(matrix_noAngle(i, 2)) - matrix_noAngle(i, 4) * std::cos(matrix_noAngle(i, 2)), std::cos(matrix_noAngle(i, 2)), -std::sin(matrix_noAngle(i, 2)), 0.0,
                 0.0, 0.0, matrix_noAngle(i, 3) * std::cos(matrix_noAngle(i, 2)) - matrix_noAngle(i, 4) * std::sin(matrix_noAngle(i, 2)), std::sin(matrix_noAngle(i, 2)), std::cos(matrix_noAngle(i, 2)), 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                 0.0, 0.0, -g * std::cos(matrix_noAngle(i, 2)), 0.0, matrix_noAngle(i, 5), matrix_noAngle(i, 4),
                 0.0, 0.0, g * std::sin(matrix_noAngle(i, 2)), -matrix_noAngle(i, 5), 0.0, -matrix_noAngle(i, 3),
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        hB[i] = hBi;
        ha[i] = fzi - hA[i] * matrix_noAngle.row(i).transpose();
    }

    return std::make_tuple(hA, hB, ha);
}