/**
 * @file PlanarQuadDynamics.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The planar quadrotor dynamics.
 * @version 0.1
 * @date 2023-05-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "NonlinearDynamics.h"
#include "../3rd-part/TinyAD/Scalar.hh"

using namespace Eigen;

namespace vimp{

class PlanarQuadrotor : public NonlinearDynamics{

public:
    PlanarQuadrotor(int nx, int nu, int nt):NonlinearDynamics(nx, nu, nt),
                                             _nx(nx),
                                             _nu(nu),
                                             _nt(nt){}


/**
 * @brief Linearization.
 * 
 * @param x linearization point
//  * @param sig time scaling factor
 * @return std::tuple<Matrix(3)d, MatrixXd, Vector4d, Vector4d> At, Bt, at, nTr
 */
std::tuple<MatrixXd, MatrixXd, VectorXd, VectorXd> linearize_at(const VectorXd& x, 
                                                                const MatrixXd& Ak, 
                                                                const MatrixXd& Sigk) override
{
    using ADouble6 = TinyAD::Double<6>;

    // mechanical parameters
    double l = 0.25;
    double m = 0.486;
    double J = 0.00383;
    double g = 9.81;
    
    Vector<ADouble6, 6> xad = ADouble6::make_active(x);

    // B
    MatrixXd B{MatrixXd::Zero(_nx, _nu)};
    B << 0, 0,
         0, 0,
         0, 0,
         0, 0,
         1/sqrt(2), 1/sqrt(2),
         1/sqrt(2), -1/sqrt(2);

    // BBT
    MatrixXd p_invBBT(_nx,_nx);
    p_invBBT << 0,0,0,0,0,0,
                0,0,0,0,0,0,
                0,0,0,0,0,0,
                0,0,0,0,0,0,
                0,0,0,0,1,0,
                0,0,0,0,0,1;

    MatrixXd hAk{MatrixXd::Zero(6, 6)};
    hAk <<  0,    0,     -x(3)*sin(x(2))-x(4)*cos(x(2)),    cos(x(2)),       -sin(x(2)),       0, 
            0,    0,     x(3)*cos(x(2))-x(4)*sin(x(2)),     sin(x(2)),        cos(x(2)),       0,
            0,    0,                  0,                       0,                0,            1,
            0,    0,              -g*cos(x(2)),                0,               x(5),        x(4),
            0,    0,              g*sin(x(2)),              -x(5),               0,         -x(3),
            0,    0,                  0,                       0,                0,            0;

    VectorXd f{VectorXd::Zero(6)};
    f << x(3)*cos(x(2)) - x(4)*sin(x(2)),
        x(3)*sin(x(2)) + x(4)*cos(x(2)),
        x(5),
        x(4)*x(5)-g*sin(x(2)),
        -x(3)*x(5)-g*cos(x(2)),
        0;

    VectorXd hak{VectorXd::Zero(6)};
    hak = f - hAk*x;

    Matrix<ADouble6, 6, 6> grad_f_T;
    grad_f_T << 0,    0,         -x(3)*sin(x(2))-x(4)*cos(x(2)),     cos(x(2)),       -sin(x(2)),            0, 
                0,    0,          x(3)*cos(x(2))-x(4)*sin(x(2)),     sin(x(2)),        cos(x(2)),            0,
                0,    0,                      0,                        0,                  0,               1,
                0,    0,               -g*cos(x(2)),                    0,                x(5),           x(4),
                0,    0,                g*sin(x(2)),                  -x(5),                0,           -x(3),
                0,    0,                      0,                        0,                  0,               0;

    Matrix<ADouble6, 6, 6> grad_f;
    grad_f = grad_f_T.transpose();

    // // grad(Tr(pinv(BBT)*(grad_f_T - Ak)*Sigk*(grad_f_T' - Ak')))
    Matrix<ADouble6, 6, 6> Ak_T;
    Ak_T = Ak.transpose();

    Matrix<ADouble6, 6, 6> temp1;
    temp1 = grad_f_T - Ak;

    Matrix<ADouble6, 6, 6> temp2;
    temp2 = grad_f - Ak_T;

    Matrix<ADouble6, 6, 6> temp3;
    temp3 = p_invBBT*temp1;

    Matrix<ADouble6, 6, 6> temp4;
    temp4 = temp3*Sigk;
    
    auto res = temp4*temp2;
    auto nTr = res.trace().grad;
    
    VectorXd VnTr(6,6);
    VnTr << nTr(0), nTr(1), nTr(2), nTr(3), nTr(4), nTr(5);

    return std::make_tuple(hAk, B, hak, VnTr);

}

/**
 * @brief Linearize along a trajectory.
 * 
 * @param x A trajectory of states, in shape (dim_state, nt).
 * @return std::tuple<Matrix3D, Matrix3D, Matrix3D, Matrix3D> return (hAt, hBt, hat, nTrt)
 */
std::tuple<LinearDynamics, Matrix3D> linearize(const Matrix3D& xt, 
                                                const Matrix3D& Akt, 
                                                const Matrix3D& Sigkt) override
{   
    // The result collectors for all time points
    Matrix3D hAt(_nx, _nx, _nt), Bt(_nx, _nu, _nt), hat(_nx, 1, _nt), nTrt(_nx, 1, _nt);

    // The i_th matrices
    Eigen::VectorXd zki(_nx), hai(_nx), nTri(_nx);
    Eigen::MatrixXd Aki(_nx, _nx), hAi(_nx, _nx), Bi(_nx, _nu);
    Eigen::MatrixXd Sigki(_nx, _nx);
    std::tuple<MatrixXd, MatrixXd, VectorXd, VectorXd> resi;

    for (int i=0; i<_nt; i++){
        // Get the nominals
        zki = _ei.decomp3d(xt, _nx, 1, i);
        Aki = _ei.decomp3d(Akt, _nx, _nx, i);
        Sigki = _ei.decomp3d(Sigkt, _nx, _nx, i);

        // Get the linearization results
        resi = linearize_at(zki, Aki, Sigki);
        hAi = std::get<0>(resi);
        Bi = std::get<1>(resi);
        hai = std::get<2>(resi);
        nTri = std::get<3>(resi);

        // Assamble into the 3d matrices
        _ei.compress3d(hAi, hAt, i);
        _ei.compress3d(Bi, Bt, i);
        _ei.compress3d(hai, hat, i);
        _ei.compress3d(nTri, nTrt, i);
        
    }
    return std::make_tuple(LinearDynamics{_nx, _nu, _nt, hAt, Bt, hat}, nTrt);
}


protected:
    int _nt, _nx, _nu;
};



}