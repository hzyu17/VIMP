/**
 * @file DoubleIntegratorDraged.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Define the double integrator with drag dynamics and linearize it.
 * @version 0.1
 * @date 2023-03-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "dynamics/NonlinearDynamics.h"
#include "3rdparty/TinyAD/Scalar.hh"

using namespace Eigen;

namespace vimp{

class DoubleIntegrator : public NonlinearDynamics{

public:
    DoubleIntegrator(int nx, int nu, int nt):NonlinearDynamics(nx, nu, nt),
                                             _nx(nx),
                                             _nu(nu),
                                             _nt(nt){}

    // virtual ~DoubleIntegrator(){}

/**
 * @brief Linearization of a double integrator dynamics with drag (time invariant system).
 * 
 * @param x linearization point
 * @return std::tuple<Matrix4d, MatrixXd, Vector4d, Vector4d> At, Bt, at, nTr
 */
std::tuple<MatrixXd, MatrixXd, VectorXd, VectorXd> linearize_at(const VectorXd& x, 
                                                                const MatrixXd& Ak, 
                                                                const MatrixXd& Sigk) override
{
    using ADouble4 = TinyAD::Double<4>;

    // drag coefficient
    double cd = 0.005;
    
    Vector4<ADouble4> xad = ADouble4::make_active(x);

    // B
    MatrixXd B{MatrixXd::Zero(4, 2)};
    B << 0, 0,
         0, 0,
         1, 0,
         0, 1;
    // B = sig*B;

    // BBT
    MatrixXd p_invBBT(4,4);
    p_invBBT << 0,0,0,0,
                0,0,0,0,
                0,0,1,0,
                0,0,0,1;
    // p_invBBT = p_invBBT/sig/sig;

    Matrix4d hAk{MatrixXd::Zero(4, 4)};
    hAk <<  0,  0,                                 1,                                           0,
            0,  0,                                 0,                                           1, 
            0,  0,  -cd*(2*x(2)*x(2)+x(3)*x(3))/sqrt(x(2)*x(2)+x(3)*x(3)),   -cd*x(2)*x(3)/sqrt(x(2)*x(2)+x(3)*x(3)),
            0,  0,  -cd*x(2)*x(3)/sqrt(x(2)*x(2)+x(3)*x(3)),                 -cd*(x(2)*x(2)+2*x(3)*x(3))/sqrt(x(2)*x(2)+x(3)*x(3));
    // hAk = sig * hAk;

    Vector4d f{VectorXd::Zero(4)};
    f << x(2), 
         x(3), 
         -cd*x(2)*sqrt(x(2)*x(2) + x(3)*x(3)),
         -cd*x(3)*sqrt(x(2)*x(2) + x(3)*x(3));
    // f = sig * f;

    Vector4d hak{VectorXd::Zero(4)};
    hak = f - hAk*x;

    Matrix4<ADouble4> grad_f_T;
    grad_f_T << 0,  0,                                 1,                                                                0,
                0,  0,                                 0,                                                                1, 
                0,  0,  -cd*(2*xad(2)*xad(2)+xad(3)*xad(3))/sqrt(xad(2)*xad(2)+xad(3)*xad(3)),   -cd*xad(2)*xad(3)/sqrt(xad(2)*xad(2)+xad(3)*xad(3)),
                0,  0,  -cd*xad(2)*xad(3)/sqrt(xad(2)*xad(2)+xad(3)*xad(3)),                     -cd*(xad(2)*xad(2)+2*xad(3)*xad(3))/sqrt(xad(2)*xad(2)+xad(3)*xad(3));
    // grad_f_T = sig * grad_f_T;

    Matrix4<ADouble4> grad_f;
    grad_f = grad_f_T.transpose();

    // // grad(Tr(pinv(BBT)*(grad_f_T - Ak)*Sigk*(grad_f_T' - Ak')))
    MatrixXd Ak_T(4,4);
    Ak_T = Ak.transpose();

    Matrix4<ADouble4> temp1;
    temp1 = grad_f_T - Ak;

    Matrix4<ADouble4> temp2;
    temp2 = grad_f - Ak_T;

    Matrix4<ADouble4> temp3;
    temp3 = p_invBBT*temp1;

    Matrix4<ADouble4> temp4;
    temp4 = temp3*Sigk;
    
    auto res = temp4*temp2;
    auto nTr = res.trace().grad;
    
    Vector4d VnTr;
    VnTr << nTr(0), nTr(1), nTr(2), nTr(3);

    return std::make_tuple(hAk, B, hak, VnTr);

}

/**
 * @brief Linearize along a trajectory.
 * 
 * @param x A trajectory of states, in shape (dim_state, nt).
 * @return std::tuple<Matrix3D, Matrix3D, Matrix3D, Matrix3D> return (hAt, hBt, hat, nTrt)
 */
std::tuple<LinearDynamics, Matrix3D> linearize(const Matrix3D& xt, 
                                                // double sig, 
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
        zki = _ei.decomp3d(xt, _nx, 1, i);
        Aki = _ei.decomp3d(Akt, _nx, _nx, i);
        Sigki = _ei.decomp3d(Sigkt, _nx, _nx, i);
        // get the linearization results
        // resi = linearize_at(zki, sig, Aki, Sigki);
        resi = linearize_at(zki, Aki, Sigki);
        hAi = std::get<0>(resi);
        Bi = std::get<1>(resi);
        hai = std::get<2>(resi);
        nTri = std::get<3>(resi);
        // assamble into the 3d matrices
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
