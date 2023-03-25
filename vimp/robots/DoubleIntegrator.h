/**
 * @file DoubleIntegrator.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Define the double integrator with drag dynamics and linearize it.
 * @version 0.1
 * @date 2023-03-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "NonlinearDynamics.h"
#include <TinyAD/Scalar.hh>
#include "../helpers/eigen_wrapper.h"

using namespace Eigen;

namespace vimp{

class DoubleIntegrator : public NonlinearDynamics{

public:
    DoubleIntegrator(){};
    /**
 * @brief Linearization of a double integrator dynamics with drag (time invariant system).
 * 
 * @param x linearization point
 * @param sig time scaling factor
 * @return std::tuple<Matrix4d, MatrixXd, Vector4d, Vector4d> At, Bt, at, nTr
 */
std::tuple<MatrixXd, MatrixXd, VectorXd, VectorXd> Linearize(const Vector4d& x, 
                                                            double sig, 
                                                            const Eigen::Matrix4d& Ak, 
                                                            const Eigen::Matrix4d& Sigk){
    using ADouble4 = TinyAD::Double<4>;

    EigenWrapper ei;

    // drag coefficient
    double cd = 0.005;
    
    Eigen::Vector4<ADouble4> xad = ADouble4::make_active(x);

    // B
    Eigen::MatrixXd B{Eigen::MatrixXd::Zero(4, 2)};
    B << 0, 0,
         0, 0,
         1, 0,
         0, 1;
    B = sig*B;

    // BBT
    Eigen::MatrixXd p_invBBT(4,4);
    p_invBBT << 0,0,0,0,
                0,0,0,0,
                0,0,1,0,
                0,0,0,1;
    p_invBBT = p_invBBT/sig/sig;

    Eigen::Matrix4d hAk{Eigen::MatrixXd::Zero(4, 4)};
    hAk <<  0,  0,                                 1,                                           0,
            0,  0,                                 0,                                           1, 
            0,  0,  cd*(2*x(2)*x(2)+x(3)*x(3))/sqrt(x(2)*x(2)+x(3)*x(3)),   cd*x(2)*x(3)/sqrt(x(2)*x(2)+x(3)*x(3)),
            0,  0,  cd*x(2)*x(3)/sqrt(x(2)*x(2)+x(3)*x(3)),                 cd*(x(2)*x(2)+2*x(3)*x(3))/sqrt(x(2)*x(2)+x(3)*x(3));
    hAk = sig * hAk;

    Eigen::Vector4d f{Eigen::VectorXd::Zero(4)};
    f << x(2), 
         x(3), 
         -cd*x(2)*sqrt(x(2)*x(2) + x(3)*x(3)),
         -cd*x(3)*sqrt(x(2)*x(2) + x(3)*x(3));
    f = sig * f;

    Eigen::Vector4d hak{Eigen::VectorXd::Zero(4)};
    hak = f - hAk*x;

    Eigen::Matrix4<ADouble4> grad_f_T;
    grad_f_T << 0,  0,                                 1,                                                                0,
                0,  0,                                 0,                                                                1, 
                0,  0,  cd*(2*xad(2)*xad(2)+xad(3)*xad(3))/sqrt(xad(2)*xad(2)+xad(3)*xad(3)),   cd*xad(2)*xad(3)/sqrt(xad(2)*xad(2)+xad(3)*xad(3)),
                0,  0,  cd*xad(2)*xad(3)/sqrt(xad(2)*xad(2)+xad(3)*xad(3)),                     cd*(xad(2)*xad(2)+2*xad(3)*xad(3))/sqrt(xad(2)*xad(2)+xad(3)*xad(3));
    grad_f_T = sig * grad_f_T;

    Eigen::Matrix4<ADouble4> grad_f;
    grad_f = grad_f_T.transpose();

    // // grad(Tr(pinv(BBT)*(grad_f_T - Ak)*Sigk*(grad_f_T' - Ak')))
    Eigen::MatrixXd Ak_T(4,4);
    Ak_T = Ak.transpose();

    Eigen::Matrix4<ADouble4> temp1;
    temp1 = grad_f_T - Ak;

    Eigen::Matrix4<ADouble4> temp2;
    temp2 = grad_f - Ak_T;

    Eigen::Matrix4<ADouble4> temp3;
    temp3 = p_invBBT*temp1;

    Eigen::Matrix4<ADouble4> temp4;
    temp4 = temp3*Sigk;
    
    auto res = temp4*temp2;
    auto nTr = res.trace().grad;
    
    Eigen::Vector4d VnTr;
    VnTr << nTr(0), nTr(1), nTr(2), nTr(3);

    return std::make_tuple(hAk, B, hak, VnTr);

}

};



}
