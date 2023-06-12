/**
 * @file NonlinearDynamics.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief A abstract class for nonlinear dynamics and its linearization for use in Covariance Steering.
 * @version 0.1
 * @date 2023-03-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include "../helpers/eigen_wrapper.h"
#include "LinearDynamics.h"

using namespace Eigen;

namespace vimp{

class NonlinearDynamics: public Dynamics{
public:
    NonlinearDynamics(){}
    NonlinearDynamics(int nx, int nu, int nt):Dynamics(nx, nu, nt){}
    virtual ~NonlinearDynamics(){}
    

    virtual std::tuple<MatrixXd, MatrixXd, VectorXd, VectorXd> linearize_at(const VectorXd& x, 
                                                                                    // double sig, 
                                                                                    const MatrixXd& Ak, 
                                                                                    const MatrixXd& Sigk){}
    /**
    * @brief Linearization for use in proximal gradient covariance steering.
    * 
    * @param x linearization point
    // * @param sig time scaling factor
    * @param Ak iteration variable Ak
    * @param Sigk iteration variable Sigk
    * @return std::tuple<LinearDynamics, Matrix3D>,
    *         representing (At, Bt, at, nTr).
    */
    virtual std::tuple<LinearDynamics, Matrix3D> linearize(const Matrix3D& xt, 
                                                            // double sig, 
                                                            const Matrix3D& Akt, 
                                                            const Matrix3D& Sigkt){}

};

}// namespace vimp