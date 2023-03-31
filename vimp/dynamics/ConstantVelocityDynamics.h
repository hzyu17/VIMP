/**
 * @file ConstantVelocityDynamics.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The definition of constant velocity dynamics.
 * @version 0.1
 * @date 2023-03-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "LinearDynamics.h"

namespace vimp{

class ConstantVelDynamics: public LTI{

public:
    ConstantVelDynamics(int nx, int nt):LTI(nx, 1, nt)
    {
        MatrixXd A(_nx, _nx), B(_nx, _nu), a(_nx, 1);
        A.setZero(); B.setZero(); a.setZero();
        A.block(0, _nx/2, _nx/2, _nx/2) = MatrixXd::Identity(_nx/2, _nx/2);
        B.block(_nx/2, 0, _nx/2, _nu) = VectorXd::Ones(_nx/2, 1);
        update_matrices(A, B, a);
    }

};

}