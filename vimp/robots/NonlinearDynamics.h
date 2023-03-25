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

#include <Eigen/Dense>

namespace vimp{

class NonlinearDynamics{
public:
    NonlinearDynamics(){};
     /**
      * @brief Linearization for use in proximal gradient covariance steering.
      * 
      * @param x linearization point
      * @param sig time scaling factor
      * @param Ak iteration variable Ak
      * @param Sigk iteration variable Sigk
      * @return std::tuple<MatrixXd, MatrixXd, VectorXd, VectorXd>,
      *         representing (At, Bt, at, nTr).
      */
    virtual std::tuple<MatrixXd, MatrixXd, VectorXd, VectorXd> Linearize(const Vector4d& x, 
                                                                        double sig, 
                                                                        const Eigen::Matrix4d& Ak, 
                                                                        const Eigen::Matrix4d& Sigk);
};

}// namespace vimp