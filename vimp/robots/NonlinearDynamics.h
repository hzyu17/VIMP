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
#include <string>

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
    virtual std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::VectorXd, Eigen::VectorXd> linearize(const Eigen::VectorXd& x, 
                                                                                                     double sig, 
                                                                                                     const Eigen::MatrixXd& Ak, 
                                                                                                     const Eigen::MatrixXd& Sigk);

    virtual ~NonlinearDynamics(){};

};

}// namespace vimp