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
#include "../helpers/eigen_wrapper.h"

using namespace Eigen;

namespace vimp{

class NonlinearDynamics{
public:
    NonlinearDynamics(){}
    NonlinearDynamics(int nx, int nu, int nt):_nx(nx),
                                              _nu(nu),
                                              _nt(nt){}
    virtual ~NonlinearDynamics(){}
    
     
    virtual std::tuple<MatrixXd, MatrixXd, VectorXd, VectorXd> linearize_timestamp(const VectorXd& x, 
                                                                                    double sig, 
                                                                                    const MatrixXd& Ak, 
                                                                                    const MatrixXd& Sigk){}
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
    virtual std::tuple<MatrixXd, MatrixXd, MatrixXd, MatrixXd> linearize(const MatrixXd& x, 
                                                                        double sig, 
                                                                        const MatrixXd& Ak, 
                                                                        const MatrixXd& Sigk){}
public:
  EigenWrapper _ei;
private:
  int _nx, _nu, _nt;
    
};

}// namespace vimp