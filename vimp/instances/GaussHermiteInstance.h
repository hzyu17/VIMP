/**
 * @file GaussHermiteInstance.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Instance of the Gauss-Hermite quadrature, for a specific function type.
 * @version 0.1
 * @date 2022-07-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <functional>
#include "gvimp/GaussHermite.h"
using namespace Eigen;

namespace vimp{

    typedef std::function<MatrixXd(const VectorXd& x)> FunctionMatrixVector;

    typedef GaussHermite<FunctionMatrixVector> GaussHermiteInstance;

}
