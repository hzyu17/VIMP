/**
 * @brief OptimizerFactorizedGHBase-impl.h
 * 
 * Author: Hongzhe Yu
 * 
 */

using namespace Eigen;
namespace vimp{
        
    /**
     * @brief Main function calculating phi * (partial V) / (partial mu), and 
     * phi * (partial V^2) / (partial mu * partial mu^T)
     * 
     * @return * void 
     */
    void VIMPOptimizerFactorizedBase::calculate_partial_V(){
        
        _Vdmu.setZero();
        _Vddmu.setZero();

        // GH approximation
        update_covariance();
        updateGH();

        /// Integrate for _Vdmu 
        _gauss_hermite.update_integrand(_func_Vmu);

        _Vdmu = _gauss_hermite.Integrate();
        _Vdmu = _precision * _Vdmu;

        /// Integrate for phi(x)
        _gauss_hermite.update_integrand(_func_phi);
        double avg_phi = _gauss_hermite.Integrate()(0, 0);

        /// Integrate for partial V^2 / ddmu_ 
        _gauss_hermite.update_integrand(_func_Vmumu);
        _Vddmu = _gauss_hermite.Integrate();

        _Vddmu.triangularView<Upper>() = (_precision * _Vddmu * _precision).triangularView<Upper>();
        _Vddmu.triangularView<StrictlyLower>() = _Vddmu.triangularView<StrictlyUpper>().transpose();

        _Vddmu.triangularView<Upper>() = (_Vddmu - _precision * avg_phi).triangularView<Upper>();
        _Vddmu.triangularView<StrictlyLower>() = _Vddmu.triangularView<StrictlyUpper>().transpose();
    }


    /**
     * @brief Main function calculating phi * (partial V) / (partial mu), and 
     * phi * (partial V^2) / (partial mu * partial mu^T) for Gaussian posterior: closed-form expression
     * 
     * @param mu_t target mean vector
     * @param covariance_t target covariance matrix
     */
    void VIMPOptimizerFactorizedBase::calculate_exact_partial_V(VectorXd mu_t, MatrixXd covariance_t){
        _Vdmu.setZero();
        _Vddmu.setZero();
        update_covariance();

        // helper vectors
        VectorXd eps{_mu - mu_t};
        MatrixXd tmp{MatrixXd::Zero(_dim, _dim)};
        MatrixXd precision_t{covariance_t.inverse()};

        // partial V / partial mu
        _Vdmu = precision_t * eps;

        // partial V^2 / partial mu*mu^T
        // update tmp matrix
        for (int i=0; i<(_dim); i++){
            for (int j=0; j<(_dim); j++) {
                for (int k=0; k<(_dim); k++){
                    for (int l=0; l<(_dim); l++){
                        tmp(i, j) += (_covariance(i, j) * (_covariance(k, l)) + _covariance(i,k) * (_covariance(j,l)) + _covariance(i,l)*_covariance(j,k))*precision_t(k,l);
                    }
                }
            }
        }

        _Vddmu = _precision * tmp * _precision - _precision * (precision_t*_covariance).trace();
        _Vddmu = _Vddmu / 2;

    }

    /**
     * @brief One step in the optimization.
     * @return true: success.
     */
    bool VIMPOptimizerFactorizedBase::step(){

        /// Zero grad
        VectorXd dmu{VectorXd::Zero(_dim)};
        MatrixXd dprecision{MatrixXd::Zero(_dim, _dim)};

        calculate_partial_V();
    //    calculate_exact_partial_V(_cost_class.get_mean(), _cost_class.get_covariance());

        dprecision = -_precision + _Vddmu;

        /// without backtracking
        _precision = _precision + _step_size_Sigma * dprecision;
        dmu = _precision.colPivHouseholderQr().solve(-_Vdmu);
        _mu = _mu + _step_size_mu * dmu;

        return true;

    }

    /**
     * @brief Compute the cost function. V(x) = E_q(\phi(x))
     */
    double VIMPOptimizerFactorizedBase::cost_value(const VectorXd& x, const MatrixXd& Cov) {
        assert(_dim == x.size());
        assert(_dim == Cov.rows());
        assert(_dim == Cov.cols());

        updateGH(x, Cov);
        _gauss_hermite.update_integrand(_func_phi);

        return _gauss_hermite.Integrate()(0, 0);
    }

    /**
     * @brief Compute the cost function. V(x) = E_q(\phi(x)) using the current values.
     */
    double VIMPOptimizerFactorizedBase::cost_value() {
        updateGH();
        _gauss_hermite.update_integrand(_func_phi);
        return _gauss_hermite.Integrate()(0, 0);
    }

}