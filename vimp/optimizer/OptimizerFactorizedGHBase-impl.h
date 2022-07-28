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

        // cout << "================= factor level calculate_partial_V =================" << endl;

        // cout << " --- mu --- " << endl << _mu.transpose() << endl;
        // cout << " --- precision --- " << endl << _precision << endl;
        // cout << " --- covariance --- " << endl << _covariance << endl;
        
        // _Vdmu.setZero();
        // _Vddmu.setZero();

        // use local variables

        // // GH approximation
        // update_covariance();
        // update the mu and sigma inside the gauss-hermite integrator
        updateGH();

        assert((_gauss_hermite.mean()-_mu).norm() < 1e-10);
        assert((_gauss_hermite.cov()-_covariance).norm() < 1e-10);

        /// Integrate for E_q{_Vdmu} 
        VectorXd Vdmu{VectorXd::Zero(_dim)};
        _gauss_hermite.update_integrand(_func_Vmu);
        Vdmu = _gauss_hermite.Integrate();

        // cout << "--- Vdmu ---" << endl << Vdmu.transpose() << endl;

        // cout << "--- _precision * Vdmu ---" << endl << (_precision * Vdmu).transpose() << endl;

        Vdmu = _precision * Vdmu;

        // cout << "--- new Vdmu --- " << endl << Vdmu << endl;

        /// Integrate for E_q{phi(x)}
        _gauss_hermite.update_integrand(_func_phi);
        double E_phi = _gauss_hermite.Integrate()(0, 0);

        // cout << "--- E_phi --- " << endl << E_phi << endl;
        
        /// Integrate for partial V^2 / ddmu_ 
        _gauss_hermite.update_integrand(_func_Vmumu);
        MatrixXd E_xxphi{_gauss_hermite.Integrate()};

        // cout << "--- Vddmu = E_{q_k}{(x_k - mu_k)(x_k - mu_k)^T * phi(x_k)} ---" << endl << E_xxphi << endl;

        MatrixXd Vddmu{MatrixXd::Zero(_dim, _dim)};
        Vddmu.triangularView<Upper>() = (_precision * E_xxphi * _precision - _precision * E_phi).triangularView<Upper>();
        Vddmu.triangularView<StrictlyLower>() = Vddmu.triangularView<StrictlyUpper>().transpose();

        // cout << "--- Vddmu ---" << endl << Vddmu << endl;

        // Vddmu.triangularView<Upper>() = (Vddmu - _precision * E_phi).triangularView<Upper>();
        // Vddmu.triangularView<StrictlyLower>() = Vddmu.triangularView<StrictlyUpper>().transpose();

        // update member variables
        _Vdmu = Vdmu;
        _Vddmu = Vddmu;
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
        // update_covariance();

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
        double E_Phi = _gauss_hermite.Integrate()(0, 0);
        if (isinf(E_Phi)){
            cout << "--- mu ---" << endl << mean() << endl;
            cout << "--- precision ---" << endl << _precision << endl;
            cout << "--- covariance ---" << endl << covariance() << endl;
            throw std::runtime_error(string("Infinity expectations ..."));
        }
        return E_Phi;
    }

    /**
     * @brief Compute the cost function. V(x) = E_q(\phi(x)) using the current values.
     */
    double VIMPOptimizerFactorizedBase::cost_value() {
        updateGH();
        _gauss_hermite.update_integrand(_func_phi);
        double E_Phi = _gauss_hermite.Integrate()(0, 0);
        if (isinf(E_Phi)){
            cout << "--- mu ---" << endl << mean() << endl;
            cout << "--- precision ---" << endl << _precision << endl;
            cout << "--- covariance ---" << endl << covariance() << endl;
            throw std::runtime_error(string("Infinity expectations ..."));
        }
        return E_Phi;
    }

}