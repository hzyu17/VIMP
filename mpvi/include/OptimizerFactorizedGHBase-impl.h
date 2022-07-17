#include "OptimizerFactorizedGHBase.h"

namespace vimp{
        
    /**
     * @brief Main function calculating phi * (partial V) / (partial mu), and 
     * phi * (partial V^2) / (partial mu * partial mu^T)
     * 
     * @return * void 
     */
    void VIMPOptimizerFactorizedBase::calculate_partial_V(){
        
        this->_Vdmu.setZero();
        this->_Vddmu.setZero();

        // GH approximation
        update_covariance();
        updateGH();

        /// Integrate for this->_Vdmu 
        this->_gauss_hermite.update_integrand(_func_Vmu);

        this->_Vdmu = this->_gauss_hermite.Integrate();
        this->_Vdmu = this->_precision * this->_Vdmu;

        /// Integrate for phi(x)
        this->_gauss_hermite.update_integrand(this->_func_phi);
        double avg_phi = this->_gauss_hermite.Integrate()(0, 0);

        /// Integrate for partial V^2 / ddmu_ 
        this->_gauss_hermite.update_integrand(this->_func_Vmumu);
        this->_Vddmu = this->_gauss_hermite.Integrate();

        this->_Vddmu.triangularView<Upper>() = (this->_precision * this->_Vddmu * this->_precision).triangularView<Upper>();
        this->_Vddmu.triangularView<StrictlyLower>() = this->_Vddmu.triangularView<StrictlyUpper>().transpose();

        this->_Vddmu.triangularView<Upper>() = (this->_Vddmu - this->_precision * avg_phi).triangularView<Upper>();
        this->_Vddmu.triangularView<StrictlyLower>() = this->_Vddmu.triangularView<StrictlyUpper>().transpose();
    }


    /**
     * @brief Main function calculating phi * (partial V) / (partial mu), and 
     * phi * (partial V^2) / (partial mu * partial mu^T) for Gaussian posterior: closed-form expression
     * 
     * @param mu_t target mean vector
     * @param covariance_t target covariance matrix
     */
    void VIMPOptimizerFactorizedBase::calculate_exact_partial_V(VectorXd mu_t, MatrixXd covariance_t){
        this->_Vdmu.setZero();
        this->_Vddmu.setZero();
        update_covariance();

        // helper vectors
        VectorXd eps{this->_mu - mu_t};
        MatrixXd tmp{MatrixXd::Zero(this->_dim, this->_dim)};
        MatrixXd precision_t{covariance_t.inverse()};

        // partial V / partial mu
        this->_Vdmu = precision_t * eps;

        // partial V^2 / partial mu*mu^T
        // update tmp matrix
        for (int i=0; i<this->_dim; i++){
            for (int j=0; j<this->_dim; j++) {
                for (int k=0; k<this->_dim; k++){
                    for (int l=0; l<this->_dim; l++){
                        tmp(i, j) += (this->_covariance(i, j)*this->_covariance(k, l) + this->_covariance(i,k)*this->_covariance(j,l) + this->_covariance(i,l)*this->_covariance(j,k))*precision_t(k,l);
                    }
                }
            }
        }

        this->_Vddmu = this->_precision * tmp * this->_precision - this->_precision * (precision_t*this->_covariance).trace();
        this->_Vddmu = this->_Vddmu / 2;

    }

    /**
     * @brief One step in the optimization.
     * @return true: success.
     */
    bool VIMPOptimizerFactorizedBase::step(){

        /// Zero grad
        this->_dmu.setZero();
        this->_dprecision.setZero();

        calculate_partial_V();
    //    calculate_exact_partial_V(_cost_class.get_mean(), _cost_class.get_covariance());

        this->_dprecision = -this->_precision + this->_Vddmu;

        /// without backtracking
        this->_precision = this->_precision + this->_step_size_Sigma * this->_dprecision;
        this->_dmu = this->_precision.colPivHouseholderQr().solve(-this->_Vdmu);
        this->_mu = this->_mu + this->_step_size_mu * this->_dmu;

        return true;

    }

}