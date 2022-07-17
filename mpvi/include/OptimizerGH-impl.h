#include "OptimizerGH.h"

namespace vimp{
    /**
     * @brief Function which computes one step of update.
     * 
     */
    template <typename FactorizedOptimizer>
    void VIMPOptimizerGH<FactorizedOptimizer>::step(){
        VectorXd Vdmu{VectorXd::Zero(this->_dim)};
        MatrixXd Vddmu{MatrixXd::Zero(this->_dim, this->_dim)};

        MatrixXd Sigma{this->_inverser.inverse(this->_precision)};

        for (int k=0; k<_nsub_vars; k++){

            this->_vec_factor_optimizers[k]->update_mu(VectorXd{this->_mu});
            this->_vec_factor_optimizers[k]->update_precision_from_joint_covariance(MatrixXd{Sigma});
            this->_vec_factor_optimizers[k]->calculate_partial_V();

            Vdmu = Vdmu + this->_vec_factor_optimizers[k]->joint_Vdmu();
            Vddmu = Vddmu + this->_vec_factor_optimizers[k]->joint_Vddmu();

        }

        MatrixXd dprecision = -this->_precision + Vddmu;
        this->_precision = this->_precision + this->_step_size_precision*dprecision;

        VectorXd dmu = -this->_precision.colPivHouseholderQr().solve(Vdmu);
        this->_mu = this->_mu + this->_step_size_mu * dmu;

    }

    /**
     * @brief Verifier function which computes one step update in the Gaussian cost case, 
     * which has the closed form solution.
     */
    template <typename FactorizedOptimizer>
    void VIMPOptimizerGH<FactorizedOptimizer>::step_closed_form(){

        VectorXd Vdmu{VectorXd::Zero(this->_dim)};
        MatrixXd Vddmu{MatrixXd::Zero(this->_dim, this->_dim)};

        MatrixXd Sigma{this->_inverser.inverse(this->_precision)};

        for (int k=0; k<_nsub_vars; k++){

            this->_vec_factor_optimizers[k]->update_mu(VectorXd{this->_mu});
            this->_vec_factor_optimizers[k]->update_precision_from_joint_covariance(MatrixXd{Sigma});

            this->_vec_factor_optimizers[k]->calculate_exact_partial_V();

            Vdmu = Vdmu + this->_vec_factor_optimizers[k]->joint_Vdmu();
            Vddmu = Vddmu + this->_vec_factor_optimizers[k]->joint_Vddmu();
        }

        MatrixXd _dprecision = -this->_precision + Vddmu;

        this->_precision = this->_precision + this->_step_size_precision*_dprecision;

        VectorXd dmu = this->_precision.colPivHouseholderQr().solve(-Vdmu);
        this->_mu = this->_mu + this->_step_size_mu * dmu;

    }

    /**
     * @brief The optimizing process.
     * 
     */
    template <typename FactorizedOptimizer>
    void VIMPOptimizerGH<FactorizedOptimizer>::optimize(){
        double step_size = 0.9;
        for (int i = 0; i < _niters; i++) {
            step_size = step_size / pow((i + 1), 1 / 3);
            set_step_size(step_size, step_size);

            /// Collect the results
            VectorXd mean_iter{mean()};
            MatrixXd cov_iter{covariance()};

            cout << "iteration: " << i << endl;
            this->_res_recorder.update_data(mean_iter, cov_iter);
            step();
        }

        /// print 5 iteration datas 
        vector<int> iters{int(_niters/5), int(_niters*2/5), int(_niters*3/5), int(_niters*4/5), _niters-1};
        print_series_results(iters);

        save_data();
    } 
    


}
