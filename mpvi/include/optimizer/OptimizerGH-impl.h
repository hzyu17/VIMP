#include "OptimizerGH.h"

namespace vimp{
    /**
     * @brief Function which computes one step of update.
     * 
     * @ return: New cost after one step.
     */
    template <typename FactorizedOptimizer>
    void VIMPOptimizerGH<FactorizedOptimizer>::step(){
        VectorXd Vdmu{VectorXd::Zero(this->_dim)};
        MatrixXd Vddmu{MatrixXd::Zero(this->_dim, this->_dim)};

        MatrixXd Sigma{this->_inverser.inverse(this->_precision)};

        double cost = this->cost_value();

        for (int k=0; k<_nsub_vars; k++){

            this->_vec_factor_optimizers[k]->update_mu(VectorXd{this->_mu});
            this->_vec_factor_optimizers[k]->update_precision_from_joint_covariance(MatrixXd{Sigma});
            this->_vec_factor_optimizers[k]->calculate_partial_V();

            Vdmu = Vdmu + this->_vec_factor_optimizers[k]->joint_Vdmu();
            Vddmu = Vddmu + this->_vec_factor_optimizers[k]->joint_Vddmu();

        }

        MatrixXd dprecision = -this->_precision + Vddmu;
        MatrixXd new_precision = this->_precision + this->_step_size_precision * dprecision;

        VectorXd dmu = -this->_precision.colPivHouseholderQr().solve(Vdmu);
        VectorXd new_mu = this->_mu + this->_step_size_mu * dmu;

        double step_size = this->_step_size_mu;
        while(cost < this->cost_value(new_mu, new_precision)){
            cout << "cost " << this->cost_value(new_mu, new_precision) << endl;

            VectorXd purturbed_mu{this->purturb_mean(0.01)};
            MatrixXd purturbed_precision{this->purturb_precision(0.01)};

            cout << "purturbed_precision " << endl << purturbed_precision << endl;
            
            cout << "purturbed cost " << this->cost_value(purturbed_mu, purturbed_precision) << endl;

            step_size = step_size / 1.1;
            set_step_size(step_size, step_size);
            new_precision = this->_precision + this->_step_size_precision * dprecision;
            new_mu = this->_mu + this->_step_size_mu * dmu;
        }
        
        this->set_mu(new_mu);
        this->set_precision(new_precision);
        // double step_size = this->_step_size_mu * 1.1;
        this->set_step_size(step_size, step_size);

        cout << "step size" << endl << _step_size_mu << endl;

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
        double step_size = 0.01;
        set_step_size(step_size, step_size);

        for (int i = 0; i < _niters; i++) {
            /// Collect the results
            this->_res_recorder.update_data(VectorXd{mean()}, MatrixXd{covariance()});
            cout << "iteration: " << i << ", cost: " << this->cost_value() << endl;
            step();
            // step_size = step_size / pow((i + 1), 1 / 3);
            // set_step_size(step_size, step_size);
        }

        // /// print 5 iteration datas 
        // vector<int> iters{int(_niters/5), int(_niters*2/5), int(_niters*3/5), int(_niters*4/5), _niters-1};
        // print_series_results(iters);

        save_data();
    } 

    /**
     * @brief Compute the total cost function value given a state.
     * 
     * @return cost value.
     */
    template <typename FactorizedOptimizer>
    double VIMPOptimizerGH<FactorizedOptimizer>::cost_value(const VectorXd& x, const MatrixXd& Cov){
        assert(_dim == x.size());
        assert(_dim == Cov.rows());
        assert(_dim == Cov.cols());

        double value = 0.0;
        for (auto& opt_k : this->_vec_factor_optimizers){
            VectorXd x_k = opt_k->Pk() * x;
            MatrixXd Cov_k = opt_k->Pk() * Cov * opt_k->Pk().transpose().eval();
            value += opt_k->cost_value(x_k, Cov_k);
        }
        return value;
    }

    /**
     * @brief Compute the total cost function value given a state, using current values.
     * 
     * @return cost value.
     */
    template <typename FactorizedOptimizer>
    double VIMPOptimizerGH<FactorizedOptimizer>::cost_value(){
        double value = 0.0;
        for (auto& opt_k : this->_vec_factor_optimizers){
            value += opt_k->cost_value();
        }
        return value;
    } 


}
