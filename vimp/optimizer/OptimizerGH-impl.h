#include <Eigen/Eigenvalues> 
using namespace Eigen;
using namespace std;
#include <stdexcept>

namespace vimp{

    /**
     * @brief optimize with backtracking
     */
    template <typename FactorizedOptimizer>
    void VIMPOptimizerGH<FactorizedOptimizer>::optimize(){
        double new_cost = 0.0;
        for (int i_iter = 0; i_iter < _niters; i_iter++) {
            cout << "========= iter " << i_iter << " ========= "<< endl;
            _timer.start();
            /// ============= Collect results ============= 
            // VectorXd mean_iter{mean()};
            // SpMat cov_iter{covariance()};
            // SpMat precision_iter{_precision};
            // VectorXd fact_costs_iter = factor_costs();
            // _res_recorder.update_data(mean_iter, cov_iter, precision_iter, cost_iter, fact_costs_iter);

            double cost_iter = cost_value();

            cout << "cost " << endl << cost_iter << endl;

            // one step
            SpMat Vdmu(_dim, _dim);
            SpMat Vddmu(_dim, _dim);

            for (auto& opt_k : _vec_factor_optimizers){
                opt_k->calculate_partial_V();
                Vdmu = Vdmu + opt_k->joint_Vdmu_sp();
                Vddmu = Vddmu + opt_k->joint_Vddmu_sp();

            }

            SpMat dprecision = -_precision + Vddmu;
            VectorXd dmu = _eigen_wrapper.solve_cgd_sp(Vddmu, -Vdmu);
            // VectorXd dmu = Vddmu.colPivHouseholderQr().solve(-Vdmu);

            int cnt = 0;
            const int MAX_ITER = 20;

            int B = 1;
            double step_size_mu;
            double step_size_precision;

            SpMat new_precision;
            VectorXd new_mu;

            while (true){
                
                step_size_mu = pow(_step_size_base_mu, B);
                step_size_precision = pow(_step_size_base_precision, B);

                new_precision = _precision + step_size_precision * dprecision;
                new_mu  = _mu + step_size_mu * dmu;

                new_cost = cost_value(new_mu, new_precision);
                if (new_cost < cost_iter){ break; }

                cnt += 1;
                if (cnt > MAX_ITER){
                    // throw std::runtime_error(std::string("Too many iterations in the backtracking ... Dead"));
                    cout << "Too many iterations in the backtracking ... Dead" << endl;
                    break;
                }

                B += 1;
            }

            cout << "step size " << endl << step_size_mu << endl;

            /// update the variables
            set_mu(new_mu);
            set_precision(new_precision);

            /// small cost decrease, stop iterations. 
            double STOP_SIGN = 1e-5;
            if (cost_iter - new_cost < STOP_SIGN){
                cout << "--- Cost Decrease less than threshold ---" << endl << cost_iter - new_cost << endl;
                cout << "--- number of backtrackings ---" << endl << B << endl;
                break;
            }
        _timer.end();
        }

        // save_data();

        /// see a purturbed cost
        double scale = 0.001;
        cout << "=== final cost ===" << endl << new_cost << endl;
        // save_matrix("data/2d_pR/final_cost.csv", MatrixXd::Constant(1, 1, new_cost));

    } 


    /**
     * @brief Verifier function which computes one step update in the Gaussian cost case, 
     * which has the closed form solution.
     */
    template <typename FactorizedOptimizer>
    void VIMPOptimizerGH<FactorizedOptimizer>::step_closed_form(){

        VectorXd Vdmu{VectorXd::Zero(_dim)};
        SpMat Vddmu{MatrixXd::Zero(_dim, _dim)};

        SpMat Sigma{inverse(_precision)};

        for (int k=0; k<_nfactors; k++){

            _vec_factor_optimizers[k]->update_mu_from_joint(_mu);
            _vec_factor_optimizers[k]->update_precision_from_joint(Sigma);

            _vec_factor_optimizers[k]->calculate_exact_partial_V();

            Vdmu = Vdmu + _vec_factor_optimizers[k]->joint_Vdmu_sp();
            Vddmu = Vddmu + _vec_factor_optimizers[k]->joint_Vddmu_sp();
        }

        SpMat _dprecision = -_precision + Vddmu;

        _precision = _precision + _step_size_precision*_dprecision;

        VectorXd dmu = _eigen_wrapper.solve_cgd_sp(_precision, -Vdmu);
        _mu = _mu + _step_size_mu * dmu;

    }


    template <typename FactorizedOptimizer>
    inline void VIMPOptimizerGH<FactorizedOptimizer>::set_precision(const SpMat& new_precision){
        _precision = new_precision;
        // sparse inverse
        inverse_inplace();

        for (auto & opt_fact : _vec_factor_optimizers){
            opt_fact->update_precision_from_joint(_covariance);
        }
    }


    /**
     * @brief Compute the costs of all factors for a given mean and cov.
     */
    template <typename FactorizedOptimizer>
    VectorXd VIMPOptimizerGH<FactorizedOptimizer>::factor_costs(const VectorXd& x, const SpMat& Precision) const{
        VectorXd fac_costs{VectorXd::Zero(n_sub_factors())};
        int cnt = 0;
        SpMat Cov = inverse(Precision);
        for (auto& opt_k : _vec_factor_optimizers){
            VectorXd x_k = opt_k->extract_mu_from_joint(x);

            MatrixXd Cov_k = opt_k->extract_cov_from_joint(Cov);
            fac_costs(cnt) = opt_k->cost_value(x_k, Cov_k);
            cnt += 1;
        }
        return fac_costs;
    }


    /**
     * @brief Compute the costs of all factors, using current values.
     */
    template <typename FactorizedOptimizer>
    VectorXd VIMPOptimizerGH<FactorizedOptimizer>::factor_costs() const{
        VectorXd fac_costs{VectorXd::Zero(n_sub_factors())};
        int cnt = 0;
        for (auto& opt_k : _vec_factor_optimizers){
            fac_costs(cnt) = opt_k->cost_value();
            cnt += 1;
        }
        return fac_costs;
    }

    /**
     * @brief Compute the total cost function value given a state.
     * @return cost value.
     */
    template <typename FactorizedOptimizer>
    double VIMPOptimizerGH<FactorizedOptimizer>::cost_value(const VectorXd& x, SpMat& Precision){
        
        SpMat Cov = inverse(Precision);
        // Cov = _inverser.inverse(Precision);

        double value = 0.0;
        for (auto& opt_k : _vec_factor_optimizers){
            VectorXd x_k = opt_k->extract_mu_from_joint(x);
            MatrixXd Cov_k = opt_k->extract_cov_from_joint(Cov);

            value += opt_k->cost_value(x_k, Cov_k);
        }
        SparseLDLT ldlt(Precision);
        double logdetprec = log(ldlt.determinant());
        if (-9999.9 > logdetprec){
            // cout << "precision " << endl << Precision << endl;
            std::runtime_error(std::string("Infinity log determinant precision matrix ..."));
        }
        value += logdetprec / 2;
        return value;
    }

    /**
     * @brief Compute the total cost function value given a state, using current values.
     * @return cost value.
     */
    template <typename FactorizedOptimizer>
    double VIMPOptimizerGH<FactorizedOptimizer>::cost_value() const{
        
        double logdetprec = log(_ldlt.determinant());
        if (-99999 >  logdetprec){
            cout << "logdetprec " << endl << logdetprec << endl;
            std::runtime_error(std::string("Infinity log determinant precision matrix ..."));
        }
        double value = cost_value_no_entropy() + logdetprec / 2;
        return value;
    }

    /**
     * @brief given a state, compute the total cost function value without the entropy term, using current values.
     * @return cost value.
     */
    template <typename FactorizedOptimizer>
    double VIMPOptimizerGH<FactorizedOptimizer>::cost_value_no_entropy() const{
        double value = 0.0;
        for (auto& opt_k : _vec_factor_optimizers){
            value += opt_k->cost_value();
        }
        return value;
    }


}
