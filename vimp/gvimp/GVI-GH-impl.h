using namespace Eigen;
using namespace std;
#include <stdexcept>

namespace vimp{
    
    /**
     * @brief optimize with backtracking
     */
    template <typename FactorizedOptimizer>
    void GVIGH<FactorizedOptimizer>::optimize(){
        double new_cost = 0.0;
        double cost_iter =  0.0;

        // for (auto& opt_k : _vec_factors){
        //     opt_k->test_integration();
        // }
        for (int i_iter = 0; i_iter < _niters; i_iter++) {

            // if (i_iter == _niters_lowtemp){
            //     cout << "*************** Entering High temperature *************** "<< endl;
            //     set_temperature(_high_temperature);

            // }
            cout << "========= iteration " << i_iter << " ========= "<< endl;
            // ============= Collect results ============= 
            VectorXd fact_costs_iter = factor_costs();

            cost_iter = cost_value();
            std::cout << "cost_iter " << std::endl << cost_iter << std::endl;

            _res_recorder.update_data(_mu, _covariance, _precision, cost_iter, fact_costs_iter);
            // one step
            _Vdmu.setZero();
            _Vddmu.setZero();

            for (auto& opt_k : _vec_factors){
                opt_k->calculate_partial_V_GH();
                _Vdmu = _Vdmu + opt_k->joint_Vdmu_sp();
                _Vddmu = _Vddmu + opt_k->joint_Vddmu_sp();
            }

            // _Vdmu = _Vdmu ; // / _temperature;
            // _Vddmu = _Vddmu ; // / _temperature;
            
            SpMat dprecision = -_precision + _Vddmu;
            VectorXd dmu = _ei.solve_cgd_sp(_Vddmu, -_Vdmu);

            _ei.print_matrix(dprecision, "dprecision");
            _ei.print_matrix(dmu, "dmu");

            int cnt = 0;
            int B = 1;
            // const int MAX_ITER = 20;
            double step_size;

            SpMat new_precision;
            VectorXd new_mu;
            
            step_size = pow(_step_size_base, B);
            new_mu  = _mu + step_size * dmu;
            new_precision = _precision + step_size * dprecision;
            
            new_cost = cost_value(new_mu, new_precision);

            while (new_cost > cost_iter){
                B += 1;
                step_size = pow(_step_size_base, B);
                new_mu  = _mu + step_size * dmu;
                new_precision = _precision + step_size * dprecision;
                new_cost = cost_value(new_mu, new_precision);

                cnt += 1;

                if (cnt > _niters_backtrack){
                    // throw std::runtime_error(std::string("Too many iterations in the backtracking ... Dead"));
                    cout << "Too many iterations in the backtracking ... Dead" << endl;
                    break;
                }
            }
            // cout << "step size " << endl << step_size << endl;

            /// update the variables
            set_mu(new_mu);
            set_precision(new_precision);
            B = 0;
            // if (cost_iter - new_cost < stop_error()){
            //     cout << "--- Cost Decrease less than threshold ---" << endl << cost_iter - new_cost << endl;
            //     save_data();
            //     /// see a purturbed cost
            //     cout << "=== final cost ===" << endl << cost_iter << endl;
            //     return ;
            // }
        }

        save_data();

        /// see a purturbed cost
        cout << "=== final cost ===" << endl << cost_iter << endl;
    } 


    template <typename FactorizedOptimizer>
    inline void GVIGH<FactorizedOptimizer>::set_precision(const SpMat& new_precision){
        _precision = new_precision;
        // sparse inverse
        inverse_inplace();

        for (auto & factor : _vec_factors){
            factor->update_precision_from_joint(_covariance);
        }
    }

    /**
     * @brief Compute the costs of all factors for a given mean and cov.
     */
    template <typename FactorizedOptimizer>
    VectorXd GVIGH<FactorizedOptimizer>::factor_costs(const VectorXd& x, const SpMat& Precision) const{
        VectorXd fac_costs(_nfactors);
        fac_costs.setZero();
        int cnt = 0;
        SpMat Cov = inverse(Precision);
        for (auto& opt_k : _vec_factors){
            VectorXd x_k = opt_k->extract_mu_from_joint(x);

            MatrixXd Cov_k = opt_k->extract_cov_from_joint(Cov);
            fac_costs(cnt) = opt_k->fact_cost_value(x_k, Cov_k) ; // / _temperature;
            cnt += 1;
        }
        return fac_costs;
    }


    /**
     * @brief Compute the costs of all factors, using current values.
     */
    template <typename FactorizedOptimizer>
    VectorXd GVIGH<FactorizedOptimizer>::factor_costs() const{
        VectorXd fac_costs(_nfactors);
        fac_costs.setZero();
        int cnt = 0;

        Timer timer;
        for (auto& opt_k : _vec_factors){
            fac_costs(cnt) = opt_k->fact_cost_value();
            cnt += 1;
        }
        return fac_costs  ; // / _temperature;
    }

    /**
     * @brief Compute the total cost function value given a state.
     */
    template <typename FactorizedOptimizer>
    double GVIGH<FactorizedOptimizer>::cost_value(const VectorXd& x, SpMat& Precision){
        
        SpMat Cov = inverse(Precision);

        double value = 0.0;
        for (auto& opt_k : _vec_factors){
            VectorXd x_k = opt_k->extract_mu_from_joint(x);
            MatrixXd Cov_k = opt_k->extract_cov_from_joint(Cov);

            value += opt_k->fact_cost_value(x_k, Cov_k) ; // / _temperature;
        }
        SparseLDLT ldlt(Precision);
        double det = ldlt.determinant();
        if (det < 0){ std::runtime_error("Infinity log determinant precision matrix ..."); }
        value += log(det) / 2;
        return value;
    }

    /**
     * @brief Compute the total cost function value given a state, using current values.
     */
    template <typename FactorizedOptimizer>
    double GVIGH<FactorizedOptimizer>::cost_value() {
        double det = _ldlt.determinant();
        if (0 >  det){
            this->save_data();
            std::__throw_out_of_range("precision matrix not psd ...");
        }
        std::cout << "log(det) " << std::endl << log(det) << std::endl;
        return cost_value_no_entropy() + log(det) / 2;
    }

    /**
     * @brief given a state, compute the total cost function value without the entropy term, using current values.
     */
    template <typename FactorizedOptimizer>
    double GVIGH<FactorizedOptimizer>::cost_value_no_entropy() const{
        double value = 0.0;
        for (auto& opt_k : _vec_factors){
            value += opt_k->fact_cost_value();
        }
        return value  ; // / _temperature;
    }

}
