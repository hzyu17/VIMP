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
            /// Collect the results
            VectorXd mean_iter{mean()};
            MatrixXd cov_iter{covariance()};
            MatrixXd precision_iter{_precision};
            double cost_iter = cost_value();

            cout << "cost " << endl << cost_iter << endl;

            VectorXd fact_costs_iter = factor_costs();

            _res_recorder.update_data(mean_iter, cov_iter, precision_iter, cost_iter, fact_costs_iter);

            // one step
            VectorXd Vdmu{VectorXd::Zero(_dim)};
            MatrixXd Vddmu{MatrixXd::Zero(_dim, _dim)};

            for (auto& opt_k : _vec_factor_optimizers){
                opt_k->calculate_partial_V();
                // Vdmu = Vdmu + opt_k->joint_Vdmu();
                // Vddmu = Vddmu + opt_k->joint_Vddmu();

                Vdmu = Vdmu + opt_k->joint_Vdmu_sp();
                Vddmu = Vddmu + opt_k->joint_Vddmu_sp();

            }

            MatrixXd dprecision = -_precision + Vddmu;
            VectorXd dmu = Vddmu.colPivHouseholderQr().solve(-Vdmu);

            int cnt = 0;
            const int MAX_ITER = 20;

            int B = 1;
            double step_size_mu;
            double step_size_precision;

            MatrixXd new_precision;
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

        save_data();

        /// see a purturbed cost
        double scale = 0.001;
        cout << "=== final cost ===" << endl << new_cost << endl;
        save_matrix("data/2d_pR/final_cost.csv", MatrixXd::Constant(1, 1, new_cost));
        // purturbation_stat(scale, 100);
        
        /// print 5 iteration datas 
        // vector<int> iters{int(_niters/5), int(_niters*2/5), int(_niters*3/5), int(_niters*4/5), _niters-1};
        // print_series_results(iters);

    } 


    /**
     * @brief Verifier function which computes one step update in the Gaussian cost case, 
     * which has the closed form solution.
     */
    template <typename FactorizedOptimizer>
    void VIMPOptimizerGH<FactorizedOptimizer>::step_closed_form(){

        VectorXd Vdmu{VectorXd::Zero(_dim)};
        MatrixXd Vddmu{MatrixXd::Zero(_dim, _dim)};

        MatrixXd Sigma{_precision.inverse()};

        for (int k=0; k<_nsub_vars; k++){

            _vec_factor_optimizers[k]->update_mu_from_joint_mean(_mu);
            // _vec_factor_optimizers[k]->update_precision_from_joint_covariance(Sigma);
            _vec_factor_optimizers[k]->update_precision_from_joint(Sigma);

            _vec_factor_optimizers[k]->calculate_exact_partial_V();

            // Vdmu = Vdmu + _vec_factor_optimizers[k]->joint_Vdmu();
            // Vddmu = Vddmu + _vec_factor_optimizers[k]->joint_Vddmu();

            Vdmu = Vdmu + _vec_factor_optimizers[k]->joint_Vdmu_sp();
            Vddmu = Vddmu + _vec_factor_optimizers[k]->joint_Vddmu_sp();
        }

        MatrixXd _dprecision = -_precision + Vddmu;

        _precision = _precision + _step_size_precision*_dprecision;

        VectorXd dmu = _precision.colPivHouseholderQr().solve(-Vdmu);
        _mu = _mu + _step_size_mu * dmu;

    }


    template <typename FactorizedOptimizer>
    inline void VIMPOptimizerGH<FactorizedOptimizer>::set_precision(const MatrixXd& new_precision){
        assert(new_precision.size() == _precision.size());
        _precision = new_precision;
        // sparse inverse
        covariance_sp();
        _covariance = _covariance_sp;

        // dense inverse
        // _covariance = _inverser.inverse(_precision);

        for (auto & opt_fact : _vec_factor_optimizers){
            // opt_fact->update_precision_from_joint_covariance(_covariance);

            SpMat cov_sp = _covariance.sparseView();
            opt_fact->update_precision_from_joint(cov_sp);
        }
    }


    /**
     * @brief Compute the costs of all factors for a given mean and cov.
     */
    template <typename FactorizedOptimizer>
    VectorXd VIMPOptimizerGH<FactorizedOptimizer>::factor_costs(const VectorXd& x, const MatrixXd& Precision) const{
        VectorXd fac_costs{VectorXd::Zero(n_sub_factors())};
        int cnt = 0;
        MatrixXd Cov{Precision.inverse()};
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
     * 
     * @return cost value.
     */
    template <typename FactorizedOptimizer>
    double VIMPOptimizerGH<FactorizedOptimizer>::cost_value(const VectorXd& x, const MatrixXd& Precision) const{
        assert(dim() == x.size());
        assert(dim() == Precision.rows());
        assert(dim() == Precision.cols());

        MatrixXd Cov = Precision.inverse();
        // Cov = _inverser.inverse(Precision);

        double value = 0.0;
        for (auto& opt_k : _vec_factor_optimizers){
            // VectorXd x_k = opt_k->Pk() * x;
            // MatrixXd Cov_k = opt_k->Pk() * Cov * opt_k->Pk().transpose().eval();

            VectorXd x_k = opt_k->extract_mu_from_joint(x);
            MatrixXd Cov_k = opt_k->extract_cov_from_joint(Cov);

            value += opt_k->cost_value(x_k, Cov_k);
        }
        // MatrixXd Precision{Cov.inverse()};
        double logdetprec = log(Precision.determinant());
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
        
        double prec_det = _precision.determinant();
        double logdetprec = log(_precision.determinant());
        if (-99999 >  logdetprec){
            cout << "_precision " << endl << _precision << endl;
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
