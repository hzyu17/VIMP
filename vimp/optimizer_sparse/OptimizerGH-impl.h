#include <Eigen/Eigenvalues> 
using namespace Eigen;
using namespace std;
#include <stdexcept>

namespace vimp{

    // /**
    //  * @brief One step in the update, without backtracking.
    //  * 
    //  * @ return: New cost after one step.
    //  */
    // template <typename FactorizedOptimizer>
    // void VIMPOptimizerGH<FactorizedOptimizer>::step(){

    //     // cout << "_mu " << _mu.transpose() << endl;

    //     cout << "cost " << cost_value() << endl;

    //     assert(abs(cost_value() - cost_value(_mu, covariance())) < 1e-10);

    //     VectorXd Vdmu{VectorXd::Zero(_dim)};
    //     MatrixXd Vddmu{MatrixXd::Zero(_dim, _dim)};


    //     for (auto& opt_k : _vec_factor_optimizers){
    //         opt_k->calculate_partial_V();

    //         Vdmu = Vdmu + opt_k->joint_Vdmu();
    //         Vddmu = Vddmu + opt_k->joint_Vddmu();
    //     }
        
    //     MatrixXd dprecision = -_precision + Vddmu;
    //     MatrixXd new_precision = _precision + _step_size_precision * dprecision;
    //     // MatrixXd new_precision = Vddmu;

    //     VectorXd dmu = _precision.colPivHouseholderQr().solve(-Vdmu);
    //     VectorXd new_mu  = _mu + _step_size_mu * dmu;

    //     set_mu(new_mu);
    //     set_precision(new_precision);

    // }


      // /**
    //  * @brief optimize without backtracking
    //  * 
    //  */
    // template <typename FactorizedOptimizer>
    // void VIMPOptimizerGH<FactorizedOptimizer>::optimize(){
        
    //     for (int i_iter = 0; i_iter < _niters; i_iter++) {
    //         double step_size = _step_size_mu;
    //         step_size = step_size / pow((i_iter + 1), 1 / 3);
    //         set_step_size(step_size, step_size);

    //         /// Collect the results
    //         VectorXd mean_iter{mean()};
    //         MatrixXd cov_iter{covariance()};
    //         MatrixXd precision_iter{_precision};
    //         double cost_iter = cost_value();

    //         cout << "iteration: " << i_iter << endl;
    //         this->_res_recorder.update_data(mean_iter, cov_iter, precision_iter, cost_iter);
    //         step();
    //     }

    //     /// print 5 iteration datas 
    //     vector<int> iters{int(_niters/5), int(_niters*2/5), int(_niters*3/5), int(_niters*4/5), _niters-1};
    //     print_series_results(iters);

    //     save_data();
    // } 


    /**
     * @brief optimize with backtracking
     */
    template <typename FactorizedOptimizer>
    void VIMPOptimizerGH<FactorizedOptimizer>::optimize(){
        double new_cost = 0.0;
        for (int i_iter = 0; i_iter < _niters; i_iter++) {
            cout << "========= iter " << i_iter << " ========= "<< endl;

            /// Collect the results
            VectorXd mean_iter{mean()};
            MatrixXd cov_iter{covariance()};
            MatrixXd precision_iter{_precision};
            double cost_iter = cost_value();

            /// save matrices for debugging ...
            // save_vector("/home/hongzhe/git/VIMP/vimp/data/debug/mean_" + to_string(i_iter)+".csv", mean());
            // // save_matrix("data/debug/j_Vddmu_"+to_string(i_iter)+".csv", Vddmu);
            // save_matrix("/home/hongzhe/git/VIMP/vimp/data/debug/precision_" + to_string(i_iter) + ".csv", _precision);
            // save_matrix("/home/hongzhe/git/VIMP/vimp/data/debug/cov_" + to_string(i_iter) + ".csv", covariance());
            // save_matrix("/home/hongzhe/git/VIMP/vimp/data/debug/cost_" + to_string(i_iter)+".csv", MatrixXd::Constant(1,1,cost_iter));
            
            // cout << "covariance() - precision.inverse()" << endl << (_precision.inverse() - covariance()).norm() << endl;
            // new_cost = cost_value(_mu, covariance());
            // cout << "diff " << endl << abs(cost_iter - new_cost) << endl;
            // assert(abs(cost_iter - new_cost) < 1e-5);

            // cout << "mean " << endl << mean().transpose() << endl;
            cout << "cost " << endl << cost_iter << endl;

            VectorXd fact_costs_iter = factor_costs();

            _res_recorder.update_data(mean_iter, cov_iter, precision_iter, cost_iter, fact_costs_iter);

            // one step
            VectorXd Vdmu{VectorXd::Zero(_dim)};
            MatrixXd Vddmu{MatrixXd::Zero(_dim, _dim)};

            for (auto& opt_k : _vec_factor_optimizers){
                opt_k->calculate_partial_V();
                Vdmu = Vdmu + opt_k->joint_Vdmu();
                Vddmu = Vddmu + opt_k->joint_Vddmu();
            }

            MatrixXd dprecision = -_precision + Vddmu;
            VectorXd dmu = Vddmu.colPivHouseholderQr().solve(-Vdmu);

            // save_matrix("/home/hongzhe/git/VIMP/vimp/data/debug/dprecision_" + to_string(i_iter) + ".csv", dprecision);

            // cout << "mean " << endl << _mu(24) << ", " << _mu(25) << endl;
            // cout << "dmu for the collided state " << endl << dmu(24) << ", " << dmu(25) << endl;

            // backtracking
            
            // MatrixXd new_precision = _precision + _step_size_base_precision * dprecision;
            // VectorXd new_mu  = _mu + _step_size_base_mu * dmu;
            // new_cost = cost_value(new_mu, new_precision.inverse());

            // // project the precision into psd cone
            // Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(new_precision);

            // VectorXcd eigvals = new_precision.eigenvalues();
            // // cout << "eigen values " << endl << new_precision.eigenvalues().real() << endl;
            // cout << "min eigen values " << endl << new_precision.eigenvalues().real().minCoeff() << endl;
            // cout << "det(precision) " << endl << new_precision.determinant() << endl;

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

                // Checking the psd of Sigma^{-1}
                // VectorXcd eigvals = new_precision.eigenvalues();
                // if (eigvals.real().minCoeff() > 0){
                //     new_cost = cost_value(new_mu, new_precision);
                //     if (new_cost < cost_iter){ break; }
                // }

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

        MatrixXd Sigma{_inverser.inverse(_precision)};

        for (int k=0; k<_nsub_vars; k++){

            _vec_factor_optimizers[k]->update_mu_from_joint_mean(_mu);
            _vec_factor_optimizers[k]->update_precision_from_joint_covariance(Sigma);

            _vec_factor_optimizers[k]->calculate_exact_partial_V();

            Vdmu = Vdmu + _vec_factor_optimizers[k]->joint_Vdmu();
            Vddmu = Vddmu + _vec_factor_optimizers[k]->joint_Vddmu();
        }

        MatrixXd _dprecision = -_precision + Vddmu;

        _precision = _precision + _step_size_precision*_dprecision;

        VectorXd dmu = _precision.colPivHouseholderQr().solve(-Vdmu);
        _mu = _mu + _step_size_mu * dmu;

    }


    /**
     * @brief Compute the costs of all factors for a given mean and cov.
     */
    template <typename FactorizedOptimizer>
    VectorXd VIMPOptimizerGH<FactorizedOptimizer>::factor_costs(const VectorXd& x, const MatrixXd& Precision) const{
        VectorXd fac_costs{VectorXd::Zero(n_sub_factors())};
        int cnt = 0;
        MatrixXd Cov{_inverser.inverse(Precision)};
        for (auto& opt_k : _vec_factor_optimizers){
            VectorXd x_k = opt_k->Pk() * x;
            MatrixXd Cov_k = opt_k->Pk() * Cov * opt_k->Pk().transpose().eval();
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

        MatrixXd Cov;
        Cov = _inverser.inverse(Precision);

        double value = 0.0;
        for (auto& opt_k : _vec_factor_optimizers){
            VectorXd x_k = opt_k->Pk() * x;
            MatrixXd Cov_k = opt_k->Pk() * Cov * opt_k->Pk().transpose().eval();
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
