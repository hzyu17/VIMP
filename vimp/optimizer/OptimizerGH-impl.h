using namespace Eigen;
using namespace std;

namespace vimp{

    /**
     * @brief One step in the update, without line search.
     * 
     * @ return: New cost after one step.
     */
    template <typename FactorizedOptimizer>
    void VIMPOptimizerGH<FactorizedOptimizer>::step(){

        // cout << "_mu " << _mu.transpose() << endl;

        cout << "cost " << cost_value() << endl;

        assert(abs(cost_value() - cost_value(_mu, covariance())) < 1e-10);

        VectorXd Vdmu{VectorXd::Zero(_dim)};
        MatrixXd Vddmu{MatrixXd::Zero(_dim, _dim)};


        for (auto& opt_k : _vec_factor_optimizers){
            opt_k->calculate_partial_V();

            Vdmu = Vdmu + opt_k->joint_Vdmu();
            Vddmu = Vddmu + opt_k->joint_Vddmu();
        }
        
        MatrixXd dprecision = -_precision + Vddmu;
        MatrixXd new_precision = _precision + _step_size_precision * dprecision;
        // MatrixXd new_precision = Vddmu;

        VectorXd dmu = _precision.colPivHouseholderQr().solve(-Vdmu);
        VectorXd new_mu  = _mu + _step_size_mu * dmu;

        set_mu(new_mu);
        set_precision(new_precision);

    }


    /**
     * @brief One step in the update, with line search.
     * 
     * @ return: New cost after one step.
     */
    // template <typename FactorizedOptimizer>
    // void VIMPOptimizerGH<FactorizedOptimizer>::step(){

    //     double cost = cost_value();

    //     assert(abs(cost_value() - cost_value(_mu, covariance())) < 1e-10);

    //     /// purturb a little and see the cost
    //     // VectorXd purt_mu{purturb_mean(0.01)};
    //     // MatrixXd purt_precision{purturb_precision(0.01)};
    //     // MatrixXd purt_covariance{_inverser.inverse(purt_precision)};
        
    //     // cout << "purturbed mean cost before " << cost_value(purt_mu, covariance()) << endl;
    //     // cout << "purturbed precision cost before " << cost_value(mean(), purt_covariance) << endl;

    //     VectorXd Vdmu{VectorXd::Zero(_dim)};
    //     MatrixXd Vddmu{MatrixXd::Zero(_dim, _dim)};


    //     for (auto& opt_k : _vec_factor_optimizers){
    //         opt_k->calculate_partial_V();

    //         Vdmu = Vdmu + opt_k->joint_Vdmu();
    //         Vddmu = Vddmu + opt_k->joint_Vddmu();
    //     }
        
    //     MatrixXd dprecision = -_precision + Vddmu;
    //     MatrixXd new_precision = _precision + _step_size_precision * dprecision;

    //     VectorXd dmu = -_precision.colPivHouseholderQr().solve(Vdmu);
    //     VectorXd new_mu  = _mu + _step_size_mu * dmu;

    //     set_mu(new_mu);
    //     set_precision(new_precision);

    //     // /// Line search
    //     // double step_size = _step_size_mu;
    //     // while(cost < cost_value(new_mu, new_precision)){
    //     //     cout << "cost after " << cost_value(new_mu, _inverser.inverse(new_precision)) << endl;

    //     //     VectorXd purturbed_mu{purturb_mean(0.01)};
    //     //     MatrixXd purturbed_precision{purturb_precision(0.01)};
            
    //     //     cout << "purturbed cost after " << cost_value(purturbed_mu, _inverser.inverse(purturbed_precision)) << endl;

    //     //     step_size = step_size / 1.1;
    //     //     set_step_size(step_size, step_size);
    //     //     new_precision = _precision + _step_size_precision * dprecision;
    //     //     new_mu = _mu + _step_size_mu * dmu;
    //     // }
        
    //     // /// decreased cost
    //     // set_mu(new_mu);
    //     // set_precision(new_precision);
    //     // // double step_size = _step_size_mu * 1.1;
    //     // set_step_size(step_size, step_size);

    // }


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
     * @brief optimize without line search
     * 
     */
    template <typename FactorizedOptimizer>
    void VIMPOptimizerGH<FactorizedOptimizer>::optimize(){
        
        for (int i = 0; i < _niters; i++) {
            double step_size = _step_size_mu;
            step_size = step_size / pow((i + 1), 1 / 3);
            set_step_size(step_size, step_size);

            /// Collect the results
            VectorXd mean_iter{mean()};
            MatrixXd cov_iter{covariance()};
            MatrixXd precision_iter{_precision};

            cout << "iteration: " << i << endl;
            this->_res_recorder.update_data(mean_iter, cov_iter, precision_iter);
            step();
        }

        /// print 5 iteration datas 
        vector<int> iters{int(_niters/5), int(_niters*2/5), int(_niters*3/5), int(_niters*4/5), _niters-1};
        print_series_results(iters);

        save_data();
    } 


    // /**
    //  * @brief The optimizing process with line search.
    //  * 
    //  */
    // template <typename FactorizedOptimizer>
    // void VIMPOptimizerGH<FactorizedOptimizer>::optimize(){
    //     double step_size = 0.01;
    //     set_step_size(step_size, step_size);

    //     for (int i = 0; i < _niters; i++) {
    //         /// Collect the results
    //         cout << "covariance()" << endl << covariance() << endl;
    //         _res_recorder.update_data(VectorXd{mean()}, MatrixXd{covariance()});
    //         cout << "iteration: " << i << ", cost: " << cost_value() << endl;
    //         step();
    //         // step_size = step_size / pow((i + 1), 1 / 3);
    //         // set_step_size(step_size, step_size);
    //     }

    //     // /// print 5 iteration datas 
    //     // vector<int> iters{int(_niters/5), int(_niters*2/5), int(_niters*3/5), int(_niters*4/5), _niters-1};
    //     // print_series_results(iters);

    //     save_data();
    // } 

    /**
     * @brief Compute the total cost function value given a state.
     * 
     * @return cost value.
     */
    template <typename FactorizedOptimizer>
    double VIMPOptimizerGH<FactorizedOptimizer>::cost_value(const VectorXd& x, const MatrixXd& Cov){
        assert(dim() == x.size());
        assert(dim() == Cov.rows());
        assert(dim() == Cov.cols());

        double value = 0.0;
        for (auto& opt_k : _vec_factor_optimizers){
            VectorXd x_k = opt_k->Pk() * x;
            MatrixXd Cov_k = opt_k->Pk() * Cov * opt_k->Pk().transpose().eval();
            value += opt_k->cost_value(x_k, Cov_k);
        }
        MatrixXd Precision{_inverser.inverse(Cov)};
        value += _inverser.logdetD(Precision);
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
        for (auto& opt_k : _vec_factor_optimizers){
            value += opt_k->cost_value();
        }
        value += _inverser.logdetD(_precision);
        return value;
    } 


}
