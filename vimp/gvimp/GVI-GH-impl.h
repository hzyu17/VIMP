using namespace Eigen;
using namespace std;
#include <stdexcept>

#define STRING(x) #x
#define XSTRING(x) STRING(x)

namespace vimp
{
    
    /**
     * @brief One step of optimization.
     */
    template <typename Factor>
    std::tuple<VectorXd, SpMat> GVIGH<Factor>::compute_gradients(){
        _Vdmu.setZero();
        _Vddmu.setZero();

        for (auto &opt_k : _vec_factors)
        {
            opt_k->calculate_partial_V();
            _Vdmu = _Vdmu + opt_k->joint_Vdmu_sp();
            _Vddmu = _Vddmu + opt_k->joint_Vddmu_sp();
        }

        SpMat dprecision = _Vddmu - _precision;
        VectorXd dmu = _ei.solve_cgd_sp(_Vddmu, -_Vdmu);

        // MatrixXd Vddmu_full{_Vddmu};
        // VectorXd dmu = Vddmu_full.colPivHouseholderQr().solve(-_Vdmu);

        return std::make_tuple(dmu, dprecision);
    }

    template <typename Factor>
    void GVIGH<Factor>::switch_to_high_temperature(){
        for (auto& i_factor:_vec_factors){
            i_factor->switch_to_high_temperature();
        }
        this->initilize_precision_matrix();
    }

    /**
     * @brief optimize with backtracking
     */ 
    template <typename Factor>
    void GVIGH<Factor>::optimize(bool verbose)
    {

        for (int i_iter = 0; i_iter < _niters; i_iter++)
        {
            // ============= High temperature phase =============
            if (i_iter == _niters_lowtemp){
                this->switch_to_high_temperature();
            }

            // ============= Cost at current iteration =============
            double cost_iter = cost_value(_mu, _precision);

            if (verbose){
                cout << "========= iteration " << i_iter << " ========= " << endl;
                cout << "--- cost_iter ---" << endl << cost_iter << endl;
            }

            // ============= Collect factor costs =============
            VectorXd fact_costs_iter = factor_cost_vector();

            _res_recorder.update_data(_mu, _covariance, _precision, cost_iter, fact_costs_iter);

            // gradients
            std::tuple<VectorXd, SpMat> dmudprecision = this->compute_gradients();

            VectorXd dmu = std::get<0>(dmudprecision);
            SpMat dprecision = std::get<1>(dmudprecision);
            
            int cnt = 0;
            int B = 1;
            double step_size = 0.0;

            SpMat new_precision; 
            VectorXd new_mu; 

            // backtracking 
            while (true)
            {
                new_mu.setZero(); new_precision.setZero();
                // new step size
                step_size = pow(_step_size_base, B);

                // update mu and precision matrix
                new_mu = _mu + step_size * dmu;
                new_precision = _precision + step_size * dprecision;

                // new cost
                double new_cost = cost_value(new_mu, new_precision);

                // accept new cost and update mu and precision matrix
                if (new_cost < cost_iter){
                    /// update mean and covariance
                    set_mu(new_mu);
                    set_precision(new_precision);
                    break;
                }else{ 
                    // shrinking the step size
                    B += 1;
                    cnt += 1;
                }

                if (cnt > _niters_backtrack)
                {
                    if (verbose){
                        cout << "Too many iterations in the backtracking ... Dead" << endl;
                    }
                    set_mu(new_mu);
                    set_precision(new_precision);
                        break;
                }                
            }
        }

        save_data(verbose);

    }


    template <typename Factor>
    inline void GVIGH<Factor>::set_precision(const SpMat &new_precision)
    {
        _precision = new_precision;
        // sparse inverse
        inverse_inplace();

        for (auto &factor : _vec_factors)
        {
            factor->update_precision_from_joint(_covariance);
        }
    }

    /**
     * @brief Compute the costs of all factors for a given mean and cov.
     */
    template <typename Factor>
    VectorXd GVIGH<Factor>::factor_cost_vector(const VectorXd& joint_mean, SpMat& joint_precision)
    {
        VectorXd fac_costs(_nfactors);
        fac_costs.setZero();
        int cnt = 0;
        SpMat joint_cov = inverse(joint_precision);
        for (auto &opt_k : _vec_factors)
        {
            fac_costs(cnt) = opt_k->fact_cost_value(joint_mean, joint_cov); // / _temperature;
            cnt += 1;
        }
        return fac_costs;
    }

    /**
     * @brief Compute the costs of all factors, using current values.
     */
    template <typename Factor>
    VectorXd GVIGH<Factor>::factor_cost_vector()
    {   
        return factor_cost_vector(_mu, _precision);
    }

    /**
     * @brief Compute the total cost function value given a state.
     */
    template <typename Factor>
    double GVIGH<Factor>::cost_value(const VectorXd &mean, SpMat &Precision)
    {

        SpMat Cov = inverse(Precision);

        double value = 0.0;
        for (auto &opt_k : _vec_factors)
        {
            value += opt_k->fact_cost_value(mean, Cov); // / _temperature;
        }

        SparseLDLT ldlt(Precision);
        VectorXd vec_D = ldlt.vectorD();

        // MatrixXd precision_full{Precision};
        double logdet = 0;
        for (int i_diag=0; i_diag<vec_D.size(); i_diag++){
            logdet += log(vec_D(i_diag));
        }

        // cout << "logdet " << endl << logdet << endl;
        
        return value + logdet / 2;
    }

    /**
     * @brief Compute the total cost function value given a state, using current values.
     */
    template <typename Factor>
    double GVIGH<Factor>::cost_value()
    {
        return cost_value(_mu, _precision);
    }

    /**
     * @brief given a state, compute the total cost function value without the entropy term, using current values.
     */
    template <typename Factor>
    double GVIGH<Factor>::cost_value_no_entropy()
    {
        
        SpMat Cov = inverse(_precision);
        
        double value = 0.0;
        for (auto &opt_k : _vec_factors)
        {
            value += opt_k->fact_cost_value(_mu, Cov);
        }
        return value; // / _temperature;
    }

}
