using namespace Eigen;
using namespace std;
#include <stdexcept>

#define STRING(x) #x
#define XSTRING(x) STRING(x)

namespace vimp
{

    /**
     * @brief optimize with backtracking
     */
    template <typename Factor>
    void GVIGH<Factor>::optimize()
    {
        double new_cost = 0.0;

        for (int i_iter = 0; i_iter < _niters; i_iter++)
        {
            cout << "========= iteration " << i_iter << " ========= " << endl;
            // ============= Collect results =============
            VectorXd fact_costs_iter = factor_costs();

            _ei.print_matrix(fact_costs_iter, "fact_costs_iter");
            double cost_iter = cost_value(_mu, _precision);

            cout << "=== cost_iter ===" << endl << cost_iter << endl;

            _res_recorder.update_data(_mu, _covariance, _precision, cost_iter, fact_costs_iter);
            // one step
            _Vdmu.setZero();
            _Vddmu.setZero();

            for (auto &opt_k : _vec_factors)
            {
                opt_k->calculate_partial_V_GH();
                _Vdmu = _Vdmu + opt_k->joint_Vdmu_sp();
                _Vddmu = _Vddmu + opt_k->joint_Vddmu_sp();
            }

            SpMat dprecision = -_precision + _Vddmu;
            MatrixXd Vddmu_full{_Vddmu};
            VectorXd Vdmu_full{_Vdmu};
            VectorXd dmu = Vddmu_full.colPivHouseholderQr().solve(-Vdmu_full);

            // VectorXd dmu = _ei.solve_cgd_sp(_Vddmu, -_Vdmu);

            int cnt = 0;
            int B = 1;
            double step_size;

            SpMat new_precision; new_precision.setZero();
            VectorXd new_mu; new_mu.setZero();

            // std::cout << "_step_size_base:   " << _step_size_base << std::endl;

            step_size = pow(_step_size_base, B);
            new_mu = _mu + step_size * dmu;
            new_precision = _precision + step_size * dprecision;

            new_cost = cost_value(new_mu, new_precision);

            std::cout << "--- new_cost ---" << std::endl << new_cost << std::endl;

            while (new_cost > cost_iter)
            {
                B += 1;
                step_size = pow(_step_size_base, B);
                new_mu = _mu + step_size * dmu;
                new_precision = _precision + step_size * dprecision;
                new_cost = cost_value(new_mu, new_precision);

                std::cout << "new_cost backtrack" << std::endl << new_cost << std::endl;

                cnt += 1;

                if (cnt > _niters_backtrack)
                {
                    // throw std::runtime_error(std::string("Too many iterations in the backtracking ... Dead"));
                    cout << "Too many iterations in the backtracking ... Dead" << endl;
                    break;
                }
            }

            /// update mean and covariance
            set_mu(new_mu);
            set_precision(new_precision);

            cost_iter = new_cost;

            B = 1;
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
        // cout << "=== final cost ===" << endl
        //      << cost_iter << endl;
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
    VectorXd GVIGH<Factor>::factor_costs(const VectorXd& joint_mean, SpMat& joint_precision)
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
    VectorXd GVIGH<Factor>::factor_costs()
    {
        return factor_costs(_mu, _precision);
    }

    /**
     * @brief Compute the total cost function value given a state.
     */
    template <typename Factor>
    double GVIGH<Factor>::cost_value(const VectorXd &mean, SpMat &Precision)
    {

        SpMat Cov = inverse(Precision);

        MatrixXd cov_sp{Cov};
        MatrixXd precision_full{Precision};
        MatrixXd cov_full = precision_full.inverse();

        double value = 0.0;
        for (auto &opt_k : _vec_factors)
        {
            value += opt_k->fact_cost_value(mean, Cov); // / _temperature;
        }

        SparseLDLT ldlt(Precision);
        double det = ldlt.determinant();

        if (det < 0)
        {
            std::runtime_error("Infinity log determinant precision matrix ...");
        }
        return value + log(det) / 2;
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
    double GVIGH<Factor>::cost_value_no_entropy() const
    {
        double value = 0.0;
        SpMat Cov = inverse(_precision);
        for (auto &opt_k : _vec_factors)
        {
            value += opt_k->fact_cost_value(_mu, Cov);
        }
        return value; // / _temperature;
    }

}
