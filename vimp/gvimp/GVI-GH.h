/**
 * @file GVI-GH.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The joint optimizer class using Gauss-Hermite quadrature. 
 * @version 0.1
 * @date 2022-03-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include <utility>
#include <memory>

#include "../helpers/result_recorder.h"
#include "../helpers/eigen_wrapper.h"
#include "../helpers/timer.h"

using namespace std;
using namespace Eigen;


namespace vimp{

template <typename FactorizedOptimizer>
class GVIGH{
public:
    /**
     * @brief Default Constructor
     */
    GVIGH(){}

    /**
     * @brief Construct a new VIMPOptimizerGH object
     * 
     * @param _vec_fact_optimizers vector of marginal optimizers
     * @param niters number of iterations
     */
    GVIGH(const std::vector<std::shared_ptr<FactorizedOptimizer>>& vec_fact_optimizers, int dim_state, int num_states, double temperature=1.0):
                                   _dim_state{dim_state},
                                   _num_states{num_states},
                                   _dim{dim_state*num_states},
                                   _niters{10},
                                   _temperature{temperature},
                                   _nfactors{vec_fact_optimizers.size()},
                                   _vec_factors{std::move(vec_fact_optimizers)},
                                   _mu{VectorXd::Zero(_dim)},
                                   _Vdmu{VectorXd::Zero(_dim)},
                                   _Vddmu{SpMat(_dim, _dim)},
                                   _precision{SpMat(_dim, _dim)},
                                   _ldlt{_precision},
                                   _covariance{SpMat(_dim, _dim)},
                                   _res_recorder{_niters, _dim}
    {
                _precision.setZero();
                // fill in the precision matrix to the known sparsity pattern
                for (int i=0; i<num_states-1; i++){
                    Eigen::MatrixXd block = MatrixXd::Ones(2*_dim_state, 2*_dim_state);
                    _eigen_wrapper.block_insert_sparse(_precision, i*_dim_state, i*_dim_state, 2*_dim_state, 2*_dim_state, block);
                }

                SpMat lower = _precision.triangularView<Eigen::Lower>();
                _eigen_wrapper.find_nnz(lower, _Rows, _Cols, _Vals); // the Rows and Cols table are fixed since the initialization.
                _nnz = _Rows.rows();
                
                _Vdmu.setZero();
                _Vddmu.setZero();
    }

protected:
    /// optimization variables
    int _dim, _niters, _nfactors, _dim_state, _num_states;

    /// @param _vec_factors Vector of marginal optimizers
    vector<std::shared_ptr<FactorizedOptimizer>> _vec_factors;

    VectorXd _mu;

    VectorXd _Vdmu;
    SpMat _Vddmu;
    
    double _temperature;

    /// Data and result storage
    VIMPResults _res_recorder;
    MatrixIO _matrix_io;

    // sparse matrices
    SpMat _precision, _covariance;
    EigenWrapper _eigen_wrapper = EigenWrapper();
    VectorXi _Rows, _Cols; VectorXd _Vals;
    int _nnz = 0;
    SparseLDLT _ldlt;
    SpMat _L; VectorXd _D, _Dinv; // for computing the determinant

    // timer helper
    Timer _timer = Timer();

    /// step sizes by default
    double _step_size = 0.9;
    double _step_size_base = 0.55;

    /// filename for the perturbed costs
    std::string _file_perturbed_cost;

    void ldlt_decompose(){
        _ldlt.compute(_precision);
        _L = _ldlt.matrixL();
        _Dinv = _ldlt.vectorD().real().cwiseInverse();
        _eigen_wrapper.find_nnz_known_ij(_L, _Rows, _Cols, _Vals);
        // _D = ldlt.vectorD().real();
    }

public:
/// **************************************************************
/// Optimizations related
    /**
     * @brief Function which computes one step of update.
     */
    void step();


    /**
     * @brief The optimizing process.
     */
    void optimize();

    /**
     * @brief Compute the total cost function value given a mean and covariace.
     */
    double cost_value(const VectorXd& x, SpMat& Precision);

    /**
     * @brief Compute the total cost function value given a state, using current values.
     */
    double cost_value();

    /**
     * @brief given a state, compute the total cost function value without the entropy term, using current values.
     */
    double cost_value_no_entropy() const;

    /**
     * @brief Compute the costs of all factors for a given mean and cov.
     */
    VectorXd factor_costs(const VectorXd& x, const SpMat& Precision) const;

    /**
     * @brief Compute the costs of all factors, using current values.
     */
    VectorXd factor_costs() const;


/// **************************************************************
/// Internal data IO
    inline VectorXd mean() const{ return _mu; }

    inline SpMat precision() const{ return _precision; }

    /// returns the covariance matrix
    inline SpMat covariance(){ 
        inverse_inplace();
        return _covariance; 
    }

    inline void inverse_inplace(){
        ldlt_decompose();
        _eigen_wrapper.inv_sparse(_precision, _covariance, _Rows, _Cols, _Vals, _Dinv);
    }

    inline SpMat inverse(SpMat & mat){
        SpMat res(_dim, _dim);
        _eigen_wrapper.inv_sparse(mat, res, _Rows, _Cols, _nnz);
        return res;
    }

    /**
     * @brief Purturb the mean by a random vector.
     */
    inline VectorXd purturb_mean(double scale=0.01) const{
        return VectorXd{_mu + VectorXd::Random(_dim) * scale};
    }

    inline MatrixXd purturb_precision(double scale=0.01) const{
        MatrixXd purturbation = MatrixXd::Zero(_dim, _dim);
        purturbation.triangularView<Upper>() = (scale * MatrixXd::Random(_dim, _dim)).triangularView<Upper>();
        purturbation.triangularView<Lower>() = purturbation.triangularView<Upper>().transpose();
        purturbation = MatrixXd::Identity(_dim, _dim) + purturbation;

        // save_matrix("data/2d_pR/purturbed.csv", purturbation*_precision*purturbation);
        return MatrixXd{purturbation*_precision*purturbation};
    }   

    inline double purturbed_cost(double scale=0.01) const{
        // VectorXd p_mean = purturb_mean(scale);
        // MatrixXd p_precision = purturb_precision(scale);
        return cost_value(purturb_mean(scale), purturb_precision(scale).inverse());
    }


    /**
     * @brief Repeated purturbations and statistics.
     */
    inline void purturbation_stat(double scale=0.01, int n_experiments=100) const{
        VectorXd purturbed_costs(n_experiments);
        for (int i=0; i<n_experiments; i++){
            purturbed_costs(i) = purturbed_cost(scale);
        }
        cout << "----- Average purturbed cost -----" << endl << purturbed_costs.mean() << endl;
        cout << "----- Min purturbed cost -----" << endl << purturbed_costs.minCoeff() << endl;
        // save_vector("data/2d_pR/purturbation_statistics.csv", purturbed_costs);
        save_vector(_file_perturbed_cost, purturbed_costs);
    }

    /// update the step sizes
    inline void set_step_size(double step_size){ _step_size = step_size; }

    /// The base step size in backtracking
    inline void set_step_size_base(double step_size_base){ _step_size_base = step_size_base; }

    inline void set_mu(const VectorXd& mean){
        _mu = mean; 
        for (std::shared_ptr<FactorizedOptimizer> & opt_fact : _vec_factors){
            opt_fact->update_mu_from_joint(_mu);
        }
    }

    inline void set_precision(const SpMat& new_precision);
    
    inline void set_niterations(int niters){
        _niters = niters;
        _res_recorder.update_niters(niters); }

    inline void set_initial_values(const VectorXd& init_mean, const SpMat& init_precision){
        set_mu(init_mean);
        set_precision(init_precision);
    }

    inline void set_GH_degree(const int deg){
        for (auto & opt_fact : _vec_factors){
            opt_fact->set_GH_points(deg);
        }
    }

    
/// **************************************************************
/// Experiment data and result recordings
    /**
     * @brief update filenames
     * @param file_mean filename for the means
     * @param file_cov filename for the covariances
     */
    inline void update_file_names(const string& file_mean, 
                                  const string& file_cov, 
                                  const string& file_precision, 
                                  const string& file_cost,
                                  const string& file_fac_costs,
                                  const string& file_perturbed_costs){
        _res_recorder.update_file_names(file_mean, file_cov, file_precision, file_cost, file_fac_costs);
        _file_perturbed_cost = file_perturbed_costs;
    }

    inline void update_file_names(const string & prefix = "", const string & afterfix=""){
        std::vector<string> vec_filenames;
        vec_filenames.emplace_back("mean");
        vec_filenames.emplace_back("cov");
        vec_filenames.emplace_back("precisoin");
        vec_filenames.emplace_back("cost");
        vec_filenames.emplace_back("factor_costs");
        vec_filenames.emplace_back("perturbation_statistics");

        string underscore{"_"};
        string file_type{".csv"};

        if (prefix != ""){
            for (string & i_file : vec_filenames){
                i_file = prefix + i_file;
            }
        }

        if (afterfix != ""){
            for (string & i_file : vec_filenames){
                i_file = i_file + underscore + afterfix;
            }
        }

        for (string & i_file : vec_filenames){
                i_file = i_file + file_type;
        }

        _res_recorder.update_file_names(vec_filenames[0], vec_filenames[1], vec_filenames[2], vec_filenames[3], vec_filenames[4]);
        _file_perturbed_cost = vec_filenames[5];
    }

    /**
     * @brief save process data into csv files.
     */
    inline void save_data() { _res_recorder.save_data();}

    /**
     * @brief save a matrix to a file. 
     */
    inline void save_matrix(const string& filename, const MatrixXd& m) const{
        _matrix_io.saveData<MatrixXd>(filename, m);
    }

    inline void save_vector(const string& filename, const VectorXd& vec) const{
        _matrix_io.saveData<VectorXd>(filename, vec);
    }

    /**
     * @brief print a given iteration data mean and covariance.
     * @param i_iter index of data
     */
    inline void print_result(const int& i_iter){
        _res_recorder.print_data(i_iter);}

    /**
     * @brief print out a given number of iterations results
     * @param iters a list of iterations to be printed
     */
    inline void print_series_results(const vector<int>& iters) {
        std::for_each(iters.begin(), iters.end(), [this](int i) { 
            cout << "--- result at iteration " << i << "---" << endl;
            print_result(i);
            }
        );

    }

    inline int dim() const{ return _dim; }   

    inline int n_sub_factors() const{ return _nfactors; }

    /**
     * @brief calculate and return the E_q{phi(x)} s for each factorized entity.
     * @return vector<double> 
     */
    vector<double> E_Phis(){
        vector<double> res;
        for (auto & p_factor: _vec_factors){
            res.emplace_back(p_factor->E_Phi());
        }
        return res;
    }

    /**
     * @brief calculate and return the E_q{(x-mu).*phi(x)} s for each factorized entity.
     * @return vector<double> 
     */
    vector<MatrixXd> E_xMuPhis(){
        vector<MatrixXd> res;
        for (auto & p_factor: _vec_factors){
            res.emplace_back(p_factor->E_xMuPhi());
        }
        return res;
    }

    /**
     * @brief calculate and return the E_q{(x-mu).*phi(x)} s for each factorized entity.
     * @return vector<double> 
     */
    vector<MatrixXd> E_xMuxMuTPhis(){
        vector<MatrixXd> res;
        for (auto & p_factor: _vec_factors){
            res.emplace_back(p_factor->E_xMuxMuTPhi());
        }
        return res;
    }


    /**************************** ONLY FOR 1D CASE ***********************/
    /**
     * @brief Draw a heat map for cost function in 1d case
     * @return MatrixXd heatmap of size (nmesh, nmesh)
     */
    MatrixXd cost_map(const double& x_start, 
                      const double& x_end, const double& y_start, 
                      const double& y_end, const int& nmesh){
        double res_x = (x_end - x_start) / nmesh;
        double res_y = (y_end - y_start) / nmesh;
        MatrixXd Z = MatrixXd::Zero(nmesh, nmesh);

        for (int i=0; i<nmesh; i++){
            VectorXd mean{VectorXd::Constant(1, x_start + i*res_x)};
            for (int j=0; j<nmesh; j++){
                SpMat precision(1, 1);
                precision.coeffRef(0, 0) = (y_start + j*res_y);
                Z(j, i) = cost_value(mean, precision); /// the order of the matrix in cpp and in matlab
            }
        }
        return Z;
    }

    /**
     * @brief save the cost map
     */
    void save_costmap(string filename="costmap.csv"){
        MatrixXd cost_m = cost_map(18, 25, 0.05, 1, 40);
        ofstream file(filename);
        if (file.is_open()){
            file << cost_m.format(CSVFormat);
            file.close();}
    }

    }; //class
} //namespace vimp

#include "../gvimp/GVI-GH-impl.h"