/**
 * @file DataBuffer.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Data buffer class which is used for backtracking and cost analysis.
 * @version 0.1
 * @date 2023-04-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "repeated_includes.h"
#include "base/Matrix.h"
#include "eigen_wrapper.h"
#include <vector>

using namespace std;
#include <vector>

using namespace Eigen;
using namespace std;

namespace vimp
{

class VIMPResults{
private:    
        Matrix3D _res_mean;
        Matrix3D _res_covariances;
        Matrix3D _res_precisions;
        int _niters, _dimension, _nfactors, _dim_state, _nstates;
        int _cur_iter = 0;
        VectorXd _res_costs;
        Matrix3D _res_factor_costs;

        std::string _file_mean{"mean.csv"};
        std::string _file_cov{"cov.csv"};
        std::string _file_precision{"precision.csv"};
        std::string _file_cost{"cost.csv"};
        std::string _file_factor_costs{"factor_costs.csv"};

        EigenWrapper _ei;
        MatrixIO _m_io;

public:
    /**
     * @brief Constructor
     * 
     * @param niters number of iterations
     * @param nstates number of states
     * @param dim_state dimension of state
     */
    VIMPResults(int niters, int dim_state, int nstates, int n_factors):
        _dim_state(dim_state),
        _nstates(nstates),
        _dimension(dim_state*nstates),
        _niters(niters),
        _nfactors(n_factors),
        _res_mean(dim_state*nstates, 1, niters),
        _res_covariances(dim_state*dim_state, nstates, niters),
        _res_precisions(dim_state*dim_state, nstates, niters),
        _res_costs(niters),
        _res_factor_costs(n_factors, 1, niters)            
        {}

    /**
     * @brief update the content of data 
     * 
     * @param new_mean the new coming mean vector
     * @param new_cov the new coming covariance matrix
     * @param new_precision the new coming precision matrix
     */
    void update_data(const Eigen::VectorXd& new_mean, const Eigen::MatrixXd& new_joint_cov, 
                    const Eigen::MatrixXd& new_joint_precision, const double& new_cost, 
                    const Eigen::VectorXd& new_factor_costs){
        if (_cur_iter < _niters){
            Matrix3D marginal_cov(_dim_state, _dim_state, _nstates);
            marginal_cov = joint2marginals(new_joint_cov);
            Matrix3D marginal_precision(_dim_state, _dim_state, _nstates);
            marginal_precision = joint2marginals(new_joint_precision);

            _ei.compress3d(new_mean, _res_mean, _cur_iter);
            _ei.compress3d(marginal_cov, _res_covariances, _cur_iter);    
            _ei.compress3d(marginal_precision, _res_precisions, _cur_iter);   
            _ei.compress3d(new_factor_costs, _res_factor_costs, _cur_iter);              
            _res_costs(_cur_iter) = new_cost;
            _cur_iter += 1;
        }
        else{
            std::cout << "reached the last iteration" << std::endl;
        }
    }

    /**
     * @brief Construct 3d marginal covariance or precision 
     * matrices (with time information) from a big joint matrix.
     * @param joint 
     * @return Matrix3D 
     */
    inline Matrix3D joint2marginals(const MatrixXd& joint){
        Matrix3D res(_dim_state, _dim_state, _nstates);
        for (int i=0; i<_nstates; i++){
            int start_i = i*_dim_state;
            _ei.compress3d(joint.block(start_i, start_i, _dim_state, _dim_state), res, i);
        }
        return res;
    }

    /**
     * @brief print the ith iteration data.
     * 
     * @param i_iter 
     */
    inline void print_data(int i_iter){
        assert(i_iter < _niters);
        _ei.print_matrix(_ei.decomp3d(_res_mean, _dimension, 1, i_iter), "mean: ");
        _ei.print_matrix(_ei.decomp3d(_res_precisions, _dimension, _dimension, i_iter), "precision: ");
        std::cout << "cost: " << std::endl << _res_costs(i_iter) << std::endl;

    }

    /**
     * @brief update filenames
     * 
     * @param file_mean filename for the means
     * @param file_cov filename for the covariances
     */
    inline void update_file_names(const std::string& file_mean, 
                                    const std::string& file_cov, 
                                    const std::string& file_precision,
                                    const std::string& file_cost,
                                    const std::string& file_factor_costs){
        _file_mean = file_mean;
        _file_cov = file_cov;
        _file_cost = file_cost;
        _file_precision = file_precision;
        _file_factor_costs = file_factor_costs;
    }

    /**
     * @brief save res means and covariances to csv file
     */
    void save_data(){
        // std::cout << "save data " << std::endl;
        /// save mean
        ofstream file(_file_mean);
        _m_io.saveData(_file_mean, _res_mean);

        /// save covariances
        ofstream f_cov(_file_cov);
        _m_io.saveData(_file_cov, _res_covariances);

        /// save precisions
        ofstream f_prec(_file_precision);
        _m_io.saveData(_file_precision, _res_precisions);
            
        /// save costs
        ofstream f_cost(_file_cost);
        _m_io.saveData(_file_cost, _res_costs);

        /// save factored osts
        ofstream f_factor_costs(_file_factor_costs);
        _m_io.saveData(_file_factor_costs, _res_factor_costs);

        std::cout << "save data " << std::endl;
    }
};

/**
 * @brief A class which record the iterations of data.
 */
class PGCSDataRecorder{
public:
    PGCSDataRecorder(){}
    PGCSDataRecorder(const Matrix3D& Akt, 
                const Matrix3D& Bt, 
                const Matrix3D& akt,
                const Matrix3D& Qkt,
                const Matrix3D& rkt,
                const Matrix3D& Kkt,
                const Matrix3D& dkt,
                const Matrix3D& zkt,
                const Matrix3D& Sigkt)
                {
                    add_iteration(Akt, Bt, akt, Qkt, rkt, Kkt, dkt, zkt, Sigkt);
                }
                
    void add_iteration(const Matrix3D& Akt, 
                    const Matrix3D& Bt, 
                    const Matrix3D& akt,
                    const Matrix3D& Qkt,
                    const Matrix3D& rkt,
                    const Matrix3D& Kkt,
                    const Matrix3D& dkt,
                    const Matrix3D& zkt,
                    const Matrix3D& Sigkt){
                        
                    _Akt.push_back(Akt);
                    _Bt.push_back(Bt);
                    _akt.push_back(akt);
                    _Qkt.push_back(Qkt);
                    _rkt.push_back(rkt);
                    _Kkt.push_back(Kkt),
                    _dkt.push_back(dkt);
                    _zkt.push_back(zkt);
                    _Sigkt.push_back(Sigkt);
                }

    void write_records(){
        
    }


private:
    EigenWrapper _ei;

    // 3D matrices
    vector<Matrix3D> _Akt, _Bt, _akt;
    vector<Matrix3D> _Qkt, _rkt;
    vector<Matrix3D> _Kkt, _dkt;

    // linearizations
    vector<Matrix3D> _hAkt, _hakt;

    // nominal means and covariances
    vector<Matrix3D> _zkt, _Sigkt;

};

} // namespace vimp