/**
 * @file result_recorder.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Struct to store the results along the optimizing process.
 * @version 0.1
 * @date 2022-07-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "repeated_includes.h"

namespace MPVI{
    struct MPVIResults{
        MatrixXd _res_mean;
        vector<MatrixXd> _vec_res_covariances;
        int _niters, _nstates;
        int _cur_iter=0;

        /**
         * @brief Constructor
         * 
         * @param niters number of iterations
         * @param nstates number of states
         */
        MPVIResults(int niters, int nstates){
            _niters = niters;
            _nstates = nstates;
            reinitialize_data();
        }

        /**
         * @brief reinitialize data sizes.
         * 
         */
        void reinitialize_data(){
            _res_mean = std::move(MatrixXd::Zero(_niters, _nstates));
            _vec_res_covariances.resize(_niters);
            _cur_iter = 0;
        }

        /**
         * @brief set a new number of iterations
         * 
         * @param niters 
         */
        void update_niters(int niters){
            _niters = niters;
            reinitialize_data();
        }

        /**
         * @brief update the content of data 
         * 
         * @param new_mean the new coming mean vector
         * @param new_cov the new coming covariance matrix
         */
        void update_data(const VectorXd& new_mean, const MatrixXd& new_cov){
            if (_cur_iter < _niters){
                _res_mean.row(_cur_iter) = std::move(new_mean.transpose());
                _vec_res_covariances[_cur_iter] = std::move(new_cov);
                _cur_iter += 1;
            }
            else{
                cout << "reached the last iteration" << endl;
            }
        }

        /**
         * @brief print the ith iteration data.
         * 
         * @param i_iter 
         */
        void print_data(int i_iter){
            assert(i_iter < _niters);
            cout << "mean: " << endl << _res_mean.row(i_iter) << endl << endl;

            cout << "covariance: " << endl << _vec_res_covariances[i_iter] << endl;

        }

        /**
         * @brief save res means and covariances to csv file
         * 
         * @param file_mean filename for the means
         * @param file_cov filename for the covariances
         */
        void save_data(const string& file_mean, const string& file_cov){
            /// save mean
            ofstream file(file_mean);
            if (file.is_open()){
                file << _res_mean.format(CSVFormat);
                file.close();
            }

            /// save covariances
            ofstream f_cov(file_cov);
            if (f_cov.is_open()){
                for (MatrixXd& i_cov:_vec_res_covariances){
                    f_cov << i_cov.format(CSVFormat) << "\n";
                }

                f_cov.close();
            }
        }
            
    };
}
