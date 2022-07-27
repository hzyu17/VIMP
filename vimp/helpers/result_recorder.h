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

namespace vimp{
    struct VIMPResults{
        MatrixXd _res_mean;
        vector<MatrixXd> _vec_res_covariances;
        vector<MatrixXd> _vec_res_precisions;
        int _niters, _nstates;
        int _cur_iter=0;
        vector<double> _vec_cost_values;

        std::string _file_mean;
        std::string _file_cov;
        std::string _file_cost;

        /**
         * @brief Constructor
         * 
         * @param niters number of iterations
         * @param nstates number of states
         */
        VIMPResults(int niters, int nstates){
            _niters = niters;
            _nstates = nstates;
            _file_mean = "mean.csv";
            _file_cov = "cov.csv";
            _file_cost = "cost.csv";
            reinitialize_data();
        }

        /**
         * @brief reinitialize data sizes.
         * 
         */
        inline void reinitialize_data(){
            _res_mean = std::move(MatrixXd::Zero(_niters, _nstates));
            _vec_res_covariances.resize(_niters);
            _vec_res_precisions.resize(_niters);
            _vec_cost_values.resize(_niters);
            _cur_iter = 0;
        }

        /**
         * @brief set a new number of iterations
         * 
         * @param niters 
         */
        inline void update_niters(int niters){
            _niters = niters;
            reinitialize_data();
        }

        /**
         * @brief update the content of data 
         * 
         * @param new_mean the new coming mean vector
         * @param new_cov the new coming covariance matrix
         * @param new_precision the new coming precision matrix
         */
        void update_data(const VectorXd& new_mean, const MatrixXd& new_cov, 
                        const MatrixXd& new_precision, const double& new_cost){
            if (_cur_iter < _niters){
                _res_mean.row(_cur_iter) = std::move(new_mean.transpose());
                _vec_res_covariances[_cur_iter] = std::move(new_cov);
                _vec_res_precisions[_cur_iter] = std::move(new_precision);
                _vec_cost_values[_cur_iter] = new_cost;
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
        inline void print_data(int i_iter){
            assert(i_iter < _niters);
            cout << "mean: " << endl << _res_mean.row(i_iter) << endl << endl;
            cout << "precision: " << endl << _vec_res_precisions[i_iter] << endl;
            cout << "cost: " << endl << _vec_cost_values[i_iter] << endl;

        }

        /**
         * @brief update filenames
         * 
         * @param file_mean filename for the means
         * @param file_cov filename for the covariances
         */
        inline void update_file_names(const string& file_mean, 
                                      const string& file_cov, 
                                      const string& file_cost){
            _file_mean = file_mean;
            _file_cov = file_cov;
            _file_cost = file_cost;
        }

        /**
         * @brief save res means and covariances to csv file
         */
        void save_data(){
            /// save mean
            ofstream file(_file_mean);
            if (file.is_open()){
                file << _res_mean.format(CSVFormat);
                file.close();
            }

            /// save covariances
            ofstream f_cov(_file_cov);
            if (f_cov.is_open()){
                for (MatrixXd& i_cov:_vec_res_covariances){
                    f_cov << i_cov.format(CSVFormat) << "\n";
                }

                f_cov.close();
            }

            /// save costs
            ofstream f_cost(_file_cost);
            if (f_cost.is_open()){
                for (double& i_cost:_vec_cost_values){
                    f_cost << i_cost << "\n";
                }
                f_cost.close();
            }
        }
            
    };
}
