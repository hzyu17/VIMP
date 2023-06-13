/**
 * @file run_experiment_template.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The template of running a motion planning experiment for 7D robot.
 * @version 0.1
 * @date 2023-04-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "ExperimentRunner.h"

using namespace Eigen;

namespace vimp{

template <typename PGCSOptimizer>
class ExperimentRunner7D: public ExperimentRunner<PGCSOptimizer>{
public:
    
    ExperimentRunner7D(int num_exp, const std::string & config):
                        ExperimentRunner<PGCSOptimizer>(14, 7, num_exp, config){}

    void read_boundary_conditions(const rapidxml::xml_node<>* paramNode) override{
        double start_1 = atof(paramNode->first_node("start_pos")->first_node("1")->value());
        double start_2 = atof(paramNode->first_node("start_pos")->first_node("2")->value());
        double start_3 = atof(paramNode->first_node("start_pos")->first_node("3")->value());
        double start_4 = atof(paramNode->first_node("start_pos")->first_node("4")->value());
        double start_5 = atof(paramNode->first_node("start_pos")->first_node("5")->value());
        double start_6 = atof(paramNode->first_node("start_pos")->first_node("6")->value());
        double start_7 = atof(paramNode->first_node("start_pos")->first_node("7")->value());

        double goal_1 = atof(paramNode->first_node("goal_pos")->first_node("1")->value());
        double goal_2 = atof(paramNode->first_node("goal_pos")->first_node("2")->value());
        double goal_3 = atof(paramNode->first_node("goal_pos")->first_node("3")->value());
        double goal_4 = atof(paramNode->first_node("goal_pos")->first_node("4")->value());
        double goal_5 = atof(paramNode->first_node("goal_pos")->first_node("5")->value());
        double goal_6 = atof(paramNode->first_node("goal_pos")->first_node("6")->value());
        double goal_7 = atof(paramNode->first_node("goal_pos")->first_node("7")->value());

        VectorXd m0(14), mT(14); 

        Eigen::VectorXd mo_pos(7);
        mo_pos << start_1, start_2, start_3, start_4, start_5, start_6, start_7;
        m0.block(0, 0, 7, 1) = mo_pos;
        m0.block(7, 0, 7, 1) = Eigen::VectorXd::Zero(7);

        Eigen::VectorXd mT_pos(7);
        mT_pos << goal_1, goal_2, goal_3, goal_4, goal_5, goal_6, goal_7;
        mT.block(0, 0, 7, 1) = mT_pos;
        mT.block(7, 0, 7, 1) = Eigen::VectorXd::Zero(7);

        double sig_obs = atof(paramNode->first_node("cost_sigma")->value());

        this->_params.set_m0(m0);
        this->_params.set_mT(mT);
        this->_params.update_sig_obs(sig_obs);
    }

};

}