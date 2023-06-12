/**
 * @file run_experiment_template.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The template of running a motion planning experiment for a 3D robot.
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
class ExperimentRunner3D: public ExperimentRunner<PGCSOptimizer>{
public:
    
    ExperimentRunner3D(int num_exp, const std::string & config):
                        ExperimentRunner<PGCSOptimizer>(6, 3, num_exp, config){}

    void read_boundary_conditions(const rapidxml::xml_node<>* paramNode) override{
        double start_x = atof(paramNode->first_node("start_pos")->first_node("x")->value());
        double start_y = atof(paramNode->first_node("start_pos")->first_node("y")->value());
        double start_z = atof(paramNode->first_node("start_pos")->first_node("z")->value());

        double start_vx = atof(paramNode->first_node("start_pos")->first_node("vx")->value());
        double start_vy = atof(paramNode->first_node("start_pos")->first_node("vy")->value());
        double start_vz = atof(paramNode->first_node("start_pos")->first_node("vz")->value());

        double goal_x = atof(paramNode->first_node("goal_pos")->first_node("x")->value());
        double goal_y = atof(paramNode->first_node("goal_pos")->first_node("y")->value());
        double goal_z = atof(paramNode->first_node("goal_pos")->first_node("z")->value());

        double goal_vx = atof(paramNode->first_node("goal_pos")->first_node("vx")->value());
        double goal_vy = atof(paramNode->first_node("goal_pos")->first_node("vy")->value());
        double goal_vz = atof(paramNode->first_node("goal_pos")->first_node("vz")->value());

        this->_m0 << start_x, start_y, start_z, start_vx, start_vy, start_vz;
        this->_Sig0 = this->sig0 * Eigen::MatrixXd::Identity(this->_nx, this->_nx);

        this->_mT << goal_x, goal_y, goal_z, goal_vx, goal_vy, goal_vz;
        this->_SigT = this->sigT * Eigen::MatrixXd::Identity(this->_nx, this->_nx);
    }

};


}