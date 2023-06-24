/**
 * @file run_experiment_template.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The template of running a motion planning experiment.
 * @version 0.1
 * @date 2023-04-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "ExperimentRunner.h"

using namespace Eigen;

namespace vimp{

template <typename PGCSOptimizer, typename Dynamics>
class ExperimentRunnerNL: public PGCSRunner<PGCSOptimizer>{
public:
    // PGCSRunner(){}
    virtual ~ExperimentRunnerNL(){}

    ExperimentRunnerNL(int nx, int nu, int num_exp, const std::string & config): 
            PGCSRunner<PGCSOptimizer>(nx, nu, num_exp, config)
            {
                rapidxml::file<> xmlFile(_config_file.data()); // Default template is char
                rapidxml::xml_document<> doc;
                doc.parse<0>(xmlFile.data());

                // Common parameters
                std::string CommonNodeName = "Commons";
                char * c_commons = CommonNodeName.data();
                rapidxml::xml_node<>* CommonNode = doc.first_node(c_commons);
                rapidxml::xml_node<>* commonParams = CommonNode->first_node("parameters");

                std::string field_file = static_cast<std::string>(commonParams->first_node("field_file")->value());

                MatrixXd field = m_io.load_csv(field_file);
                gtsam::Point2 origin(-20, -10);
                double cell_size = 0.1;
                _sdf = gpmp2::PlanarSDF(origin, cell_size, field);

            }

    void read_boundary_conditions(const rapidxml::xml_node<>* paramNode){
        double start_x = atof(paramNode->first_node("start_pos")->first_node("x")->value());
        double start_y = atof(paramNode->first_node("start_pos")->first_node("y")->value());
        double start_phi = atof(paramNode->first_node("start_pos")->first_node("phi")->value());
        double start_vx = atof(paramNode->first_node("start_pos")->first_node("vx")->value());
        double start_vy = atof(paramNode->first_node("start_pos")->first_node("vy")->value());
        double start_vphi = atof(paramNode->first_node("start_pos")->first_node("vphi")->value());

        double goal_x = atof(paramNode->first_node("goal_pos")->first_node("x")->value());
        double goal_y = atof(paramNode->first_node("goal_pos")->first_node("y")->value());
        double goal_phi = atof(paramNode->first_node("goal_pos")->first_node("phi")->value());
        double goal_vx = atof(paramNode->first_node("goal_pos")->first_node("vx")->value());
        double goal_vy = atof(paramNode->first_node("goal_pos")->first_node("vy")->value());
        double goal_vphi = atof(paramNode->first_node("goal_pos")->first_node("vphi")->value());

        _m0 << start_x, start_y, start_phi, start_vx, start_vy, start_vphi;
        _Sig0 = sig0 * Eigen::MatrixXd::Identity(_nx, _nx);

        _mT << goal_x, goal_y, goal_phi, goal_vx, goal_vy, goal_vphi;
        _SigT = sigT * Eigen::MatrixXd::Identity(_nx, _nx);
    }

    void run() override{
        rapidxml::file<> xmlFile(_config_file.data()); // Default template is char
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());

        for (int i=1; i<_num_exp+1; i++){
            std::string ExpNodeName = "Experiment" + std::to_string(i);
            char * c_expname = ExpNodeName.data();
            rapidxml::xml_node<>* ExpNode = doc.first_node(c_expname);
            rapidxml::xml_node<>* paramNode = ExpNode->first_node("parameters");            
            double sig_obs = atof(paramNode->first_node("cost_sigma")->value());
            
            double sig = _params.speed * _nt;

            // proximal gradient parameters
            double eps=0.01;

            read_boundary_conditions(paramNode);
            MatrixXd A0(_nx, _nx), B0(_nx, _nu), a0(_nx, 1);
            A0.setZero(); B0.setZero(); a0.setZero();
            
            std::shared_ptr<Dynamics> pdyn{new Dynamics(_nx, _nu, nt)};
            std::tuple<MatrixXd, MatrixXd, VectorXd, VectorXd> linearized_0 = pdyn->linearize_at(_m0, sig, A0, _Sig0);
            A0  = std::get<0>(linearized_0);
            B0  = std::get<1>(linearized_0);
            a0  = std::get<2>(linearized_0);
            PGCSOptimizer pgcs_nonlin_sdf(A0, a0, B0, _params, pdyn, _sdf);
            
            std::tuple<MatrixXd, MatrixXd> res_Kd;
            res_Kd = pgcs_nonlin_sdf.optimize();

            MatrixXd Kt(_nx*_nx, nt), dt(_nx, nt);
            Kt = std::get<0>(res_Kd);
            dt = std::get<1>(res_Kd);
            MatrixXd zk_star(_nx, nt), Sk_star(_nx*_nx, nt);
            zk_star = pgcs_nonlin_sdf.zkt();
            Sk_star = pgcs_nonlin_sdf.Sigkt();

            std::string saving_prefix = static_cast<std::string>(paramNode->first_node("saving_prefix")->value());
            m_io.saveData(saving_prefix + std::string{"zk_sdf.csv"}, zk_star);
            m_io.saveData(saving_prefix + std::string{"Sk_sdf.csv"}, Sk_star);

            m_io.saveData(saving_prefix + std::string{"Kt_sdf.csv"}, Kt);
            m_io.saveData(saving_prefix + std::string{"dt_sdf.csv"}, dt);

        }
    }

};


}