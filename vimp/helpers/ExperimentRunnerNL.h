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
            {}

    void read_boundary_conditions(const rapidxml::xml_node<>* paramNode, PGCSParams& params){
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

        VectorXd m0(params.nx()), mT(params.nx());
        MatrixXd Sig0(params.nx(), params.nx()), SigT(params.nx(), params.nx());
        m0 << start_x, start_y, start_phi, start_vx, start_vy, start_vphi;
        mT << goal_x, goal_y, goal_phi, goal_vx, goal_vy, goal_vphi;

        params.set_m0(m0);
        params.set_mT(mT);

    }

    void run_one_exp(int exp, PGCSParams& param) override{
        rapidxml::file<> xmlFile(this->_config_file.data()); // Default template is char
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());
        
        std::string ExpNodeName = "Experiment" + std::to_string(exp);

        char * c_expname = ExpNodeName.data();
        rapidxml::xml_node<>* paramNode = doc.first_node(c_expname);
        
        this->read_boundary_conditions(paramNode, param);

        MatrixXd A0(param.nx(), param.nx()), B0(param.nx(), param.nu()), a0(param.nx(), 1);
        A0.setZero(); B0.setZero(); a0.setZero();
        std::shared_ptr<ConstantVelDynamics> pdyn{new ConstantVelDynamics(param.nx(), param.nu(), param.nt())};
        A0 = pdyn->A0();
        B0 = pdyn->B0();
        a0 = pdyn->a0();
        
        PGCSOptimizer pgcs_nolinear_sdf(A0, a0, B0, pdyn, param);

        using NominalHistory = std::tuple<std::vector<Matrix3D>, std::vector<Matrix3D>>;
        std::tuple<MatrixXd, MatrixXd, NominalHistory> res_Kd;
        // res_Kd = pgcs_lin_sdf.optimize();
        res_Kd = pgcs_nolinear_sdf.backtrack();

        MatrixXd Kt(param.nx()*param.nx(), param.nt()), dt(param.nx(), param.nt());
        Kt = std::get<0>(res_Kd);
        dt = std::get<1>(res_Kd);

        NominalHistory ztSigt;
        ztSigt = std::get<2>(res_Kd);
        Matrix3D h_zt, h_Sigt;
        h_zt = this->_ei.vec2mat3d(std::get<0>(ztSigt));
        h_Sigt = this->_ei.vec2mat3d(std::get<1>(ztSigt));

        MatrixXd zk_star(param.nx(), param.nt()), Sk_star(param.nx()*param.nx(), param.nt());
        zk_star = pgcs_nolinear_sdf.zkt();
        Sk_star = pgcs_nolinear_sdf.Sigkt();

        std::string saving_prefix = static_cast<std::string>(paramNode->first_node("saving_prefix")->value());

        this->m_io.saveData(saving_prefix + std::string{"zk_sdf.csv"}, zk_star);
        this->m_io.saveData(saving_prefix + std::string{"Sk_sdf.csv"}, Sk_star);

        this->m_io.saveData(saving_prefix + std::string{"zk_history.csv"}, h_zt);
        this->m_io.saveData(saving_prefix + std::string{"Sk_history.csv"}, h_Sigt);

        this->m_io.saveData(saving_prefix + std::string{"Kt_sdf.csv"}, Kt);
        this->m_io.saveData(saving_prefix + std::string{"dt_sdf.csv"}, dt);

        pgcs_nolinear_sdf.save_costs(saving_prefix + std::string{"costs.csv"});

    }

};


}