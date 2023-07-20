/**
 * test_planar_quad.cpp
 * Brief: Test the linearization of the planar quadrotor dynamics.
 * Author: Hongzhe Yu
*/

#define STRING(x) #x
#define XSTRING(x) STRING(x)

#include "covariance_steering/PGCSPlanarQuadSDF.h"
#include "helpers/EigenWrapper.h"
#include "helpers/ExperimentRunner.h"

using namespace Eigen;
using namespace vimp;

int main(){
    int nt = 500;
    int nx = 6;
    int nu = 2;

    EigenWrapper _ei;
    VectorXd m_0(6);
    m_0 << 4,4,0,0,0,0;
    MatrixXd Sig_0 = MatrixXd::Identity(6,6)*0.001;
    MatrixXd A0 = MatrixXd::Identity(6,6);

    PlanarQuadDynamics planar_quad(6, 2, nt);

    // Linearization at initial state m_0
    using AtBtatnTr = std::tuple<MatrixXd, MatrixXd, VectorXd, VectorXd>;
    AtBtatnTr res = planar_quad.linearize_at(m_0, A0, Sig_0);

    MatrixXd A1 = std::get<0>(res);
    MatrixXd B1 = std::get<1>(res);
    VectorXd a1 = std::get<2>(res);
    VectorXd nTr1 = std::get<3>(res);

    // _ei.print_matrix(A1, "At");
    // _ei.print_matrix(B1, "Bt");
    // _ei.print_matrix(a1, "at");

    Matrix3D At = _ei.replicate3d(A1, nt);
    Matrix3D Bt = _ei.replicate3d(B1, nt);
    Matrix3D at = _ei.replicate3d(a1, nt);

    // propagating the nominals 
    std::string source_root{XSTRING(SOURCE_ROOT)};
    std::string config_file{source_root+"/configs/pgcs/planar_quad_map2.xml"};

    PGCSRunnerNonLinear<PGCSPlanarQuadSDF, PlanarQuadDynamics> runner(6, 2, 1, config_file);
    PGCSParams params;
    runner.read_config(params);

    std::shared_ptr<PlanarQuadDynamics> p_dyn = std::make_shared<PlanarQuadDynamics>(planar_quad);
    PGCSPlanarQuadSDF pgcs_quad(A1, a1, B1, p_dyn, params);

    using ztSt = std::tuple<Matrix3D, Matrix3D>;
    ztSt propagate_res =  pgcs_quad.propagate_nominal(At, at, Bt, m_0, Sig_0);
    
    Matrix3D ztnew(nx, 1, nt), Sigt_new(nx, nx, nt);
    ztnew = std::get<0>(propagate_res);
    Sigt_new = std::get<1>(propagate_res);
    
    // MatrixIO mio;
    // mio.saveData(source_root+"/At.csv", At);
    // mio.saveData(source_root+"/at.csv", at);
    // mio.saveData(source_root+"/Bt.csv", Bt);

    // mio.saveData(source_root+"/zkt_new.csv", ztnew);
    // mio.saveData(source_root+"/Sigt_new.csv", Sigt_new);

    Matrix3D Qkt(nx, nx, nt), rkt(nx, 1, nt);

    using LinearizationResult = std::tuple<LinearDynamics, Matrix3D>;
    LinearizationResult lin_res = p_dyn->linearize(ztnew, At, Sigt_new);

    LinearDynamics lin_dyn = std::get<0>(lin_res);
    Matrix3D nTrt = std::get<1>(lin_res);
    Matrix3D hAt = lin_dyn.At();
    Matrix3D hat = lin_dyn.at();
    Matrix3D hBt = lin_dyn.Bt();

    double step_size = 1e-6;

    std::tuple<Matrix3D, Matrix3D> res_Qr = pgcs_quad.update_Qrk_NL(ztnew, Sigt_new, At, at, hBt, hAt, hat, nTrt, step_size);

    Matrix3D Qt = std::get<0>(res_Qr);
    Matrix3D rt = std::get<1>(res_Qr);

    MatrixXd Aprior{At/(1+step_size) + step_size*hAt/(1+step_size)};
    MatrixXd aprior{at/(1+step_size) + step_size*hat/(1+step_size)};
    // Matrix3D Aprior   = At/(1+step_size) + step_size*hAt/(1+step_size);
    // Matrix3D aprior   = at/(1+step_size) + step_size*hat/(1+step_size);

    LinearCSResult linearcs_res = pgcs_quad.solve_linearCS_return(Aprior, Bt, aprior, Qt, rt);

    Matrix3D Kt = std::get<0>(linearcs_res);
    Matrix3D dt = std::get<1>(linearcs_res);

    MatrixIO mio;
    mio.saveData(source_root+"/Kt.csv", Kt);
    mio.saveData(source_root+"/dt.csv", dt);

    mio.saveData(source_root+"/Qt.csv", Qt);
    mio.saveData(source_root+"/rt.csv", rt);

    mio.saveData(source_root+"/nTrt.csv", nTrt);

    mio.saveData(source_root+"/Aprior.csv", Aprior);
    
}