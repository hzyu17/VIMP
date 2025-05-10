#include <gpmp2/obstacle/getJacobianPlanarGP.h>
#include <gpmp2/obstacle/ObstacleCost.h>

using namespace std;
using namespace gtsam;


namespace gpmp2 {
/* ************************************************************************** */
template <class ROBOT, class GPINTER>
gtsam::Vector GetJacobianGP<ROBOT, GPINTER>::evaluateError(
        const Pose& conf1, const Velocity& vel1,
        const Pose& conf2, const Velocity& vel2,
        boost::optional<gtsam::Matrix&> H1, boost::optional<gtsam::Matrix&> H2,
        boost::optional<gtsam::Matrix&> H3, boost::optional<gtsam::Matrix&> H4) const {
    return conf1;

}

template <class ROBOT, class GPINTER>
gtsam::Matrix GetJacobianGP<ROBOT, GPINTER>::getJacobians(
        const Pose& conf1, const Velocity& vel1,
        const Pose& conf2, const Velocity& vel2,
        boost::optional<gtsam::Matrix&> H1, boost::optional<gtsam::Matrix&> H2,
        boost::optional<gtsam::Matrix&> H3, boost::optional<gtsam::Matrix&> H4) const {

    const bool use_H = (H1 || H2 || H3 || H4);

    // if Jacobians used, initialize Jerr_conf as zeros
    // size: arm_nr_points_ * DOF
    Matrix Jerr_conf = Matrix::Zero(robot_.nr_body_spheres(), robot_.dof());


    // get conf by interpolation, except last pose
    typename Robot::Pose conf;
    Matrix Jconf_c1, Jconf_c2, Jconf_v1, Jconf_v2;
    if (use_H)
        conf = GPbase_.interpolatePose(conf1, vel1, conf2, vel2, Jconf_c1, Jconf_v1, Jconf_c2, Jconf_v2);
    else
        conf = GPbase_.interpolatePose(conf1, vel1, conf2, vel2);


    // run forward kinematics of this configuration
    vector<Point3> sph_centers;
    vector<Matrix> J_px_jp;
    if (H1)
        robot_.sphereCenters(conf, sph_centers, J_px_jp);
    else
        robot_.sphereCenters(conf, sph_centers);


    // allocate cost vector
    Vector err(robot_.nr_body_spheres());

    // for each point on arm stick, get error
    for (size_t sph_idx = 0; sph_idx < robot_.nr_body_spheres(); sph_idx++) {

        const double total_eps = robot_.sphere_radius(sph_idx) + epsilon_;

        if (H1) {
            Matrix12 Jerr_point;
            const Point2 sph_center_2d(sph_centers[sph_idx].x(), sph_centers[sph_idx].y());
            err(sph_idx) = hingeLossObstacleCost(sph_center_2d, sdf_, total_eps, Jerr_point);

            // chain rules
            Jerr_conf.row(sph_idx) = Jerr_point * J_px_jp[sph_idx].topRows<2>();

        } else {
            const Point2 sph_center_2d(sph_centers[sph_idx].x(), sph_centers[sph_idx].y());
            err(sph_idx) = hingeLossObstacleCost(sph_center_2d, sdf_, total_eps);
        }
    }

    // update jacobians
    if (use_H)
        GPBase::updatePoseJacobians(Jerr_conf, Jconf_c1, Jconf_v1, Jconf_c2, Jconf_v2,
                                    H1, H2, H3, H4);

    gtsam::Matrix res_mat = Matrix::Zero(robot_.nr_body_spheres(), 8);
    if (use_H)
    {
        res_mat.block(0, 0, H1->rows(), H1->cols()) = *H1;
        res_mat.block(0, H1->cols()-1, H2->rows(), H2->cols()) = *H2;
        res_mat.block(0, 2*H1->cols()-1, H3->rows(), H3->cols()) = *H3;
        res_mat.block(0, 3*H1->cols()-1, H4->rows(), H4->cols()) = *H4;
    }

    return res_mat;
}
}