#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>   // Eigen <-> NumPy
#include <pybind11/stl.h>     // std::vector / std::string helpers

#include "robots/ForwardKinematics.h"

namespace py = pybind11;

PYBIND11_MODULE(bind_FK, m) {
    m.doc() = "Python bindings for ForwardKinematics";

    /* --- DHType enum ---------------------------------------------------- */
    py::enum_<DHType>(m, "DHType", py::arithmetic())
        .value("Classical", DHType::Classical)
        .value("Modified", DHType::Modified)
        .export_values();

    /* --- ForwardKinematics class --------------------------------------- */
    py::class_<ForwardKinematics, std::shared_ptr<ForwardKinematics>> (m, "ForwardKinematics")
        /* -- constructors -- */
        .def(py::init<>(), "Default constructor")
        .def(py::init< const Eigen::VectorXd&, const Eigen::VectorXd&,
                 const Eigen::VectorXd&, const Eigen::VectorXd&,
                 const Eigen::VectorXi&, const Eigen::MatrixXd&, DHType>(),
             py::arg("a"), py::arg("alpha"), py::arg("d"),
             py::arg("theta_bias"), py::arg("frames"),
             py::arg("centers"), py::arg("dh_type") = DHType::Modified,
             "Fully specified constructor")

        /* -- methods ----------------------------------------------------- */
        .def("compute_sphere_centers",
            &ForwardKinematics::compute_sphere_centers,
            py::arg("theta"),
            "Return a 3*N matrix with world-space positions of all collision spheres")
        
        .def("compute_sphere_centers_batched",
            &ForwardKinematics::compute_sphere_centers_batched,
            py::arg("thetas"),
            "Return a 3*N matrix with world-space positions of all collision spheres for a batch of joint angles")

        /* (Forward‑kinematics helpers are left unexposed; add if needed) */

        /* -- read‑only accessors ----------------------------------------- */
        .def_property_readonly("a",           &ForwardKinematics::a,
             "DH link lengths a_i")
        .def_property_readonly("alpha",       &ForwardKinematics::alpha,
             "DH twists alpha_i")
        .def_property_readonly("d",           &ForwardKinematics::d,
             "DH offsets d_i")
        .def_property_readonly("theta_bias",  &ForwardKinematics::theta_bias,
             "Joint_angle biases theta_i")
        .def_property_readonly("frames",      &ForwardKinematics::frames,
             "Sphere → joint index mapping")
        .def_property_readonly("centers",     &ForwardKinematics::centers,
             py::return_value_policy::reference_internal,
             "Sphere centers expressed in their local joint frames")
        .def_property_readonly("num_joints",  &ForwardKinematics::num_joints,
             "Number of actuated joints")
        .def_property_readonly("num_spheres", &ForwardKinematics::num_spheres,
             "Number of collision spheres");
}