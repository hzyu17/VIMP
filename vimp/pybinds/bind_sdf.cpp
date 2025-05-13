#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>    // Eigen ⇄ numpy support
#include <pybind11/stl.h>      // std::vector/std::string support

#include "maps/SignedDistanceField.h"  // SDF header
#include "maps/PlanarSDF.h"  // PlanarSDF header

namespace py = pybind11;

PYBIND11_MODULE(bind_SDF, m) {
    m.doc() = "Python bindings for SignedDistanceField and planar SDF classes";

    py::class_<SignedDistanceField, SignedDistanceField::shared_ptr>(m, "SignedDistanceField")
        // -- constructors --
        .def(py::init<>(), "Default constructor")
        .def(py::init<const Eigen::Vector3d&, double, const std::vector<Eigen::MatrixXd>&>(),
             py::arg("origin"), py::arg("cell_size"), py::arg("data"),
             "Construct from origin, cell size, and a vector of field layers")
        .def(py::init<const Eigen::Vector3d&, double, int, int, int>(),
             py::arg("origin"), py::arg("cell_size"),
             py::arg("field_rows"), py::arg("field_cols"), py::arg("field_z"),
             "Construct empty field of given dimensions")

        // -- methods --
        .def("initFieldData", &SignedDistanceField::initFieldData,
             py::arg("z_idx"), py::arg("field_layer"),
             "Set the matrix layer at index z_idx")

        .def("getSignedDistance", &SignedDistanceField::getSignedDistance,
             py::arg("point"),
             "Query the interpolated signed distance at a 3D point")

        .def("getSignedDistanceBatched",
             &SignedDistanceField::getSignedDistanceBatched, py::arg("point_vec"),
                "Query the interpolated signed distance at a batch of 3D points")

        .def("convertPoint3toCell", &SignedDistanceField::convertPoint3toCell,
             py::arg("point"),
             "Convert a world-coordinate point to fractional cell indices")

        .def("convertCelltoPoint2", &SignedDistanceField::convertCelltoPoint2,
             py::arg("cell"),
             "Convert cell indices back to world-coordinate point")

        .def("loadSDF", &SignedDistanceField::loadSDF,
             py::arg("filename"),
             "Load SDF from file (XML, Bin, or JSON)")

        .def("saveSDF", &SignedDistanceField::saveSDF,
             py::arg("filename"),
             "Save SDF to binary file")

        // -- read‐only properties / accessors --
        .def_property_readonly("origin", &SignedDistanceField::origin,
             "The origin of the grid")
        .def("x_count", &SignedDistanceField::x_count,
             "Number of columns")
        .def("y_count", &SignedDistanceField::y_count,
             "Number of rows")
        .def("z_count", &SignedDistanceField::z_count,
             "Number of layers")
        .def("cell_size", &SignedDistanceField::cell_size,
             "Grid cell size")
        .def("raw_data", &SignedDistanceField::raw_data,
             py::return_value_policy::reference_internal,
             "Access to the underlying vector of Eigen::MatrixXd");

     
     py::class_<PlanarSDF, PlanarSDF::shared_ptr>(m, "PlanarSDF")
          // -- constructors --
          .def(py::init<>(), "Default constructor")
          .def(py::init<const Eigen::Vector2d&, double, const Eigen::MatrixXd&>(),
               py::arg("origin"), py::arg("cell_size"), py::arg("data"),
               "Construct from origin, cell size, and a 2D field layer")

          // -- methods --
          .def("getSignedDistance", &PlanarSDF::getSignedDistance,
               py::arg("point"),
               "Query the interpolated signed distance at a 2D point")
          
          .def("getSignedDistanceBatched",
               &PlanarSDF::getSignedDistanceBatched,
               py::arg("point_vec"),
               "Query the interpolated signed distance at a batch of 2D points")

          .def("getGradient", &PlanarSDF::getGradient,
               py::arg("point"),
               "Query the gradient at a 2D point")

          .def("convertPoint2toCell", &PlanarSDF::convertPoint2toCell,
               py::arg("point"),
               "Convert a world-coordinate point to fractional cell indices")

          .def("convertCelltoPoint2", &PlanarSDF::convertCelltoPoint2,
               py::arg("cell"),
               "Convert cell indices back to world-coordinate point")

          // -- read‐only properties / accessors --
          .def_property_readonly("origin", &PlanarSDF::origin,
               "The origin of the grid")
          .def("x_count", &PlanarSDF::x_count,
               "Number of columns")
          .def("y_count", &PlanarSDF::y_count,
               "Number of rows")
          .def("cell_size", &PlanarSDF::cell_size,
               "Grid cell size")
          .def("raw_data", &PlanarSDF::raw_data,
               py::return_value_policy::reference_internal,
               "Access to the underlying Eigen matrix");
}
