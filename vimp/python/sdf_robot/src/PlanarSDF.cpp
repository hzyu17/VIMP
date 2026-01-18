/**
 *  @file  PlanarSDF.cpp
 *  @brief Re-definition of signed distance class
 *         https://github.com/gtrll/gpmp2/blob/main/gpmp2/obstacle/PlanarSDF.h
 *         Removed Boost dependency for convinence in python wrapping.
 *  @date  Dec 19, 2023
 **/

#include <pybind11/pybind11.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <cstring>
#include <tuple>
#include <memory>
#include <stdexcept>
#include <limits>

#include <Eigen/Dense>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

namespace py = pybind11;


/**
 * Signed distance field use Matrix as data type
 * Matrix represent the X (col) & Y (row) dimension
 */
class PlanarSDF {

public:
  // index and float_index is <row, col>
  typedef std::tuple<size_t, size_t> index;
  typedef std::tuple<double, double> float_index;
  typedef std::shared_ptr<PlanarSDF> shared_ptr;

  // Sentinel value for invalid index
  static inline constexpr double INVALID_INDEX = std::numeric_limits<double>::max();
  // Return value when point is out of bounds
  static inline constexpr double OUT_OF_BOUNDS_DISTANCE = 1000.0;

private:
  Eigen::Vector2d origin_;
  // geometry setting of signed distance field
  size_t field_rows_, field_cols_;
  double cell_size_;
  Eigen::MatrixXd data_;

public:
  /// constructor
  PlanarSDF() : field_rows_(0), field_cols_(0), cell_size_(0.0) {}

  /// constructor with data
  PlanarSDF(const Eigen::Vector2d& origin, double cell_size, const Eigen::MatrixXd& data) :
      origin_(origin), field_rows_(data.rows()), field_cols_(data.cols()),
      cell_size_(cell_size), data_(data) {}

  ~PlanarSDF() {}


  /// give a point, search for signed distance field and (optional) gradient
  /// return signed distance
  inline double getSignedDistance(const Eigen::Vector2d& point) const {
    const float_index pidx = convertPoint2toCell(point);
    if (std::get<0>(pidx) == INVALID_INDEX) {
      return OUT_OF_BOUNDS_DISTANCE;
    }
    return signed_distance(pidx);
  }

  inline Eigen::Vector2d getGradient(const Eigen::Vector2d& point) const {
    const float_index pidx = convertPoint2toCell(point);
    if (std::get<0>(pidx) == INVALID_INDEX) {
      Eigen::Vector2d zero_vec;
      zero_vec.setZero();
      return zero_vec;
    }

    const Eigen::Vector2d g_idx = gradient(pidx);
    // convert gradient of index to gradient of metric unit
    return Eigen::Vector2d(g_idx(1), g_idx(0)) / cell_size_;
  }


  /// convert between point and cell corrdinate
  inline float_index convertPoint2toCell(const Eigen::Vector2d& point) const {
    // check point range
    if (point.x() < origin_.x() || point.x() > (origin_.x() + (field_cols_-1.0)*cell_size_) ||
        point.y() < origin_.y() || point.y() > (origin_.y() + (field_rows_-1.0)*cell_size_)) {
        
      // Log warning for debugging
      std::cerr << "[PlanarSDF] Warning: Point out of bounds. point.x: " << point.x() 
                << "; point.y: " << point.y() 
                << " (valid range: x=[" << origin_.x() << ", " << origin_.x() + (field_cols_-1.0)*cell_size_ << "]"
                << ", y=[" << origin_.y() << ", " << origin_.y() + (field_rows_-1.0)*cell_size_ << "])"
                << std::endl;

      return std::make_tuple(INVALID_INDEX, INVALID_INDEX);
    }

    const double col = (point.x() - origin_.x()) / cell_size_;
    const double row = (point.y() - origin_.y()) / cell_size_;
    return std::make_tuple(row, col);
  }

  inline Eigen::Vector2d convertCelltoPoint2(const float_index& cell) const {
    return origin_ + Eigen::Vector2d(
        std::get<1>(cell) * cell_size_,
        std::get<0>(cell) * cell_size_);
  }


  /// bilinear interpolation
  inline double signed_distance(const float_index& idx) const {
    const double lr = floor(std::get<0>(idx)), lc = floor(std::get<1>(idx));
    const double hr = lr + 1.0, hc = lc + 1.0;
    const size_t lri = static_cast<size_t>(lr), lci = static_cast<size_t>(lc),
                 hri = static_cast<size_t>(hr), hci = static_cast<size_t>(hc);
    return
        (hr-std::get<0>(idx))*(hc-std::get<1>(idx))*signed_distance(lri, lci) +
        (std::get<0>(idx)-lr)*(hc-std::get<1>(idx))*signed_distance(hri, lci) +
        (hr-std::get<0>(idx))*(std::get<1>(idx)-lc)*signed_distance(lri, hci) +
        (std::get<0>(idx)-lr)*(std::get<1>(idx)-lc)*signed_distance(hri, hci);
  }

  /// gradient operator for bilinear interpolation
  /// gradient regrads to float_index
  /// not numerical differentiable at index point
  inline Eigen::Vector2d gradient(const float_index& idx) const {

    const double lr = floor(std::get<0>(idx)), lc = floor(std::get<1>(idx));
    const double hr = lr + 1.0, hc = lc + 1.0;
    const size_t lri = static_cast<size_t>(lr), lci = static_cast<size_t>(lc),
        hri = static_cast<size_t>(hr), hci = static_cast<size_t>(hc);
    return Eigen::Vector2d(
        (hc-std::get<1>(idx)) * (signed_distance(hri, lci)-signed_distance(lri, lci)) +
        (std::get<1>(idx)-lc) * (signed_distance(hri, hci)-signed_distance(lri, hci)),

        (hr-std::get<0>(idx)) * (signed_distance(lri, hci)-signed_distance(lri, lci)) +
        (std::get<0>(idx)-lr) * (signed_distance(hri, hci)-signed_distance(hri, lci)));
  }

  // access
  inline double signed_distance(size_t r, size_t c) const {
    return data_(r, c);
  }

  const Eigen::Vector2d& origin() const { return origin_; }
  size_t x_count() const { return field_cols_; }
  size_t y_count() const { return field_rows_; }
  double cell_size() const { return cell_size_; }
  const Eigen::MatrixXd& raw_data() const { return data_; }

  /// Get the bounds of the SDF
  double x_min() const { return origin_.x(); }
  double x_max() const { return origin_.x() + (field_cols_ - 1.0) * cell_size_; }
  double y_min() const { return origin_.y(); }
  double y_max() const { return origin_.y() + (field_rows_ - 1.0) * cell_size_; }

  /// print
  void print(const std::string& str = "") const {
    std::cout << str;
    std::cout << "field origin:     [" << origin_.x() << ", " << origin_.y() << "]" << std::endl;
    std::cout << "field resolution: " << cell_size_ << std::endl;
    std::cout << "field size:       " << field_cols_ << " x " << field_rows_ << std::endl;
    std::cout << "x range:          [" << x_min() << ", " << x_max() << "]" << std::endl;
    std::cout << "y range:          [" << y_min() << ", " << y_max() << "]" << std::endl;
  }

};

PYBIND11_MODULE(libplanar_sdf, m) {
    py::class_<PlanarSDF>(m, "PlanarSDF")
        .def(py::init<const Eigen::Vector2d& , double , const Eigen::MatrixXd& >())
        .def("origin", &PlanarSDF::origin)
        .def("raw_data", &PlanarSDF::raw_data)
        .def("getSignedDistance", &PlanarSDF::getSignedDistance)
        .def("getGradient", &PlanarSDF::getGradient)
        .def("x_min", &PlanarSDF::x_min)
        .def("x_max", &PlanarSDF::x_max)
        .def("y_min", &PlanarSDF::y_min)
        .def("y_max", &PlanarSDF::y_max)
        .def("x_count", &PlanarSDF::x_count)
        .def("y_count", &PlanarSDF::y_count)
        .def("cell_size", &PlanarSDF::cell_size)
        .def("print", &PlanarSDF::print, py::arg("str") = "");
}