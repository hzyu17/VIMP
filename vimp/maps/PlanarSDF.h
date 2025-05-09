#include <Eigen/Dense>
#include <vector>
#include <string>
// #include <helpers/MatrixHelper.h>
#include <helpers/SerializeEigenMaps.h>

class PlanarSDF {

    public:
      // index and float_index is <row, col>
      typedef std::tuple<int, int> index;
      typedef Eigen::Vector2d float_index;
      typedef std::shared_ptr<PlanarSDF> shared_ptr;
    
    private:
      Eigen::Vector2d origin_;
      int field_rows_, field_cols_;
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
        return signed_distance(pidx);
      }
    
      inline Eigen::Vector2d getGradient(const Eigen::Vector2d& point) const {
        const float_index pidx = convertPoint2toCell(point);    
        const Eigen::Vector2d g_idx = gradient(pidx);
        // convert gradient of index to gradient of metric unit
        return Eigen::Vector2d(g_idx(1), g_idx(0)) / cell_size_;
      }
    
    
      /// convert between point and cell corrdinate
      inline float_index convertPoint2toCell(const Eigen::Vector2d& point) const {
        // check point range
        if (point.x() < origin_.x() || point.x() > (origin_.x() + (field_cols_-1.0)*cell_size_) ||
            point.y() < origin_.y() || point.y() > (origin_.y() + (field_rows_-1.0)*cell_size_)) {

            throw std::runtime_error("Querying SDF out of range");
        }
    
        const double col = (point.x() - origin_.x()) / cell_size_;
        const double row = (point.y() - origin_.y()) / cell_size_;
        return Vector2d(row, col);
      }
    
      inline Eigen::Vector2d convertCelltoPoint2(const float_index& cell) const {
        return origin_ + Eigen::Vector2d(
            cell(1) * cell_size_,
            cell(0) * cell_size_);
      }
    
    
      /// bilinear interpolation
      inline double signed_distance(const float_index& idx) const {
        const double lr = floor(idx(0)), lc = floor(idx(1));
        const double hr = lr + 1.0, hc = lc + 1.0;
        const int lri = static_cast<int>(lr), lci = static_cast<int>(lc),
                  hri = static_cast<int>(hr), hci = static_cast<int>(hc);
        return
            (hr-idx(0))*(hc-idx(1))*signed_distance(lri, lci) +
            (idx(0)-lr)*(hc-idx(1))*signed_distance(hri, lci) +
            (hr-idx(0))*(idx(1)-lc)*signed_distance(lri, hci) +
            (idx(0)-lr)*(idx(1)-lc)*signed_distance(hri, hci);
      }
    
      /// gradient operator for bilinear interpolation
      /// gradient regrads to float_index
      /// not numerical differentiable at index point
      inline Eigen::Vector2d gradient(const float_index& idx) const {
    
        const double lr = floor(idx(0)), lc = floor(idx(1));
        const double hr = lr + 1.0, hc = lc + 1.0;
        const int lri = static_cast<int>(lr), lci = static_cast<int>(lc),
            hri = static_cast<int>(hr), hci = static_cast<int>(hc);
        return Eigen::Vector2d(
            (hc-idx(1)) * (signed_distance(hri, lci)-signed_distance(lri, lci)) +
            (idx(1)-lc) * (signed_distance(hri, hci)-signed_distance(lri, hci)),
    
            (hr-idx(0)) * (signed_distance(lri, hci)-signed_distance(lri, lci)) +
            (idx(0)-lr) * (signed_distance(hri, hci)-signed_distance(hri, lci)));
      }
    
      // access
      inline double signed_distance(int r, int c) const {
        return data_(r, c);
      }
    
      const Eigen::Vector2d& origin() const { return origin_; }
      int x_count() const { return field_cols_; }
      int y_count() const { return field_rows_; }
      double cell_size() const { return cell_size_; }
      const Eigen::MatrixXd& raw_data() const { return data_; }
    };