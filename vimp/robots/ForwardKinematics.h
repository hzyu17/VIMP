#pragma once
#include <Eigen/Dense>
#include <vector>
#include <string>

using namespace Eigen;

enum DHType { Classical=0, Modified=1 };

class ForwardKinematics{
    public:
      // Denavit-Hartenberg (DH) variables
      VectorXd _a;
      VectorXd _alpha;
      VectorXd _d;
      VectorXd _theta_bias;
    
      // Body sphere variables
      int _num_spheres;
      int _num_joints;
      Eigen::VectorXi _frames;
      Eigen::MatrixXd _centers; // Note: centers must be constructed in row-major form (each center belongs to a row)
    
      DHType _dh_type;
    
    private:
    
    public:
        // Constructors/Destructor
        ForwardKinematics() {}
    
        ForwardKinematics(const Eigen::VectorXd& a, const Eigen::VectorXd& alpha,
                          const Eigen::VectorXd& d, const Eigen::VectorXd& theta_bias,
                          const Eigen::VectorXi& frames, const Eigen::MatrixXd& centers, DHType dh_type) :
          _a(a), _alpha(alpha), _d(d), _theta_bias(theta_bias), _frames(frames), _centers(centers), 
          _num_joints(a.size()), _num_spheres(frames.size()), _dh_type(dh_type)
          {}
    
        ~ForwardKinematics() {}
    
        inline MatrixXd compute_sphere_centers(const VectorXd& theta) const {
            // Precompute DH matrices for all joints.
            MatrixXd dh_mats(4, 4 * _num_joints);
            MatrixXd pose(_num_spheres, 3);
            VectorXd theta_full = theta;

            if (theta.size() < _num_joints) {
                theta_full = VectorXd::Zero(_num_joints);
                theta_full.head(theta.size()) = theta;
            } 

            precompute_dh_matrices(theta_full, dh_mats);
    
            for (int i = 0; i < _num_spheres; ++i) {
                int frame = _frames(i);
                Vector3d center = _centers.col(i);
                Vector3d pos;
                forward_kinematics(dh_mats, frame, center, pos);
                pose.row(i) = pos;
            }
            return pose;
        }

        inline MatrixXd compute_sphere_centers_batched(const Eigen::MatrixXd& thetas) const {
            const int N = thetas.rows();
            MatrixXd pts(_num_spheres * N, 3);
            pts.setZero();

            for (int i = 0; i < N; ++i) {
                VectorXd theta = thetas.row(i);
                MatrixXd pose = compute_sphere_centers(theta);
                pts.block(i * _num_spheres, 0, _num_spheres, 3) = pose;
            }
            return pts;
        }
    
        inline void forward_kinematics(const MatrixXd& dh_mats, int frame, const Vector3d& center, Vector3d& pos) const {
            Matrix4d T = dh_mats.block(0, frame * 4, 4, 4);
            Vector4d center_homogeneous(center(0), center(1), center(2), 1.0);
            
            // Compute the transformed position using the transformation matrix
            Vector4d transformed = T * center_homogeneous;
            // Extract the position from the transformed vector
            pos = transformed.segment(0, 3);
        }
    
        inline void precompute_dh_matrices(const VectorXd& theta, MatrixXd& dh_mats) const {
            Matrix4d T = Matrix4d::Identity();
            Matrix4d mul_result = Matrix4d::Zero();
            
            for (int i = 0; i < _num_joints; i++) {
                double th = theta(i) + _theta_bias(i);
                Matrix4d dh_mat = Matrix4d::Zero();
                if (_dh_type == Classical)
                    dh_mat = dh_matrix(i, th);
                else
                    dh_mat = dh_matrix_modified(i, th);

                mul_result = T * dh_mat;
                T = mul_result;
                dh_mats.block(0, i * 4, 4, 4) = mul_result;
            }
        }
    
        inline Matrix4d dh_matrix(int i, double theta) const {
            double ct = cos(theta);
            double st = sin(theta);
            double ca = cos(_alpha(i));
            double sa = sin(_alpha(i));
            Matrix4d mat;
            mat <<  ct,  -st * ca,  st * sa,   _a(i) * ct,
                    st,  ct * ca,   -ct * sa,  _a(i) * st,
                    0,   sa,        ca,        _d(i),
                    0,   0,         0,         1;
            return mat;
        }
    
        inline Matrix4d dh_matrix_modified(int i, double theta) const {
            double ct = std::cos(theta);
            double st = std::sin(theta);
            double ca = std::cos(_alpha(i));
            double sa = std::sin(_alpha(i));
            double ai = _a(i);
            double di = _d(i);
            Matrix4d mat;
            mat << ct,       -st,      0,    ai,
                   st * ca,  ct * ca,  -sa,  -di * sa,
                   st * sa,  ct * sa,  ca,   di * ca,
                   0,        0,        0,    1;
            return mat;
        }

        const Eigen::VectorXd& a() const { return _a; }
        const Eigen::VectorXd& alpha() const { return _alpha; }
        const Eigen::VectorXd& d() const { return _d; }
        const Eigen::VectorXd& theta_bias() const { return _theta_bias; }
        const Eigen::VectorXi& frames() const { return _frames; }
        const Eigen::MatrixXd& centers() const { return _centers; }
        int num_joints() const { return _num_joints; }
        int num_spheres() const { return _num_spheres; }
        DHType dh_type() const { return _dh_type; }
    
    };