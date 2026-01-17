/**
 * @file TrajectoryIO.h
 * @brief Compact binary I/O for trajectory data
 * 
 * Binary format (.traj):
 *   Header (16 bytes):
 *     - Magic number: "TRAJ" (4 bytes)
 *     - Version: uint32_t (4 bytes) 
 *     - n_states: uint32_t (4 bytes)
 *     - dim_state: uint32_t (4 bytes)
 *   Data:
 *     - n_states * dim_state doubles (8 bytes each), row-major order
 *   Optional metadata (if version >= 2):
 *     - dt: double (8 bytes)
 *     - total_time: double (8 bytes)
 * 
 * Size comparison for 200 states x 12 dims:
 *   CSV:    ~58 KB (with 6 decimal places)
 *   Binary: ~19 KB (raw doubles) - 3x smaller
 * 
 * IMPORTANT: Eigen uses column-major storage by default. This library
 * writes data in row-major order for compatibility with Python/NumPy/JavaScript.
 * Use save_trajectory_binary_fast() with RowMajor matrices for better performance.
 */

#pragma once

#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <stdexcept>
#include <cstring>

namespace vimp {

constexpr char TRAJ_MAGIC[4] = {'T', 'R', 'A', 'J'};
constexpr uint32_t TRAJ_VERSION = 2;

// Row-major matrix type for efficient I/O
using MatrixXdRowMajor = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

struct TrajectoryHeader {
    char magic[4];
    uint32_t version;
    uint32_t n_states;
    uint32_t dim_state;
};

struct TrajectoryMetadata {
    double dt = 0.0;
    double total_time = 0.0;
};

/**
 * @brief Save trajectory to compact binary format
 * 
 * @param filepath Output file path (.traj extension recommended)
 * @param trajectory Matrix [n_states x dim_state]
 * @param dt Time step between states (optional)
 * @param total_time Total trajectory time (optional)
 */
inline void save_trajectory_binary(const std::string& filepath,
                                   const Eigen::MatrixXd& trajectory,
                                   double dt = 0.0,
                                   double total_time = 0.0) {
    std::ofstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file for writing: " + filepath);
    }
    
    // Write header
    TrajectoryHeader header;
    std::memcpy(header.magic, TRAJ_MAGIC, 4);
    header.version = TRAJ_VERSION;
    header.n_states = static_cast<uint32_t>(trajectory.rows());
    header.dim_state = static_cast<uint32_t>(trajectory.cols());
    
    file.write(reinterpret_cast<const char*>(&header), sizeof(header));
    
    // Write trajectory data (row-major)
    // Note: Eigen is column-major by default, so we must copy element by element
    for (int i = 0; i < trajectory.rows(); ++i) {
        for (int j = 0; j < trajectory.cols(); ++j) {
            double val = trajectory(i, j);
            file.write(reinterpret_cast<const char*>(&val), sizeof(double));
        }
    }
    
    // Write metadata (version 2+)
    file.write(reinterpret_cast<const char*>(&dt), sizeof(double));
    file.write(reinterpret_cast<const char*>(&total_time), sizeof(double));
    
    file.close();
}

/**
 * @brief Fast save using RowMajor matrix (single memcpy)
 * 
 * Use this when performance matters. Create your trajectory as:
 *   vimp::MatrixXdRowMajor trajectory(n_states, dim_state);
 */
inline void save_trajectory_binary_fast(const std::string& filepath,
                                        const MatrixXdRowMajor& trajectory,
                                        double dt = 0.0,
                                        double total_time = 0.0) {
    std::ofstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file for writing: " + filepath);
    }
    
    TrajectoryHeader header;
    std::memcpy(header.magic, TRAJ_MAGIC, 4);
    header.version = TRAJ_VERSION;
    header.n_states = static_cast<uint32_t>(trajectory.rows());
    header.dim_state = static_cast<uint32_t>(trajectory.cols());
    
    file.write(reinterpret_cast<const char*>(&header), sizeof(header));
    
    // RowMajor data can be written directly
    file.write(reinterpret_cast<const char*>(trajectory.data()),
               trajectory.size() * sizeof(double));
    
    file.write(reinterpret_cast<const char*>(&dt), sizeof(double));
    file.write(reinterpret_cast<const char*>(&total_time), sizeof(double));
    
    file.close();
}

/**
 * @brief Load trajectory from compact binary format
 * 
 * @param filepath Input file path
 * @param metadata Optional pointer to receive metadata
 * @return Trajectory matrix [n_states x dim_state]
 */
inline Eigen::MatrixXd load_trajectory_binary(const std::string& filepath,
                                              TrajectoryMetadata* metadata = nullptr) {
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file for reading: " + filepath);
    }
    
    // Read and validate header
    TrajectoryHeader header;
    file.read(reinterpret_cast<char*>(&header), sizeof(header));
    
    if (std::memcmp(header.magic, TRAJ_MAGIC, 4) != 0) {
        throw std::runtime_error("Invalid trajectory file format: " + filepath);
    }
    
    if (header.version > TRAJ_VERSION) {
        throw std::runtime_error("Unsupported trajectory version: " + std::to_string(header.version));
    }
    
    // Read trajectory data
    Eigen::MatrixXd trajectory(header.n_states, header.dim_state);
    
    // Read row-major data into column-major Eigen matrix
    for (uint32_t i = 0; i < header.n_states; ++i) {
        for (uint32_t j = 0; j < header.dim_state; ++j) {
            double val;
            file.read(reinterpret_cast<char*>(&val), sizeof(double));
            trajectory(i, j) = val;
        }
    }
    
    // Read metadata if version >= 2 and requested
    if (metadata && header.version >= 2) {
        file.read(reinterpret_cast<char*>(&metadata->dt), sizeof(double));
        file.read(reinterpret_cast<char*>(&metadata->total_time), sizeof(double));
    }
    
    return trajectory;
}

/**
 * @brief Fast load into RowMajor matrix (single memcpy)
 */
inline MatrixXdRowMajor load_trajectory_binary_fast(const std::string& filepath,
                                                     TrajectoryMetadata* metadata = nullptr) {
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file for reading: " + filepath);
    }
    
    TrajectoryHeader header;
    file.read(reinterpret_cast<char*>(&header), sizeof(header));
    
    if (std::memcmp(header.magic, TRAJ_MAGIC, 4) != 0) {
        throw std::runtime_error("Invalid trajectory file format: " + filepath);
    }
    
    MatrixXdRowMajor trajectory(header.n_states, header.dim_state);
    
    // RowMajor data can be read directly
    file.read(reinterpret_cast<char*>(trajectory.data()),
              trajectory.size() * sizeof(double));
    
    if (metadata && header.version >= 2) {
        file.read(reinterpret_cast<char*>(&metadata->dt), sizeof(double));
        file.read(reinterpret_cast<char*>(&metadata->total_time), sizeof(double));
    }
    
    return trajectory;
}

/**
 * @brief Save covariance matrices to binary format
 * 
 * Format:
 *   Header (12 bytes):
 *     - Magic: "COVR" (4 bytes)
 *     - n_states: uint32_t
 *     - dim_state: uint32_t
 *   Data:
 *     - n_states covariance matrices, each dim_state x dim_state doubles
 */
inline void save_covariances_binary(const std::string& filepath,
                                    const std::vector<Eigen::MatrixXd>& covariances) {
    if (covariances.empty()) {
        throw std::runtime_error("Empty covariance vector");
    }
    
    std::ofstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file for writing: " + filepath);
    }
    
    // Header
    char magic[4] = {'C', 'O', 'V', 'R'};
    uint32_t n_states = static_cast<uint32_t>(covariances.size());
    uint32_t dim_state = static_cast<uint32_t>(covariances[0].rows());
    
    file.write(magic, 4);
    file.write(reinterpret_cast<const char*>(&n_states), sizeof(uint32_t));
    file.write(reinterpret_cast<const char*>(&dim_state), sizeof(uint32_t));
    
    // Data
    for (const auto& cov : covariances) {
        file.write(reinterpret_cast<const char*>(cov.data()),
                   cov.size() * sizeof(double));
    }
    
    file.close();
}

/**
 * @brief Load covariance matrices from binary format
 */
inline std::vector<Eigen::MatrixXd> load_covariances_binary(const std::string& filepath) {
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file for reading: " + filepath);
    }
    
    // Read header
    char magic[4];
    uint32_t n_states, dim_state;
    
    file.read(magic, 4);
    if (std::memcmp(magic, "COVR", 4) != 0) {
        throw std::runtime_error("Invalid covariance file format");
    }
    
    file.read(reinterpret_cast<char*>(&n_states), sizeof(uint32_t));
    file.read(reinterpret_cast<char*>(&dim_state), sizeof(uint32_t));
    
    // Read data
    std::vector<Eigen::MatrixXd> covariances(n_states);
    for (uint32_t i = 0; i < n_states; ++i) {
        covariances[i].resize(dim_state, dim_state);
        file.read(reinterpret_cast<char*>(covariances[i].data()),
                  dim_state * dim_state * sizeof(double));
    }
    
    return covariances;
}

/**
 * @brief Save full GVIMP result (mean + covariance) to single file
 * 
 * Format:
 *   Header (20 bytes):
 *     - Magic: "GVIM" (4 bytes)
 *     - Version: uint32_t
 *     - n_states: uint32_t
 *     - dim_state: uint32_t
 *     - flags: uint32_t (bit 0: has_covariance, bit 1: sparse_covariance)
 *   Trajectory: n_states * dim_state doubles
 *   Metadata: dt, total_time (16 bytes)
 *   Covariance (if flag set): 
 *     - Dense: n_states * dim_state * dim_state doubles (block diagonal only)
 *     - Or full joint covariance
 */
inline void save_gvimp_result(const std::string& filepath,
                              const Eigen::VectorXd& mean,
                              int n_states,
                              int dim_state,
                              double dt,
                              double total_time,
                              const Eigen::MatrixXd* block_covariances = nullptr) {
    std::ofstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file for writing: " + filepath);
    }
    
    // Header
    char magic[4] = {'G', 'V', 'I', 'M'};
    uint32_t version = 1;
    uint32_t ns = static_cast<uint32_t>(n_states);
    uint32_t ds = static_cast<uint32_t>(dim_state);
    uint32_t flags = block_covariances ? 1 : 0;
    
    file.write(magic, 4);
    file.write(reinterpret_cast<const char*>(&version), sizeof(uint32_t));
    file.write(reinterpret_cast<const char*>(&ns), sizeof(uint32_t));
    file.write(reinterpret_cast<const char*>(&ds), sizeof(uint32_t));
    file.write(reinterpret_cast<const char*>(&flags), sizeof(uint32_t));
    
    // Trajectory (reshape mean to matrix and write)
    file.write(reinterpret_cast<const char*>(mean.data()), mean.size() * sizeof(double));
    
    // Metadata
    file.write(reinterpret_cast<const char*>(&dt), sizeof(double));
    file.write(reinterpret_cast<const char*>(&total_time), sizeof(double));
    
    // Block diagonal covariances (only diagonal blocks, not full matrix)
    if (block_covariances) {
        file.write(reinterpret_cast<const char*>(block_covariances->data()),
                   block_covariances->size() * sizeof(double));
    }
    
    file.close();
}

/**
 * @brief Helper to convert optimizer output to trajectory matrix
 */
inline Eigen::MatrixXd mean_to_trajectory(const Eigen::VectorXd& mean, 
                                          int n_states, 
                                          int dim_state) {
    Eigen::MatrixXd trajectory(n_states, dim_state);
    for (int i = 0; i < n_states; ++i) {
        trajectory.row(i) = mean.segment(i * dim_state, dim_state).transpose();
    }
    return trajectory;
}

/**
 * @brief Helper to save optimizer result directly
 */
template<typename Optimizer>
void save_optimizer_result(const std::string& filepath,
                           const Optimizer& optimizer,
                           int n_states,
                           int dim_state,
                           double total_time) {
    auto [mean, precision] = optimizer.get_mu_precision();
    Eigen::MatrixXd trajectory = mean_to_trajectory(mean, n_states, dim_state);
    double dt = total_time / (n_states - 1);
    save_trajectory_binary(filepath, trajectory, dt, total_time);
}

} // namespace vimp