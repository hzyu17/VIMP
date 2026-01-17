/**
 * @file save_trajectory_example.cpp
 * @brief Example: Generate a figure-8 trajectory and save using TrajectoryIO
 * 
 * Compile:
 *   g++ -std=c++17 -O2 save_trajectory_example.cpp -o save_trajectory -I/usr/include/eigen3
 * 
 * Run:
 *   ./save_trajectory output.traj
 */

#include "dynamics/TrajectoryIO.h"
#include <iostream>
#include <cmath>

/**
 * @brief Generate a figure-8 (lemniscate) trajectory for 3D quadrotor
 * 
 * State: [x, y, z, phi, theta, psi, vx, vy, vz, p, q, r] (12 dims)
 * 
 * Matches the HTML demo exactly for consistency.
 */
Eigen::MatrixXd generate_figure8_trajectory(int n_states, double total_time) {
    Eigen::MatrixXd trajectory(n_states, 12);
    
    const double w = 2.0 * M_PI / total_time;
    
    for (int i = 0; i < n_states; ++i) {
        double t = static_cast<double>(i) * total_time / (n_states - 1);
        
        // Position (lemniscate) - matches HTML demo
        double x = 3.0 * std::sin(w * t);
        double y = 1.5 * std::sin(w * t) * std::cos(w * t);
        double z = 2.0 + 0.3 * std::sin(2.0 * w * t);
        
        // World velocity (for yaw computation)
        double vx = 3.0 * w * std::cos(w * t);
        double vy = 1.5 * w * (std::cos(w * t) * std::cos(w * t) - 
                               std::sin(w * t) * std::sin(w * t));
        
        // Attitude (Euler angles)
        double phi   = 0.1 * std::sin(2.0 * w * t);
        double theta = 0.05 * std::cos(w * t);
        double psi   = std::atan2(vy, vx);
        
        // Body-frame velocities
        double vx_body = std::sqrt(vx * vx + vy * vy);
        double vy_body = 0.0;
        double vz_body = 0.6 * w * std::cos(2.0 * w * t);
        
        // Angular rates
        double p = 0.2 * w * std::cos(2.0 * w * t);
        double q = -0.05 * w * std::sin(w * t);
        double r = w;
        
        trajectory.row(i) << x, y, z, phi, theta, psi, 
                             vx_body, vy_body, vz_body, p, q, r;
    }
    
    return trajectory;
}

int main(int argc, char** argv) {
    // Default output file
    std::string output_file = "figure8_trajectory.traj";
    if (argc > 1) {
        output_file = argv[1];
    }
    
    // Parameters
    const int n_states = 200;
    const double total_time = 10.0;  // seconds
    const double dt = total_time / (n_states - 1);
    
    std::cout << "Generating figure-8 trajectory..." << std::endl;
    std::cout << "  States: " << n_states << std::endl;
    std::cout << "  Duration: " << total_time << " s" << std::endl;
    std::cout << "  dt: " << dt << " s" << std::endl;
    
    // Generate trajectory
    Eigen::MatrixXd trajectory = generate_figure8_trajectory(n_states, total_time);
    
    // Print some stats
    std::cout << "\nTrajectory bounds:" << std::endl;
    std::cout << "  X: [" << trajectory.col(0).minCoeff() << ", " 
              << trajectory.col(0).maxCoeff() << "]" << std::endl;
    std::cout << "  Y: [" << trajectory.col(1).minCoeff() << ", " 
              << trajectory.col(1).maxCoeff() << "]" << std::endl;
    std::cout << "  Z: [" << trajectory.col(2).minCoeff() << ", " 
              << trajectory.col(2).maxCoeff() << "]" << std::endl;
    
    // Save to binary format
    std::cout << "\nSaving to: " << output_file << std::endl;
    vimp::save_trajectory_binary(output_file, trajectory, dt, total_time);
    
    // Verify by reloading
    std::cout << "Verifying..." << std::endl;
    vimp::TrajectoryMetadata metadata;
    Eigen::MatrixXd loaded = vimp::load_trajectory_binary(output_file, &metadata);
    
    double max_error = (loaded - trajectory).cwiseAbs().maxCoeff();
    std::cout << "  Loaded shape: " << loaded.rows() << " x " << loaded.cols() << std::endl;
    std::cout << "  dt: " << metadata.dt << " s" << std::endl;
    std::cout << "  total_time: " << metadata.total_time << " s" << std::endl;
    std::cout << "  Max reconstruction error: " << max_error << std::endl;
    
    std::cout << "\nDone! Visualize with:" << std::endl;
    std::cout << "  python quadrotor_visualizer.py " << output_file << " -i" << std::endl;
    
    return 0;
}