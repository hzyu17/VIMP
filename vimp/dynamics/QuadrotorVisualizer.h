/**
 * @file QuadrotorVisualizer.h
 * @brief 3D Quadrotor trajectory visualization using OpenGL/GLFW
 * 
 * Dependencies:
 *   - GLFW3: Window management
 *   - GLEW: OpenGL extensions
 *   - Eigen3: Linear algebra
 * 
 * Install on Ubuntu:
 *   sudo apt-get install libglfw3-dev libglew-dev libeigen3-dev
 * 
 * Compile with:
 *   g++ -std=c++17 your_main.cpp -lglfw -lGLEW -lGL -lGLU -o visualizer
 */

#pragma once

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <Eigen/Dense>

#include <vector>
#include <cmath>
#include <iostream>
#include <chrono>
#include <thread>
#include <functional>

namespace vimp {

/**
 * @brief Camera for 3D scene navigation
 */
class Camera3D {
public:
    Eigen::Vector3d position{5.0, 5.0, 5.0};
    Eigen::Vector3d target{0.0, 0.0, 0.0};
    Eigen::Vector3d up{0.0, 0.0, 1.0};
    
    double fov = 45.0;
    double near_plane = 0.1;
    double far_plane = 100.0;
    
    // Orbital camera parameters
    double distance = 10.0;
    double azimuth = 45.0;    // degrees
    double elevation = 30.0;  // degrees
    
    void update_from_orbit() {
        double az_rad = azimuth * M_PI / 180.0;
        double el_rad = elevation * M_PI / 180.0;
        
        position.x() = target.x() + distance * std::cos(el_rad) * std::cos(az_rad);
        position.y() = target.y() + distance * std::cos(el_rad) * std::sin(az_rad);
        position.z() = target.z() + distance * std::sin(el_rad);
    }
    
    void orbit(double d_azimuth, double d_elevation) {
        azimuth += d_azimuth;
        elevation = std::clamp(elevation + d_elevation, -89.0, 89.0);
        update_from_orbit();
    }
    
    void zoom(double factor) {
        distance = std::clamp(distance * factor, 1.0, 50.0);
        update_from_orbit();
    }
    
    void pan(double dx, double dy) {
        double az_rad = azimuth * M_PI / 180.0;
        Eigen::Vector3d right(-std::sin(az_rad), std::cos(az_rad), 0);
        target += right * dx + up * dy;
        update_from_orbit();
    }
};

/**
 * @brief Quadrotor 3D model parameters
 */
struct QuadrotorModel {
    double arm_length = 0.25;
    double arm_width = 0.02;
    double body_radius = 0.08;
    double body_height = 0.04;
    double rotor_radius = 0.1;
    double rotor_height = 0.01;
    
    // Colors (RGB, 0-1)
    Eigen::Vector3f body_color{0.2f, 0.2f, 0.8f};
    Eigen::Vector3f arm_color{0.3f, 0.3f, 0.3f};
    Eigen::Vector3f rotor_color{0.8f, 0.2f, 0.2f};
    Eigen::Vector3f front_marker_color{0.0f, 1.0f, 0.0f};
};

/**
 * @brief Main visualizer class
 */
class QuadrotorVisualizer {
public:
    QuadrotorVisualizer(int width = 1280, int height = 720, const std::string& title = "Quadrotor Visualizer")
        : width_(width), height_(height), title_(title) {
        camera_.update_from_orbit();
    }
    
    ~QuadrotorVisualizer() {
        if (window_) {
            glfwDestroyWindow(window_);
        }
        glfwTerminate();
    }
    
    bool initialize() {
        if (!glfwInit()) {
            std::cerr << "Failed to initialize GLFW" << std::endl;
            return false;
        }
        
        glfwWindowHint(GLFW_SAMPLES, 4);  // Anti-aliasing
        window_ = glfwCreateWindow(width_, height_, title_.c_str(), nullptr, nullptr);
        if (!window_) {
            std::cerr << "Failed to create GLFW window" << std::endl;
            glfwTerminate();
            return false;
        }
        
        glfwMakeContextCurrent(window_);
        glfwSetWindowUserPointer(window_, this);
        
        // Set callbacks
        glfwSetScrollCallback(window_, scroll_callback);
        glfwSetMouseButtonCallback(window_, mouse_button_callback);
        glfwSetCursorPosCallback(window_, cursor_pos_callback);
        glfwSetKeyCallback(window_, key_callback);
        
        if (glewInit() != GLEW_OK) {
            std::cerr << "Failed to initialize GLEW" << std::endl;
            return false;
        }
        
        // OpenGL settings
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        glEnable(GL_COLOR_MATERIAL);
        glEnable(GL_NORMALIZE);
        glEnable(GL_MULTISAMPLE);
        
        glClearColor(0.9f, 0.9f, 0.95f, 1.0f);
        
        // Light setup
        GLfloat light_pos[] = {5.0f, 5.0f, 10.0f, 1.0f};
        GLfloat light_ambient[] = {0.3f, 0.3f, 0.3f, 1.0f};
        GLfloat light_diffuse[] = {0.8f, 0.8f, 0.8f, 1.0f};
        glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
        glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
        glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
        
        return true;
    }
    
    /**
     * @brief Set the trajectory to visualize
     * @param trajectory Matrix [N x 12], each row is a state
     * @param dt Time step between states (for animation speed)
     */
    void set_trajectory(const Eigen::MatrixXd& trajectory, double dt = 0.1) {
        trajectory_ = trajectory;
        dt_ = dt;
        current_frame_ = 0;
        
        // Compute trajectory bounds for camera
        if (trajectory_.rows() > 0) {
            Eigen::Vector3d min_pos = trajectory_.block(0, 0, trajectory_.rows(), 3).colwise().minCoeff();
            Eigen::Vector3d max_pos = trajectory_.block(0, 0, trajectory_.rows(), 3).colwise().maxCoeff();
            Eigen::Vector3d center = (min_pos + max_pos) / 2.0;
            double extent = (max_pos - min_pos).norm();
            
            camera_.target = center;
            camera_.distance = std::max(extent * 1.5, 5.0);
            camera_.update_from_orbit();
        }
    }
    
    /**
     * @brief Add obstacle spheres to the scene
     */
    void add_obstacle(const Eigen::Vector3d& center, double radius, 
                      const Eigen::Vector3f& color = Eigen::Vector3f(0.5f, 0.5f, 0.5f)) {
        obstacles_.push_back({center, radius, color});
    }
    
    /**
     * @brief Clear all obstacles
     */
    void clear_obstacles() {
        obstacles_.clear();
    }
    
    /**
     * @brief Run the visualization loop
     */
    void run() {
        if (!window_) {
            std::cerr << "Window not initialized. Call initialize() first." << std::endl;
            return;
        }
        
        auto last_time = std::chrono::high_resolution_clock::now();
        double accumulated_time = 0.0;
        
        while (!glfwWindowShouldClose(window_)) {
            auto current_time = std::chrono::high_resolution_clock::now();
            double frame_time = std::chrono::duration<double>(current_time - last_time).count();
            last_time = current_time;
            
            // Update animation
            if (playing_ && trajectory_.rows() > 0) {
                accumulated_time += frame_time * playback_speed_;
                if (accumulated_time >= dt_) {
                    accumulated_time = 0.0;
                    current_frame_++;
                    if (current_frame_ >= trajectory_.rows()) {
                        if (loop_) {
                            current_frame_ = 0;
                        } else {
                            current_frame_ = trajectory_.rows() - 1;
                            playing_ = false;
                        }
                    }
                }
            }
            
            render();
            
            glfwSwapBuffers(window_);
            glfwPollEvents();
            
            // Cap frame rate
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
    }
    
    /**
     * @brief Render a single frame (for custom render loops)
     */
    void render() {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        setup_camera();
        draw_grid();
        draw_axes();
        draw_obstacles();
        draw_trajectory_path();
        
        if (trajectory_.rows() > 0 && current_frame_ < trajectory_.rows()) {
            draw_quadrotor(trajectory_.row(current_frame_));
        }
        
        draw_hud();
    }
    
    // Playback controls
    void play() { playing_ = true; }
    void pause() { playing_ = false; }
    void toggle_play() { playing_ = !playing_; }
    void reset() { current_frame_ = 0; }
    void set_frame(int frame) { current_frame_ = std::clamp(frame, 0, (int)trajectory_.rows() - 1); }
    void set_playback_speed(double speed) { playback_speed_ = speed; }
    void set_loop(bool loop) { loop_ = loop; }
    
    // Visualization options
    void set_show_trajectory(bool show) { show_trajectory_ = show; }
    void set_show_grid(bool show) { show_grid_ = show; }
    void set_show_axes(bool show) { show_axes_ = show; }
    
    Camera3D& camera() { return camera_; }
    QuadrotorModel& model() { return model_; }
    
private:
    void setup_camera() {
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        
        double aspect = static_cast<double>(width_) / height_;
        double fov_rad = camera_.fov * M_PI / 180.0;
        double top = camera_.near_plane * std::tan(fov_rad / 2.0);
        double right = top * aspect;
        glFrustum(-right, right, -top, top, camera_.near_plane, camera_.far_plane);
        
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(camera_.position.x(), camera_.position.y(), camera_.position.z(),
                  camera_.target.x(), camera_.target.y(), camera_.target.z(),
                  camera_.up.x(), camera_.up.y(), camera_.up.z());
    }
    
    void draw_grid() {
        if (!show_grid_) return;
        
        glDisable(GL_LIGHTING);
        glColor3f(0.7f, 0.7f, 0.7f);
        glLineWidth(1.0f);
        
        glBegin(GL_LINES);
        for (int i = -10; i <= 10; ++i) {
            glVertex3f(i, -10, 0);
            glVertex3f(i, 10, 0);
            glVertex3f(-10, i, 0);
            glVertex3f(10, i, 0);
        }
        glEnd();
        
        glEnable(GL_LIGHTING);
    }
    
    void draw_axes() {
        if (!show_axes_) return;
        
        glDisable(GL_LIGHTING);
        glLineWidth(2.0f);
        
        glBegin(GL_LINES);
        // X axis - red
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(0, 0, 0);
        glVertex3f(1, 0, 0);
        // Y axis - green
        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 1, 0);
        // Z axis - blue
        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 1);
        glEnd();
        
        glEnable(GL_LIGHTING);
    }
    
    void draw_trajectory_path() {
        if (!show_trajectory_ || trajectory_.rows() < 2) return;
        
        glDisable(GL_LIGHTING);
        glLineWidth(2.0f);
        
        // Draw full trajectory in gray
        glColor3f(0.5f, 0.5f, 0.5f);
        glBegin(GL_LINE_STRIP);
        for (int i = 0; i < trajectory_.rows(); ++i) {
            glVertex3d(trajectory_(i, 0), trajectory_(i, 1), trajectory_(i, 2));
        }
        glEnd();
        
        // Draw traversed portion in blue
        if (current_frame_ > 0) {
            glColor3f(0.2f, 0.4f, 0.8f);
            glLineWidth(3.0f);
            glBegin(GL_LINE_STRIP);
            for (int i = 0; i <= current_frame_; ++i) {
                glVertex3d(trajectory_(i, 0), trajectory_(i, 1), trajectory_(i, 2));
            }
            glEnd();
        }
        
        glEnable(GL_LIGHTING);
    }
    
    void draw_obstacles() {
        GLUquadric* quad = gluNewQuadric();
        
        for (const auto& obs : obstacles_) {
            glPushMatrix();
            glTranslated(obs.center.x(), obs.center.y(), obs.center.z());
            glColor3f(obs.color.x(), obs.color.y(), obs.color.z());
            gluSphere(quad, obs.radius, 32, 32);
            glPopMatrix();
        }
        
        gluDeleteQuadric(quad);
    }
    
    /**
     * @brief Draw the quadrotor at a given state
     * @param state [x, y, z, φ, θ, ψ, vx, vy, vz, p, q, r]
     */
    void draw_quadrotor(const Eigen::VectorXd& state) {
        double x = state(0), y = state(1), z = state(2);
        double phi = state(3), theta = state(4), psi = state(5);
        
        glPushMatrix();
        
        // Translate to position
        glTranslated(x, y, z);
        
        // Apply ZYX Euler rotation (convert to OpenGL order)
        glRotated(psi * 180.0 / M_PI, 0, 0, 1);    // Yaw
        glRotated(theta * 180.0 / M_PI, 0, 1, 0);  // Pitch
        glRotated(phi * 180.0 / M_PI, 1, 0, 0);    // Roll
        
        draw_quadrotor_body();
        
        glPopMatrix();
    }
    
    void draw_quadrotor_body() {
        GLUquadric* quad = gluNewQuadric();
        
        // Central body (cylinder)
        glColor3f(model_.body_color.x(), model_.body_color.y(), model_.body_color.z());
        glPushMatrix();
        glTranslated(0, 0, -model_.body_height / 2);
        gluCylinder(quad, model_.body_radius, model_.body_radius, model_.body_height, 32, 1);
        // Top cap
        glTranslated(0, 0, model_.body_height);
        gluDisk(quad, 0, model_.body_radius, 32, 1);
        // Bottom cap
        glTranslated(0, 0, -model_.body_height);
        glRotated(180, 1, 0, 0);
        gluDisk(quad, 0, model_.body_radius, 32, 1);
        glPopMatrix();
        
        // Arms and rotors (+ configuration)
        glColor3f(model_.arm_color.x(), model_.arm_color.y(), model_.arm_color.z());
        
        // Arm angles for + configuration: 0°, 90°, 180°, 270°
        double arm_angles[] = {0, 90, 180, 270};
        
        for (int i = 0; i < 4; ++i) {
            glPushMatrix();
            glRotated(arm_angles[i], 0, 0, 1);
            
            // Draw arm
            glPushMatrix();
            glColor3f(model_.arm_color.x(), model_.arm_color.y(), model_.arm_color.z());
            glTranslated(model_.arm_length / 2, 0, 0);
            glScaled(model_.arm_length, model_.arm_width, model_.arm_width);
            draw_cube();
            glPopMatrix();
            
            // Draw rotor at end of arm
            glPushMatrix();
            glTranslated(model_.arm_length, 0, model_.rotor_height / 2);
            
            // Front rotor marker (green)
            if (i == 0) {
                glColor3f(model_.front_marker_color.x(), model_.front_marker_color.y(), model_.front_marker_color.z());
            } else {
                glColor3f(model_.rotor_color.x(), model_.rotor_color.y(), model_.rotor_color.z());
            }
            
            gluCylinder(quad, model_.rotor_radius, model_.rotor_radius, model_.rotor_height, 32, 1);
            glTranslated(0, 0, model_.rotor_height);
            gluDisk(quad, 0, model_.rotor_radius, 32, 1);
            glPopMatrix();
            
            glPopMatrix();
        }
        
        // Draw body frame axes
        glDisable(GL_LIGHTING);
        glLineWidth(2.0f);
        double axis_len = model_.arm_length * 0.5;
        glBegin(GL_LINES);
        // Body X - red
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(0, 0, 0);
        glVertex3f(axis_len, 0, 0);
        // Body Y - green
        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, axis_len, 0);
        // Body Z - blue
        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, axis_len);
        glEnd();
        glEnable(GL_LIGHTING);
        
        gluDeleteQuadric(quad);
    }
    
    void draw_cube() {
        glBegin(GL_QUADS);
        // Front
        glNormal3f(0, 0, 1);
        glVertex3f(-0.5f, -0.5f, 0.5f);
        glVertex3f(0.5f, -0.5f, 0.5f);
        glVertex3f(0.5f, 0.5f, 0.5f);
        glVertex3f(-0.5f, 0.5f, 0.5f);
        // Back
        glNormal3f(0, 0, -1);
        glVertex3f(-0.5f, -0.5f, -0.5f);
        glVertex3f(-0.5f, 0.5f, -0.5f);
        glVertex3f(0.5f, 0.5f, -0.5f);
        glVertex3f(0.5f, -0.5f, -0.5f);
        // Top
        glNormal3f(0, 1, 0);
        glVertex3f(-0.5f, 0.5f, -0.5f);
        glVertex3f(-0.5f, 0.5f, 0.5f);
        glVertex3f(0.5f, 0.5f, 0.5f);
        glVertex3f(0.5f, 0.5f, -0.5f);
        // Bottom
        glNormal3f(0, -1, 0);
        glVertex3f(-0.5f, -0.5f, -0.5f);
        glVertex3f(0.5f, -0.5f, -0.5f);
        glVertex3f(0.5f, -0.5f, 0.5f);
        glVertex3f(-0.5f, -0.5f, 0.5f);
        // Right
        glNormal3f(1, 0, 0);
        glVertex3f(0.5f, -0.5f, -0.5f);
        glVertex3f(0.5f, 0.5f, -0.5f);
        glVertex3f(0.5f, 0.5f, 0.5f);
        glVertex3f(0.5f, -0.5f, 0.5f);
        // Left
        glNormal3f(-1, 0, 0);
        glVertex3f(-0.5f, -0.5f, -0.5f);
        glVertex3f(-0.5f, -0.5f, 0.5f);
        glVertex3f(-0.5f, 0.5f, 0.5f);
        glVertex3f(-0.5f, 0.5f, -0.5f);
        glEnd();
    }
    
    void draw_hud() {
        // Switch to 2D for HUD
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        glOrtho(0, width_, 0, height_, -1, 1);
        
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
        
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        
        // Draw frame counter background
        glColor4f(0.0f, 0.0f, 0.0f, 0.5f);
        glBegin(GL_QUADS);
        glVertex2f(10, height_ - 60);
        glVertex2f(200, height_ - 60);
        glVertex2f(200, height_ - 10);
        glVertex2f(10, height_ - 10);
        glEnd();
        
        // Note: For actual text rendering, you'd need a text library like FreeType
        // Here we just draw indicators
        
        // Play/Pause indicator
        glColor3f(1.0f, 1.0f, 1.0f);
        if (playing_) {
            // Pause icon (two bars)
            glBegin(GL_QUADS);
            glVertex2f(20, height_ - 50);
            glVertex2f(28, height_ - 50);
            glVertex2f(28, height_ - 20);
            glVertex2f(20, height_ - 20);
            glVertex2f(35, height_ - 50);
            glVertex2f(43, height_ - 50);
            glVertex2f(43, height_ - 20);
            glVertex2f(35, height_ - 20);
            glEnd();
        } else {
            // Play icon (triangle)
            glBegin(GL_TRIANGLES);
            glVertex2f(20, height_ - 50);
            glVertex2f(45, height_ - 35);
            glVertex2f(20, height_ - 20);
            glEnd();
        }
        
        // Progress bar
        double progress = trajectory_.rows() > 0 ? 
            static_cast<double>(current_frame_) / (trajectory_.rows() - 1) : 0.0;
        
        glColor3f(0.3f, 0.3f, 0.3f);
        glBegin(GL_QUADS);
        glVertex2f(60, height_ - 40);
        glVertex2f(190, height_ - 40);
        glVertex2f(190, height_ - 30);
        glVertex2f(60, height_ - 30);
        glEnd();
        
        glColor3f(0.2f, 0.6f, 1.0f);
        glBegin(GL_QUADS);
        glVertex2f(60, height_ - 40);
        glVertex2f(60 + 130 * progress, height_ - 40);
        glVertex2f(60 + 130 * progress, height_ - 30);
        glVertex2f(60, height_ - 30);
        glEnd();
        
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LIGHTING);
        
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
        glPopMatrix();
    }
    
    // ==================== Callbacks ====================
    
    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
        auto* vis = static_cast<QuadrotorVisualizer*>(glfwGetWindowUserPointer(window));
        vis->camera_.zoom(1.0 - yoffset * 0.1);
    }
    
    static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
        auto* vis = static_cast<QuadrotorVisualizer*>(glfwGetWindowUserPointer(window));
        
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            vis->left_mouse_down_ = (action == GLFW_PRESS);
        }
        if (button == GLFW_MOUSE_BUTTON_RIGHT) {
            vis->right_mouse_down_ = (action == GLFW_PRESS);
        }
        if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
            vis->middle_mouse_down_ = (action == GLFW_PRESS);
        }
        
        glfwGetCursorPos(window, &vis->last_mouse_x_, &vis->last_mouse_y_);
    }
    
    static void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos) {
        auto* vis = static_cast<QuadrotorVisualizer*>(glfwGetWindowUserPointer(window));
        
        double dx = xpos - vis->last_mouse_x_;
        double dy = ypos - vis->last_mouse_y_;
        
        if (vis->left_mouse_down_) {
            vis->camera_.orbit(-dx * 0.5, dy * 0.5);
        }
        if (vis->middle_mouse_down_ || vis->right_mouse_down_) {
            vis->camera_.pan(-dx * 0.01, dy * 0.01);
        }
        
        vis->last_mouse_x_ = xpos;
        vis->last_mouse_y_ = ypos;
    }
    
    static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
        if (action != GLFW_PRESS && action != GLFW_REPEAT) return;
        
        auto* vis = static_cast<QuadrotorVisualizer*>(glfwGetWindowUserPointer(window));
        
        switch (key) {
            case GLFW_KEY_SPACE:
                vis->toggle_play();
                break;
            case GLFW_KEY_R:
                vis->reset();
                break;
            case GLFW_KEY_LEFT:
                vis->set_frame(vis->current_frame_ - 1);
                break;
            case GLFW_KEY_RIGHT:
                vis->set_frame(vis->current_frame_ + 1);
                break;
            case GLFW_KEY_UP:
                vis->playback_speed_ = std::min(vis->playback_speed_ * 1.5, 10.0);
                break;
            case GLFW_KEY_DOWN:
                vis->playback_speed_ = std::max(vis->playback_speed_ / 1.5, 0.1);
                break;
            case GLFW_KEY_L:
                vis->loop_ = !vis->loop_;
                break;
            case GLFW_KEY_T:
                vis->show_trajectory_ = !vis->show_trajectory_;
                break;
            case GLFW_KEY_G:
                vis->show_grid_ = !vis->show_grid_;
                break;
            case GLFW_KEY_A:
                vis->show_axes_ = !vis->show_axes_;
                break;
            case GLFW_KEY_ESCAPE:
            case GLFW_KEY_Q:
                glfwSetWindowShouldClose(window, GLFW_TRUE);
                break;
        }
    }
    
    // ==================== Members ====================
    GLFWwindow* window_ = nullptr;
    int width_, height_;
    std::string title_;
    
    Camera3D camera_;
    QuadrotorModel model_;
    
    Eigen::MatrixXd trajectory_;
    double dt_ = 0.1;
    int current_frame_ = 0;
    
    struct Obstacle {
        Eigen::Vector3d center;
        double radius;
        Eigen::Vector3f color;
    };
    std::vector<Obstacle> obstacles_;
    
    // Playback state
    bool playing_ = false;
    bool loop_ = true;
    double playback_speed_ = 1.0;
    
    // Visualization options
    bool show_trajectory_ = true;
    bool show_grid_ = true;
    bool show_axes_ = true;
    
    // Mouse state
    bool left_mouse_down_ = false;
    bool right_mouse_down_ = false;
    bool middle_mouse_down_ = false;
    double last_mouse_x_ = 0, last_mouse_y_ = 0;
};

} // namespace vimp