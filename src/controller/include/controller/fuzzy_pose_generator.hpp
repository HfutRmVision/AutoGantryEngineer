#pragma once
#include <chrono>
#include <random>
#include <cmath>
#include <ostream>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include "moveit/task_constructor/stages.h"
#include "moveit/task_constructor/storage.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace mtc = moveit::task_constructor;

class FuzzyPoseGenerator : public mtc::stages::GeneratePose{
public:
    FuzzyPoseGenerator(const std::string& name) 
        : mtc::stages::GeneratePose(name), 
          pos_tol_(0.1), 
          ori_tol_(0.1), 
          sample_count_(10),
          generator_(std::chrono::system_clock::now().time_since_epoch().count()) {}

    void setTolerance(double position_tolerance, double orientation_tolerance){
        pos_tol_ = position_tolerance;
        ori_tol_ = orientation_tolerance;
    }

    void setSampleCount(int count){
        sample_count_ = count;
    }

    void compute() override{
        // 如果没有上游解，直接返回
        if (upstream_solutions_.empty()) {
            return;
        }

        if (!properties().hasProperty("pose")){
            throw std::runtime_error("Pose property not set for FuzzyPoseGenerator");
        }

        const geometry_msgs::msg::PoseStamped& pose = properties().get<geometry_msgs::msg::PoseStamped>("pose");
        std::uniform_real_distribution<double> pos_dist(-pos_tol_, pos_tol_);
        std::uniform_real_distribution<double> ori_dist(-ori_tol_, ori_tol_);

        // 只处理第一个上游解
        auto upstream_solution = upstream_solutions_.pop();
        const planning_scene::PlanningSceneConstPtr& scene = upstream_solution->end()->scene();
        
        // 生成样本
        for (int i = 0; i < sample_count_; ++i){
            geometry_msgs::msg::PoseStamped fuzzy_pose = pose;

            // Add position noise
            fuzzy_pose.pose.position.x += pos_dist(generator_);
            fuzzy_pose.pose.position.y += pos_dist(generator_);
            fuzzy_pose.pose.position.z += pos_dist(generator_);

            // Add orientation noise
            double d_roll = ori_dist(generator_);
            double d_pitch = ori_dist(generator_);
            std::uniform_real_distribution<double> yaw_dist(-M_PI/3, M_PI/3);
            double d_yaw = yaw_dist(generator_);

            tf2::Quaternion q_orig, q_delta, q_new;
            tf2::fromMsg(fuzzy_pose.pose.orientation, q_orig);
            q_delta.setRPY(d_roll, d_pitch, d_yaw);
            q_new = q_orig * q_delta;
            q_new.normalize();
            fuzzy_pose.pose.orientation = tf2::toMsg(q_new);

            // Create an InterfaceState with the scene from upstream
            mtc::InterfaceState state(scene);
            state.properties().set("target_pose", fuzzy_pose);

            // Calculate cost
            double pos_deviation = std::sqrt(
                std::pow(fuzzy_pose.pose.position.x - pose.pose.position.x, 2) +
                std::pow(fuzzy_pose.pose.position.y - pose.pose.position.y, 2) +
                std::pow(fuzzy_pose.pose.position.z - pose.pose.position.z, 2)
            );
            double ori_deviation = std::abs(d_roll) + std::abs(d_pitch);
            double cost = pos_deviation + ori_deviation;

            // Reload阶段避免“零调整”样本被优先选中：
            // 若几乎不调整，则施加大惩罚，迫使求解器优先尝试非零调整解。
            if (name().find("reloading") != std::string::npos) {
                constexpr double near_zero_adjustment_threshold = 1e-4;
                if (cost < near_zero_adjustment_threshold) {
                    cost += 1000.0;
                }
            }

            logPoseSampleToFile(i, fuzzy_pose, d_roll, d_pitch, d_yaw, pos_deviation, ori_deviation, cost);
            logPoseSampleToTerminal(i, fuzzy_pose, d_roll, d_pitch, d_yaw, cost);
                    
            spawn(std::move(state), cost);
        }
        
        while (!upstream_solutions_.empty()) {
            upstream_solutions_.pop();
        }
    }
    
    void reset() override {
        mtc::stages::GeneratePose::reset();
    }

protected:
    void onNewSolution(const mtc::SolutionBase& s) override {
        mtc::stages::GeneratePose::onNewSolution(s);
    }

private:
    void prepareFreshLogFileIfNeeded() const {
        if (log_initialized_) {
            return;
        }

        namespace fs = std::filesystem;
        const fs::path log_path = fs::path("/home/ooofieee/Desktop/ws_sim") / (name() + ".log");

        if (fs::exists(log_path)) {
            std::error_code ec;
            fs::remove(log_path, ec);
        }

        std::ofstream create_file(log_path, std::ios::out | std::ios::trunc);
        if (create_file.is_open()) {
            log_initialized_ = true;
        }
    }

    void logPoseSampleToFile(int sample_index,
                             const geometry_msgs::msg::PoseStamped& target_pose,
                             double d_roll,
                             double d_pitch,
                             double d_yaw,
                             double pos_deviation,
                             double ori_deviation,
                             double cost) const {
        namespace fs = std::filesystem;

        prepareFreshLogFileIfNeeded();

        const fs::path log_path = fs::path("/home/ooofieee/Desktop/ws_sim") / (name() + ".log");
        std::ofstream log_file(log_path, std::ios::app);
        if (!log_file.is_open()) {
            return;
        }

        log_file << std::fixed << std::setprecision(6)
                 << "sample=" << sample_index
                 << ", frame=" << target_pose.header.frame_id
                 << ", pos_m=("
                 << target_pose.pose.position.x << ", "
                 << target_pose.pose.position.y << ", "
                 << target_pose.pose.position.z << ")"
                 << ", quat=("
                 << target_pose.pose.orientation.x << ", "
                 << target_pose.pose.orientation.y << ", "
                 << target_pose.pose.orientation.z << ", "
                 << target_pose.pose.orientation.w << ")"
                 << ", d_rpy_rad=("
                 << d_roll << ", " << d_pitch << ", " << d_yaw << ")"
                 << ", pos_dev_m=" << pos_deviation
                 << ", ori_dev_rad=" << ori_deviation
                 << ", cost=" << cost
                 << '\n';
    }

    void logPoseSampleToTerminal(int sample_index,
                                 const geometry_msgs::msg::PoseStamped& target_pose,
                                 double d_roll,
                                 double d_pitch,
                                 double d_yaw,
                                 double cost) const {
        if (name().find("fuzzy push goal") == std::string::npos) {
            return;
        }

        std::cout << std::fixed << std::setprecision(6)
                  << "[FuzzyPoseGenerator][" << name() << "] "
                  << "sample=" << sample_index
                  << " pos=(" << target_pose.pose.position.x << ", "
                  << target_pose.pose.position.y << ", "
                  << target_pose.pose.position.z << ")"
                  << " quat=(" << target_pose.pose.orientation.x << ", "
                  << target_pose.pose.orientation.y << ", "
                  << target_pose.pose.orientation.z << ", "
                  << target_pose.pose.orientation.w << ")"
                  << " d_rpy_rad=(" << d_roll << ", " << d_pitch << ", " << d_yaw << ")"
                  << " cost=" << cost
                  << std::endl;
    }

    double pos_tol_;
    double ori_tol_;
    int sample_count_;
    std::default_random_engine generator_;
    mutable bool log_initialized_{ false };
};
