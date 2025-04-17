#include <cmath>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <csignal>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "ros_ign_interfaces/msg/dataframe.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

#include "multi-uav-trace-mbzirc-gt/Utils.h"

#include "nlohmann/json.hpp"
#include "Eigen/Dense"

using namespace Eigen;
using json = nlohmann::json;

using namespace std::chrono_literals;

#define pi acos(-1.0)

class UAVCommNode : public rclcpp::Node {
public:
    UAVCommNode(const std::string &id)
        : Node("uav_comm_" + id), id_(id) {
        vel_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("uav_" + id_ + "/cmd_vel", 10);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "uav_" + id_ + "/imu/data", 10,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                last_imu_ = *msg;
            });

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "uav_" + id_ + "/pose/groundtruth", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                last_pose_ = *msg;
            });

        clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
            "clock", 10,
            [this](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                last_clock_ = *msg;
            });
    }

    double get_time() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return last_clock_.clock.sec + last_clock_.clock.nanosec * 1e-9;
    }

    void spin() {
        rclcpp::spin(shared_from_this());
    }

    void publish_velocity(const Eigen::Vector3d &velocity) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = velocity.x();
        cmd.linear.y = velocity.y();
        cmd.linear.z = velocity.z();
        vel_cmd_pub_->publish(cmd);
    }

    Eigen::Vector3d get_last_pose() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return Eigen::Vector3d(
            last_pose_.pose.position.x,
            last_pose_.pose.position.y,
            last_pose_.pose.position.z);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;

    std::string id_;

    mutable std::mutex data_mutex_;
    sensor_msgs::msg::Imu last_imu_;
    geometry_msgs::msg::PoseStamped last_pose_;
    rosgraph_msgs::msg::Clock last_clock_;
};

class Task {
public:
    Task(const json settings)
        : id_(settings["id"]), uav_comm_(std::make_shared<UAVCommNode>(id_)), stop_flag_(false) {
        
        global_offset_ = Eigen::Vector3d(-1500.0, 0.0, get_my_height());

        auto arr = settings["prepare_point"].get<std::vector<double>>();
        prepare_point_ = Eigen::Vector3d(arr.data()) + global_offset_;
        search_height_ = prepare_point_.z();

        spin_thread_ = std::thread([this]() {
            uav_comm_->spin();
        });

        task_begin_time_ = get_time();
        task_time_ = get_time() - task_begin_time_;
        state_begin_time_ = get_time();
        state_time_ = get_time() - state_begin_time_;
    }

    ~Task() {
        stop();
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
    }

    double get_time() const {
        return uav_comm_->get_time();
    }

    void run() {
        while (!stop_flag_) {
            std::this_thread::sleep_for(100ms);
            update();
        }
    }

    void runOnce() {
        if (!stop_flag_) {
            update();
        }
    }

    void stop() {
        stop_flag_ = true;
    }

    bool isReady() {
        return ready_to_perform_;
    }

    void startPerform() {
        transition_to(State::PERFORM);
    }

    bool isInPerform() {
        return current_state_ == State::PERFORM;
    }

    bool timeToEndPerform() {
        if (!isInPerform()) return false;
        return state_time_ > 50.0;
    }

    void endPerform() {
        if (!isInPerform()) return;
        transition_to(State::BACK);
    }

private:
    enum class State {
        INIT,
        TAKEOFF,
        PREPARE,
        PERFORM,
        BACK,
        LAND
    };

    void update() {
        Eigen::Vector3d current_pose = uav_comm_->get_last_pose();
        task_time_ = get_time() - task_begin_time_;
        state_time_ = get_time() - state_begin_time_;

        switch (current_state_) {
            case State::INIT:
                if (current_pose.norm() > 1e-3) {
                    spawn_point_ = current_pose;
                    takeoff_point_ = spawn_point_ + Eigen::Vector3d(0.0, 0.0, 20.0);
                    transition_to(State::TAKEOFF);
                }
                break;
            case State::TAKEOFF:
                control_to_point(current_pose, takeoff_point_);
                if (is_at_point(current_pose, takeoff_point_)) {
                    transition_to(State::PREPARE);
                }
                break;
            case State::PREPARE:
                control_to_point(current_pose, prepare_point_);
                if (is_at_point(current_pose, prepare_point_)) {
                    ready_to_perform_ = true;
                }
                break;
            case State::PERFORM:
                set_trace_point(state_time_);
                // desired_point_ (3d) = trace_point_(2d) + global_offset_(3d)
                desired_point_ = global_offset_;
                desired_point_.x() += trace_point_.x();
                desired_point_.y() += trace_point_.y();
                control_to_point(current_pose, desired_point_, 0.4);
                if (timeToEndPerform()) {
                    transition_to(State::BACK);
                }
                break;
            case State::BACK:
                control_to_point(current_pose, takeoff_point_);
                if (is_at_point(current_pose, takeoff_point_)) {
                    transition_to(State::LAND);
                }
                break;
            case State::LAND:
                control_to_point(current_pose, spawn_point_);
                break;
        }

        uav_comm_->publish_velocity(velocity_cmd_);

        std::lock_guard<std::mutex> lock(log_mutex_);
        printf(
            "#%s | Time: %.2lf | Position: %.2f, %.2f, %.2f | State: %s(%.2lf s) | Control: (%.2f, %.2f, %.2f) | System Time: %.5lf\n",
            id_.c_str(), task_time_,
            current_pose.x(), current_pose.y(), current_pose.z(),
            state_to_string(current_state_).c_str(), state_time_,
            velocity_cmd_.x(), velocity_cmd_.y(), velocity_cmd_.z(),
            get_time()
        );
    }

    double get_my_height() {
        return 1.0 * std::stoi(id_) + 10;
    }

    void set_trace_point(double t) {
        center_point_ = Eigen::Vector2d(
            t,
            3 * sin(pi * t / 50.0)
        );
        int id = std::stoi(id_);
        if (id <= 5) {
            trace_point_ = center_point_ + Eigen::Vector2d(
                3.0 * cos(pi * t / 50.0 + 2.0 * (id - 1) / 10 * 2 * pi),
                3.0 * sin(pi * t / 50.0 + 2.0 * (id - 1) / 10 * 2 * pi)
            );
        }
        else {
            trace_point_ = center_point_ + Eigen::Vector2d(
                5.0 * cos(pi * t / 50.0 + (2.0 * id - 1) / 10 * 2 * pi),
                5.0 * sin(pi * t / 50.0 + (2.0 * id - 1) / 10 * 2 * pi)
            );
        }
    }

    void control_to_point(const Eigen::Vector3d &current_pose, const Eigen::Vector3d &target_pose, double kp = 0.2) {
        Eigen::Vector3d delta = target_pose - current_pose;
        
        double max_speed = 5.0;
        Eigen::Vector3d vel = kp * delta;

        if (vel.norm() > max_speed) {
            vel = vel.normalized() * max_speed;
        }
        
        velocity_cmd_ = vel;
    }

    void velocity2dControl(const Eigen::Vector3d &current_pose, const Eigen::Vector2d &velocity) {
        double kp = 0.2;
        velocity_cmd_ = Eigen::Vector3d(velocity.x(), velocity.y(), kp * (search_height_ - current_pose.z()));
    }

    void setVelocity2dCmd(const Eigen::Vector2d &velocity) {
        velocity_2d_cmd_ = velocity;
    }

    bool is_at_point(const Eigen::Vector3d &current_pose, const Eigen::Vector3d &target_pose, double tolerance = 0.5) {
        return (current_pose - target_pose).norm() < tolerance;
    }

    void transition_to(State new_state) {
        current_state_ = new_state;
        std::lock_guard<std::mutex> lock(log_mutex_);
        state_begin_time_ = get_time();
    }

    std::string state_to_string(State state) {
        switch (state) {
            case State::INIT: return "INIT";
            case State::TAKEOFF: return "TAKEOFF";
            case State::PREPARE: return BLUE + "PREPARE" + RESET;
            case State::PERFORM: return GREEN + "PERFORM" + RESET;
            case State::BACK: return "BACK";
            case State::LAND: return "LAND";
            default: return "UNKNOWN";
        }
    }

    std::string id_;
    std::shared_ptr<UAVCommNode> uav_comm_;
    std::atomic<bool> stop_flag_;
    std::thread spin_thread_;
    State current_state_ = State::INIT;

    double search_height_;

    Eigen::Vector3d spawn_point_;
    Eigen::Vector3d takeoff_point_;
    Eigen::Vector3d prepare_point_;
    
    Eigen::Vector3d velocity_cmd_ = Eigen::Vector3d::Zero();
    Eigen::Vector2d velocity_2d_cmd_ = Eigen::Vector2d::Zero();

    std::mutex log_mutex_;

    Eigen::Vector3d global_offset_ = Eigen::Vector3d::Zero();

    Eigen::Vector2d center_point_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d trace_point_ = Eigen::Vector2d::Zero();

    Eigen::Vector3d desired_point_ = Eigen::Vector3d::Zero();

    bool ready_to_perform_ = false;

    double task_begin_time_ = 0.0;
    double task_time_ = 0.0;
    double state_begin_time_ = 0.0;
    double state_time_ = 0.0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, [](int) {
        rclcpp::shutdown();
    });

    int num = 10;
    nlohmann::json settings = {
        {
            {"id", "1"},
            {"prepare_point", {2.17, 2.20, 0.0}}
        },
        {
            {"id", "2"},
            {"prepare_point", {-2.61, -1.22, 0.0}}
        },
        {
            {"id", "3"},
            {"prepare_point", {-3.46, 2.15, 0.0}}
        },
        {
            {"id", "4"},
            {"prepare_point", {-5.38, 2.01, 0.0}}
        },
        {
            {"id", "5"},
            {"prepare_point", {0.10, -2.26, 0.0}}
        },
        {
            {"id", "6"},
            {"prepare_point", {7.05, 7.62, 0.0}}
        },
        {
            {"id", "7"},
            {"prepare_point", {2.21, 8.70, 0.0}}
        },
        {
            {"id", "8"},
            {"prepare_point", {-8.30, 3.78, 0.0}}
        },
        {
            {"id", "9"},
            {"prepare_point", {3.03, -4.42, 0.0}}
        },
        {
            {"id", "10"},
            {"prepare_point", {5.91, 0.41, 0.0}}
        }
    };
    
    std::vector<std::unique_ptr<Task>> tasks;

    for (auto &[key, value] : settings.items()) {
        tasks.emplace_back(std::make_unique<Task>(value));
    }

    while (rclcpp::ok()) {
        std::this_thread::sleep_for(100ms);

        bool all_ready = true;
        for (auto &task : tasks) {
            if (!task->isReady() || task->isInPerform()) {
                all_ready = false;
                break;
            }
        }
        if (all_ready) {
            for (auto &task : tasks) {
                task->startPerform();
            }
        }

        for (auto &task : tasks) {
            std::cout << std::flush;
            task->runOnce();
        }
    }

    return 0;
}