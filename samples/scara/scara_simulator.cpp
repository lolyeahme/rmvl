/**
 * @file scara_simulator.cpp
 * @brief SCARA 机械臂仿真节点
 * @details 基于 LPSS 轻量发布订阅服务实现，提供以下功能：
 * - 发布 `/scara/joint_states` 话题，包含 4 个关节的实时状态
 * - 发布 `/scara/wrench` 话题，模拟末端执行器所受的外部力/力矩
 * - 订阅 `/scara/cmd_joint_states` 话题，接收来自导纳控制器的关节指令
 * - 使用一阶低通滤波跟踪目标关节位置，模拟简化的关节动力学
 *
 * SCARA 机械臂结构（4 自由度）：
 * - 关节 1：旋转关节（绕 Z 轴），基座处
 * - 关节 2：旋转关节（绕 Z 轴），肘部
 * - 关节 3：移动关节（沿 Z 轴），末端升降
 * - 关节 4：旋转关节（绕 Z 轴），末端旋转
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <mutex>
#include <thread>

#include "rmvl/lpss.hpp"
#include "rmvlmsg/geometry/wrench.hpp"

using namespace rm;

// ======================== 机械臂参数 ========================
constexpr double L1 = 0.30;   //!< 连杆 1 长度 (m)
constexpr double L2 = 0.25;   //!< 连杆 2 长度 (m)
constexpr int PERIOD_MS = 10; //!< 控制周期 (ms)

// ======================== 全局状态 ========================
static std::atomic_bool running{true};
static void onSignal(int) { running = false; }

int main()
{
    signal(SIGINT, onSignal);

    // 创建 LPSS 节点
    lpss::Node node("scara_simulator");

    // 当前关节状态
    msg::JointState current_state;
    current_state.header.frame_id = "scara_base";
    current_state.name = {"joint1", "joint2", "joint3", "joint4"};
    current_state.position = {0.0, M_PI / 4, 0.0, 0.0}; // 初始位姿
    current_state.velocity = {0.0, 0.0, 0.0, 0.0};
    current_state.effort = {0.0, 0.0, 0.0, 0.0};

    // 目标关节位置（来自控制器的指令）
    std::vector<double> cmd_position = current_state.position;
    std::mutex cmd_mtx;

    // 创建发布者
    auto joint_pub = node.createPublisher<msg::JointState>("/scara/joint_states");
    auto wrench_pub = node.createPublisher<msg::Wrench>("/scara/wrench");

    // 创建订阅者：接收控制器下发的关节指令
    auto cmd_sub = node.createSubscriber<msg::JointState>("/scara/cmd_joint_states", [&](const msg::JointState &msg) {
        std::lock_guard lk(cmd_mtx);
        if (msg.position.size() == 4)
            cmd_position = msg.position;
    });

    printf("[scara_simulator] 节点已启动，Ctrl+C 退出\n");

    // 仿真主循环
    uint32_t seq = 0;
    auto next_time = std::chrono::steady_clock::now();
    constexpr double dt = PERIOD_MS / 1000.0;
    constexpr double alpha = 0.3; // 低通滤波系数，模拟关节惯性

    while (running)
    {
        next_time += std::chrono::milliseconds(PERIOD_MS);
        double t = seq * dt;

        // ---- 1. 跟踪目标关节位置（一阶惯性环节） ----
        {
            std::lock_guard lk(cmd_mtx);
            for (int i = 0; i < 4; ++i)
            {
                double prev = current_state.position[i];
                current_state.position[i] += alpha * (cmd_position[i] - prev);
                current_state.velocity[i] = (current_state.position[i] - prev) / dt;
            }
        }

        // ---- 2. 生成模拟外力（正弦扰动力） ----
        msg::Wrench wrench;
        wrench.force.x = 5.0 * std::sin(2.0 * M_PI * 0.5 * t); // X 方向 0.5Hz 正弦力
        wrench.force.y = 3.0 * std::sin(2.0 * M_PI * 0.3 * t); // Y 方向 0.3Hz 正弦力
        wrench.force.z = 0.0;
        wrench.torque.x = 0.0;
        wrench.torque.y = 0.0;
        wrench.torque.z = 0.5 * std::sin(2.0 * M_PI * 0.2 * t); // Z 方向 0.2Hz 正弦力矩

        // ---- 3. 发布当前状态与外力 ----
        current_state.header.seq = seq;
        current_state.header.stamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                                         std::chrono::system_clock::now().time_since_epoch())
                                         .count();
        joint_pub.publish(current_state);
        wrench_pub.publish(wrench);

        // ---- 4. 打印状态信息 ----
        if (seq % 100 == 0)
        {
            double q1 = current_state.position[0], q2 = current_state.position[1];
            double x = L1 * std::cos(q1) + L2 * std::cos(q1 + q2);
            double y = L1 * std::sin(q1) + L2 * std::sin(q1 + q2);
            printf("\r[simulator] t=%.1fs  pos=(%.3f, %.3f, %.3f, %.3f)  "
                   "tcp=(%.3f, %.3f)  F=(%.2f, %.2f)",
                   t, current_state.position[0], current_state.position[1],
                   current_state.position[2], current_state.position[3], x, y,
                   wrench.force.x, wrench.force.y);
            fflush(stdout);
        }

        ++seq;
        std::this_thread::sleep_until(next_time);
    }

    printf("\n[scara_simulator] 节点已停止\n");
    return 0;
}
