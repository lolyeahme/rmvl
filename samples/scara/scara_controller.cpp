/**
 * @file scara_controller.cpp
 * @brief SCARA 机械臂导纳控制器节点
 * @details 基于 LPSS 轻量发布订阅服务实现，提供以下功能：
 * - 订阅 `/scara/joint_states` 话题，获取机械臂当前关节状态
 * - 订阅 `/scara/wrench` 话题，获取末端执行器所受的外部力/力矩
 * - 发布 `/scara/cmd_joint_states` 话题，输出导纳控制后的关节指令
 *
 * 导纳控制原理：
 *   将外部力/力矩映射为笛卡尔空间中的柔顺运动，基本模型为
 *       M · ẍ + D · ẋ = F_ext
 *   其中 M 为虚拟惯性矩阵，D 为虚拟阻尼矩阵，F_ext 为外力，
 *   ẍ、ẋ 分别为笛卡尔空间中的加速度和速度。
 *   离散化后通过 SCARA Jacobian 逆矩阵转换至关节空间输出关节指令。
 *
 * SCARA 正运动学（4 自由度）：
 *   x     = L1·cos(q1) + L2·cos(q1+q2)
 *   y     = L1·sin(q1) + L2·sin(q1+q2)
 *   z     = q3
 *   theta = q1 + q2 + q4
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

// ======================== 导纳控制参数 ========================
// 虚拟惯性 M (kg 或 kg·m²)
constexpr double M_x = 5.0;
constexpr double M_y = 5.0;
constexpr double M_z = 5.0;
constexpr double M_theta = 0.5;

// 虚拟阻尼 D (N·s/m 或 N·m·s/rad)
constexpr double D_x = 50.0;
constexpr double D_y = 50.0;
constexpr double D_z = 50.0;
constexpr double D_theta = 5.0;

constexpr double SINGULARITY_THRESHOLD = 1e-6; //!< Jacobian 奇异性检测阈值
constexpr double ADMITTANCE_GAIN = 100.0;       //!< 导纳速度到关节位置偏移的缩放增益

// ======================== 全局状态 ========================
static std::atomic_bool running{true};
static void onSignal(int) { running = false; }

/**
 * @brief SCARA Jacobian 逆运算，将笛卡尔速度转换为关节速度
 * @note 当 sin(q2) 接近 0 时（奇异位形），返回零速度以避免数值发散
 *
 * @param[in] q 当前 4 个关节位置 [q1, q2, q3, q4]
 * @param[in] dx 笛卡尔空间速度 [ẋ, ẏ, ż, θ̇]
 * @param[out] dq 关节空间速度 [dq1, dq2, dq3, dq4]
 */
static void jacobianInverse(const double q[4], const double dx[4], double dq[4])
{
    double s2 = std::sin(q[1]);
    double det = L1 * L2 * s2;

    // 奇异位形检测
    if (std::abs(det) < SINGULARITY_THRESHOLD)
    {
        dq[0] = dq[1] = dq[2] = dq[3] = 0.0;
        return;
    }

    double s1 = std::sin(q[0]);
    double c1 = std::cos(q[0]);
    double s12 = std::sin(q[0] + q[1]);
    double c12 = std::cos(q[0] + q[1]);
    double inv_det = 1.0 / det;

    // J_xy 逆矩阵: 1/det * [[L2*cos(q1+q2), L2*sin(q1+q2)],
    //                        [-L1*cos(q1)-L2*cos(q1+q2), -L1*sin(q1)-L2*sin(q1+q2)]]
    dq[0] = inv_det * (L2 * c12 * dx[0] + L2 * s12 * dx[1]);
    dq[1] = inv_det * ((-L1 * c1 - L2 * c12) * dx[0] + (-L1 * s1 - L2 * s12) * dx[1]);

    // Z 轴直接映射
    dq[2] = dx[2];

    // 末端旋转：dq4 = dtheta - dq1 - dq2
    dq[3] = dx[3] - dq[0] - dq[1];
}

int main()
{
    signal(SIGINT, onSignal);

    // 创建 LPSS 节点
    lpss::Node node("scara_controller");

    // 最新关节状态（由回调更新）
    msg::JointState latest_joint;
    latest_joint.position = {0.0, M_PI / 4, 0.0, 0.0};
    latest_joint.velocity = {0.0, 0.0, 0.0, 0.0};
    std::mutex joint_mtx;
    std::atomic_bool joint_received{false};

    // 最新外力（由回调更新）
    msg::Wrench latest_wrench;
    std::mutex wrench_mtx;

    // 创建订阅者
    auto joint_sub = node.createSubscriber<msg::JointState>("/scara/joint_states", [&](const msg::JointState &msg) {
        std::lock_guard lk(joint_mtx);
        latest_joint = msg;
        joint_received.store(true, std::memory_order_release);
    });

    auto wrench_sub = node.createSubscriber<msg::Wrench>("/scara/wrench", [&](const msg::Wrench &msg) {
        std::lock_guard lk(wrench_mtx);
        latest_wrench = msg;
    });

    // 创建发布者
    auto cmd_pub = node.createPublisher<msg::JointState>("/scara/cmd_joint_states");

    printf("[scara_controller] 导纳控制器已启动，Ctrl+C 退出\n");
    printf("[scara_controller] 等待关节状态数据...\n");

    // 导纳控制状态：笛卡尔空间速度
    double cart_vel[4] = {0.0, 0.0, 0.0, 0.0}; // [ẋ, ẏ, ż, θ̇]

    // 基准关节位置（导纳输出将叠加在此基础上）
    double base_position[4] = {0.0, M_PI / 4, 0.0, 0.0};
    double admittance_offset[4] = {0.0, 0.0, 0.0, 0.0}; // 笛卡尔空间导纳偏移

    constexpr double dt = PERIOD_MS / 1000.0;
    uint32_t seq = 0;
    auto next_time = std::chrono::steady_clock::now();

    while (running)
    {
        next_time += std::chrono::milliseconds(PERIOD_MS);

        // ---- 1. 获取当前状态 ----
        if (!joint_received.load(std::memory_order_acquire))
        {
            ++seq;
            std::this_thread::sleep_until(next_time);
            continue;
        }

        double cur_q[4];
        double F_ext[4]; // [Fx, Fy, Fz, Tz]
        {
            std::lock_guard lk(joint_mtx);
            for (int i = 0; i < 4; ++i)
                cur_q[i] = latest_joint.position[i];
        }
        {
            std::lock_guard lk(wrench_mtx);
            F_ext[0] = latest_wrench.force.x;
            F_ext[1] = latest_wrench.force.y;
            F_ext[2] = latest_wrench.force.z;
            F_ext[3] = latest_wrench.torque.z;
        }

        // ---- 2. 导纳控制律（笛卡尔空间） ----
        // M · ẍ + D · ẋ = F_ext  =>  ẍ = (F_ext - D · ẋ) / M
        // 离散化: ẋ(k+1) = ẋ(k) + dt · ẍ(k)
        constexpr double M_arr[4] = {M_x, M_y, M_z, M_theta};
        constexpr double D_arr[4] = {D_x, D_y, D_z, D_theta};

        for (int i = 0; i < 4; ++i)
        {
            double accel = (F_ext[i] - D_arr[i] * cart_vel[i]) / M_arr[i];
            cart_vel[i] += dt * accel;
            admittance_offset[i] += dt * cart_vel[i];
        }

        // ---- 3. 笛卡尔偏移转换为关节偏移 ----
        double dq[4];
        jacobianInverse(cur_q, cart_vel, dq);

        // 累积关节偏移量
        double cmd_q[4];
        for (int i = 0; i < 4; ++i)
            cmd_q[i] = base_position[i] + dq[i] * dt * ADMITTANCE_GAIN;

        // 关节限位保护
        constexpr double q_max[4] = {M_PI, M_PI * 0.9, 0.2, M_PI};
        constexpr double q_min[4] = {-M_PI, -M_PI * 0.9, -0.2, -M_PI};
        for (int i = 0; i < 4; ++i)
            cmd_q[i] = std::max(q_min[i], std::min(q_max[i], cmd_q[i]));

        // ---- 4. 发布关节指令 ----
        msg::JointState cmd;
        cmd.header.seq = seq;
        cmd.header.stamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::system_clock::now().time_since_epoch())
                               .count();
        cmd.header.frame_id = "scara_base";
        cmd.name = {"joint1", "joint2", "joint3", "joint4"};
        cmd.position = {cmd_q[0], cmd_q[1], cmd_q[2], cmd_q[3]};
        cmd.velocity = {dq[0], dq[1], dq[2], dq[3]};
        cmd.effort = {};
        cmd_pub.publish(cmd);

        // ---- 5. 打印控制信息 ----
        if (seq % 100 == 0)
        {
            printf("\r[controller] F=(%.2f, %.2f, %.2f) Tz=%.2f  "
                   "vel=(%.4f, %.4f, %.4f, %.4f)  "
                   "cmd_q=(%.3f, %.3f, %.3f, %.3f)",
                   F_ext[0], F_ext[1], F_ext[2], F_ext[3],
                   cart_vel[0], cart_vel[1], cart_vel[2], cart_vel[3],
                   cmd_q[0], cmd_q[1], cmd_q[2], cmd_q[3]);
            fflush(stdout);
        }

        ++seq;
        std::this_thread::sleep_until(next_time);
    }

    printf("\n[scara_controller] 导纳控制器已停止\n");
    return 0;
}
