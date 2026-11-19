#ifndef MPC_TRACKER_MPC_H
#define MPC_TRACKER_MPC_H

#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <vector> // 用于 std::vector

namespace mpc_tracker
{

class Mpc
{
public:
    Mpc();
    ~Mpc() = default;

    /**
     * @brief  设置 MPC 维度和时域
     */
    void setDimensions(int state_dim, int control_dim, int mpc_window, double dt);

    /**
     * @brief 设置车辆轴距 (用于线性化)
     */
    void setWheelbase(double L);

    /**
     * @brief 求解 MPC 问题 (基于误差状态)
     * @param x0 当前状态 [x, y, yaw]
     * @param xref (state_dim x N+1) 参考状态 [x_r; y_r; yaw_r]
     * @param uref (control_dim x N) 参考控制 [v_r; delta_r]
     * @return bool 是否求解成功
     */
    bool solveMpc(const Eigen::Vector3d &x0, const Eigen::MatrixXd &xref, const Eigen::MatrixXd &uref);
    
    /**
     * @brief 获取控制指令 [v, delta]
     * (现在返回 u_r[0] + eu[0])
     */
    Eigen::Vector2d getControlCmd();
    
    /**
     * @brief 获取预测的 *绝对* 状态 [x, y, yaw, ...]
     * (现在返回 x_r[k] + e[k])
     */
    Eigen::VectorXd getPredictState();

private:
    int state_dim_;    // 状态维度 (e.g., 3: [x, y, yaw])
    int control_dim_;  // 控制维度 (e.g., 2: [v, delta])
    int mpc_window_;   // 预测时域 N
    double dt_;        // 时间步长

    double wheelbase_L_; // 轴距 (!! 新增 !!)
    
    int num_vars_;     // QP 变量总数 (误差状态 e 和 误差控制 eu)
    int num_cons_;     // QP 约束总数

    // 权重矩阵 (用于误差)
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd Q_terminal_;
    Eigen::MatrixXd R_;

    // 控制约束
    double v_max_, v_min_;
    double delta_max_, delta_min_; // 前轮转角约束

    // QP 矩阵 (Hessian, gradient)
    Eigen::SparseMatrix<double> hessian_;
    Eigen::VectorXd gradient_;
    
    // QP 约束矩阵 (A_eq, l, u)
    Eigen::SparseMatrix<double> linear_matrix_; // A_eq
    Eigen::VectorXd lower_bound_; // l
    Eigen::VectorXd upper_bound_; // u

    // 求解结果 (存储误差状态 e 和 eu)
    Eigen::VectorXd mpc_solution_error_state_; 
    
    // 存储上一次的参考输入 (用于 getControlCmd)
    Eigen::MatrixXd last_uref_; 
    // 存储上一次的参考状态 (用于 getPredictState)
    Eigen::MatrixXd last_xref_; 
    // 存储上一次的求解状态
    bool last_solve_ok_;

    // OSQP 求解器
    OsqpEigen::Solver solver_;
    bool solver_initialized_; 

    // 内部辅助函数
    void setWeightMatrices();
    void setConstraints();

    /**
     * @brief  构建 Hessian 矩阵 (对角线 Q, Q_f, R)
     */
    void buildHessian();

    /**
     * @brief  构建梯度向量 (全 0)
     */
    void buildGradient();

    /**
     * @brief 构建约束矩阵 A_eq (动态线性化)
     * e_{k+1} = A_k*e_k + B_k*eu_k 
     * @param xref (state_dim x N+1) 参考状态 [x_r; y_r; yaw_r]
     * @param uref (control_dim x N) 参考控制 [v_r; delta_r]
     */
    void buildConstraintMatrix(const Eigen::MatrixXd &xref, const Eigen::MatrixXd &uref);

    /**
     * @brief  更新约束向量 l 和 u
     * @param x0 当前状态 [x, y, yaw]
     * @param xref (state_dim x N+1) 参考状态 [x_r; y_r; yaw_r]
     * @param uref (control_dim x N) 参考控制 [v_r; delta_r]
     */
    void updateConstraintVectors(const Eigen::Vector3d &x0, 
                                 const Eigen::MatrixXd &xref, 
                                 const Eigen::MatrixXd &uref);
    
    // 辅助函数：获取 QP 变量中 e_k 和 eu_k 的起始索引
    inline int state_offset(int k) const { return k * state_dim_; }
    inline int control_offset(int k) const { return (mpc_window_ + 1) * state_dim_ + k * control_dim_; }

}; // class Mpc

} // namespace mpc_tracker

#endif // MPC_TRACKER_MPC_H