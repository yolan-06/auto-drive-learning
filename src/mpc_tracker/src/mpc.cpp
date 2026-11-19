#include <mpc_tracker/mpc.h>
#include <ros/ros.h> 
#include <cmath> // 用于 sin, cos, tan
#include <vector> // 用于 std::vector

namespace mpc_tracker
{

// 归一化角度到 [-pi, pi] 的辅助函数
double normalizeAngle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// 构造函数：初始化所有参数和矩阵
Mpc::Mpc()
    : state_dim_(3), 
      control_dim_(2),
      mpc_window_(20),
      dt_(0.05),
      wheelbase_L_(0.9), // 默认值
      solver_initialized_(false),
      last_solve_ok_(false)
{
    // --- 从参数服务器获取 (或使用默认值) ---
    ros::NodeHandle pnh("~"); 

    // --- 设置权重 (已包含调参) ---
    setWeightMatrices();

    // --- 设置约束 ---
    setConstraints();
}

// 设置维度
void Mpc::setDimensions(int state_dim, int control_dim, int mpc_window, double dt)
{
    state_dim_ = state_dim;
    control_dim_ = control_dim;
    mpc_window_ = mpc_window;
    dt_ = dt;

    // --- 初始化 QP 变量/约束总数 ---
    num_vars_ = state_dim_ * (mpc_window_ + 1) + control_dim_ * mpc_window_;
    num_cons_ = state_dim_ * (mpc_window_ + 1) + control_dim_ * mpc_window_;

    // --- 初始化 QP 矩阵 (仅调整大小) ---
    hessian_.resize(num_vars_, num_vars_);
    gradient_.resize(num_vars_);
    linear_matrix_.resize(num_cons_, num_vars_);
    lower_bound_.resize(num_cons_);
    upper_bound_.resize(num_cons_);

    // --- 初始化存储 ---
    mpc_solution_error_state_.resize(num_vars_);
    mpc_solution_error_state_.setZero();
    last_uref_.resize(control_dim_, mpc_window_);
    last_uref_.setZero();
    last_xref_.resize(state_dim_, mpc_window_ + 1);
    last_xref_.setZero();
}

// 设置轴距
void Mpc::setWheelbase(double L)
{
    wheelbase_L_ = L;
}


// --- 核心求解函数 ---
bool Mpc::solveMpc(const Eigen::Vector3d &x0, const Eigen::MatrixXd &xref, const Eigen::MatrixXd &uref)
{
    // 1.构建梯度向量 g (全0)
    buildGradient();

    // 2. 构建约束矩阵 A_eq (动态线性化)
    buildConstraintMatrix(xref, uref);

    // 3. 更新约束向量 l 和 u (设置 e_0 和 eu_k 的界限)
    updateConstraintVectors(x0, xref, uref); // (!! 归一化Bug已在此函数内部修复 !!)

    // 4. 初始化或更新求解器
    if (!solver_initialized_)
    {
        // 4a. 仅构建一次 Hessian
        buildHessian(); 

        solver_.data()->setNumberOfVariables(num_vars_);
        solver_.data()->setNumberOfConstraints(num_cons_);
        solver_.data()->setHessianMatrix(hessian_);
        solver_.data()->setGradient(gradient_);
        solver_.data()->setLinearConstraintsMatrix(linear_matrix_);
        solver_.data()->setLowerBound(lower_bound_);
        solver_.data()->setUpperBound(upper_bound_);
        
        solver_.settings()->setWarmStart(true);
        solver_.settings()->setPolish(true);
        
        //  迭代次数到 150，防止计算超时导致停车
        solver_.settings()->setMaxIteration(150); 
        // solver_.settings()->setVerbose(true); 

        if (!solver_.initSolver())
        {
            ROS_WARN("[MPC] OSQP failed to initialize solver!");
            return false;
        }
        solver_initialized_ = true;
    }
    else
    {
        // 4c. 后续运行：更新所有时变矩阵
        solver_.updateGradient(gradient_); 
        solver_.updateLinearConstraintsMatrix(linear_matrix_); 
        solver_.updateBounds(lower_bound_, upper_bound_); 
    }

    // 5. 求解 QP
    if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
    {
        ROS_WARN("[MPC] OSQP failed to solve problem!");
        last_solve_ok_ = false;
        return false;
    }

    // 6. 获取解 (e 和 eu)
    mpc_solution_error_state_ = solver_.getSolution();
    
    // 7. 存储参考
    last_xref_ = xref;
    last_uref_ = uref;
    last_solve_ok_ = true;
    
    return true;
}

// 获取控制指令 [v, delta]
Eigen::Vector2d Mpc::getControlCmd()
{
    if (!last_solve_ok_)
    {
        // 如果上次求解失败，返回 [0, 0] (安全刹车)
        return Eigen::Vector2d(0.0, 0.0);
    }
    
    // u = u_r + eu
    Eigen::Vector2d eu0 = mpc_solution_error_state_.segment(control_offset(0), control_dim_);
    Eigen::Vector2d ur0 = last_uref_.col(0);
    
    Eigen::Vector2d u_actual = ur0 + eu0;

    // 对最终控制指令进行裁剪
    u_actual(0) = std::max(v_min_, std::min(v_max_, u_actual(0)));
    u_actual(1) = std::max(delta_min_, std::min(delta_max_, u_actual(1)));

    return u_actual;
}

Eigen::VectorXd Mpc::getPredictState()
{
    // 预测的 *绝对* 状态 x_pred = x_ref + e
    Eigen::VectorXd pred_states_absolute(state_dim_ * (mpc_window_ + 1));

    for (int k = 0; k <= mpc_window_; ++k)
    {
        Eigen::Vector3d e_k = mpc_solution_error_state_.segment(state_offset(k), state_dim_);
        Eigen::Vector3d xr_k = last_xref_.col(k);
        
        // (!! 关键修复 !!) 归一化预测的航向角，防止Rviz显示跳动
        Eigen::Vector3d x_pred_k = xr_k + e_k;
        x_pred_k(2) = normalizeAngle(x_pred_k(2));
        
        pred_states_absolute.segment(k * state_dim_, state_dim_) = x_pred_k;
    }
    
    return pred_states_absolute;
}


// --- 内部辅助函数 ---

void Mpc::setWeightMatrices()
{
    ros::NodeHandle pnh("~"); 
    double q_x, q_y, q_yaw, q_term_x, q_term_y, q_term_yaw, r_v, r_delta;
    
    pnh.param("q_x", q_x, 5.0);
    pnh.param("q_y", q_y, 5.0);
    pnh.param("q_yaw", q_yaw, 1.0);
    pnh.param("q_term_x", q_term_x, 1.0);
    pnh.param("q_term_y", q_term_y, 1.0);
    pnh.param("q_term_yaw", q_term_yaw, 0.1);
    pnh.param("r_v", r_v, 0.1);
    pnh.param("r_delta", r_delta, 0.1);

    Q_ = Eigen::MatrixXd(state_dim_, state_dim_);
    Q_ << q_x, 0, 0,
          0, q_y, 0,
          0, 0, q_yaw;

    Q_terminal_ = Eigen::MatrixXd(state_dim_, state_dim_);
    Q_terminal_ << q_term_x, 0, 0,
                   0, q_term_y, 0,
                   0, 0, q_term_yaw;

    R_ = Eigen::MatrixXd(control_dim_, control_dim_);
    R_ << r_v, 0,
          0, r_delta;
}

void Mpc::setConstraints()
{
    ros::NodeHandle pnh("~"); 
    pnh.param("v_max", v_max_, 5.0);  
    pnh.param("v_min", v_min_, 0.0);  
    pnh.param("delta_max", delta_max_, 0.43); 
    pnh.param("delta_min", delta_min_, -0.43); 
}

void Mpc::buildHessian()
{
    hessian_.setZero();
    std::vector<Eigen::Triplet<double>> triplets;

    // 状态权重 e_k^T * Q * e_k (k=0...N-1)
    for (int k = 0; k < mpc_window_; ++k)
    {
        for (int i = 0; i < state_dim_; ++i)
        {
            triplets.emplace_back(state_offset(k) + i, state_offset(k) + i, Q_(i, i));
        }
    }
    
    // 终端权重 e_N^T * Q_f * e_N
    for (int i = 0; i < state_dim_; ++i)
    {
        triplets.emplace_back(state_offset(mpc_window_) + i, state_offset(mpc_window_) + i, Q_terminal_(i, i));
    }

    // 控制权重 eu_k^T * R * eu_k (k=0...N-1)
    for (int k = 0; k < mpc_window_; ++k)
    {
        for (int i = 0; i < control_dim_; ++i)
        {
            triplets.emplace_back(control_offset(k) + i, control_offset(k) + i, R_(i, i));
        }
    }
    
    hessian_.setFromTriplets(triplets.begin(), triplets.end());
}

void Mpc::buildGradient()
{
    gradient_.setZero();
}

void Mpc::buildConstraintMatrix(const Eigen::MatrixXd &xref, const Eigen::MatrixXd &uref)
{
    linear_matrix_.setZero();
    std::vector<Eigen::Triplet<double>> triplets;
    
    // --- 约束 1: 初始状态 ---
    for (int i = 0; i < state_dim_; ++i)
    {
        triplets.emplace_back(state_offset(0) + i, state_offset(0) + i, 1.0);
    }
    
    // --- 约束 2: 动态约束 e_{k+1} = A_k*e_k + B_k*eu_k ---
    for (int k = 0; k < mpc_window_; ++k)
    {
        double yaw_r = xref(2, k);
        double v_r = uref(0, k);
        double delta_r = uref(1, k);

        // --- 计算 A_k 和 B_k ---
        Eigen::Matrix3d A_k;
        A_k << 1.0, 0.0, -v_r * std::sin(yaw_r) * dt_,
               0.0, 1.0,  v_r * std::cos(yaw_r) * dt_,
               0.0, 0.0,  1.0;

        Eigen::MatrixXd B_k(state_dim_, control_dim_);
        double tan_delta_r = std::tan(delta_r);
        double cos_delta_r = std::cos(delta_r);
        B_k << std::cos(yaw_r) * dt_,             0.0,
               std::sin(yaw_r) * dt_,             0.0,
               tan_delta_r / wheelbase_L_ * dt_,  v_r / (wheelbase_L_ * cos_delta_r * cos_delta_r) * dt_;
        
        int row_start = state_offset(k + 1);
        
        // 填充 -A_k
        for (int r = 0; r < state_dim_; ++r) {
            for (int c = 0; c < state_dim_; ++c) {
                triplets.emplace_back(row_start + r, state_offset(k) + c, -A_k(r, c));
            }
        }

        // 填充 I
        for (int i = 0; i < state_dim_; ++i) {
            triplets.emplace_back(row_start + i, state_offset(k + 1) + i, 1.0);
        }

        // 填充 -B_k
        for (int r = 0; r < state_dim_; ++r) {
            for (int c = 0; c < control_dim_; ++c) {
                triplets.emplace_back(row_start + r, control_offset(k) + c, -B_k(r, c));
            }
        }
    }

    // --- 约束 3: 控制约束 ---
    for (int k = 0; k < mpc_window_; ++k)
    {
        for (int i = 0; i < control_dim_; ++i)
        {
            int row = state_offset(mpc_window_ + 1) + k * control_dim_ + i;
            int col = control_offset(k) + i;
            triplets.emplace_back(row, col, 1.0);
        }
    }
    
    linear_matrix_.setFromTriplets(triplets.begin(), triplets.end());
}


void Mpc::updateConstraintVectors(const Eigen::Vector3d &x0, 
                                const Eigen::MatrixXd &xref, 
                                const Eigen::MatrixXd &uref)
{
    // l 和 u 向量
    
    // --- 约束 1: 初始状态 ---
    
    
    // 分别计算 e0，并对航向角进行归一化
    Eigen::Vector3d e0;
    e0(0) = x0(0) - xref(0, 0); // x 误差
    e0(1) = x0(1) - xref(1, 0); // y 误差
    e0(2) = normalizeAngle( x0(2) - xref(2, 0) ); // 归一化的 yaw 误差
    
    lower_bound_.segment(state_offset(0), state_dim_) = e0;
    upper_bound_.segment(state_offset(0), state_dim_) = e0;
    
    // --- 约束 2: 动态约束 ---
    for (int k = 0; k < mpc_window_; ++k)
    {
        lower_bound_.segment(state_offset(k + 1), state_dim_).setZero();
        upper_bound_.segment(state_offset(k + 1), state_dim_).setZero();
    }
    
    // --- 约束 3: 控制约束 ---
    // u_min - u_r <= eu_k <= u_max - u_r
    
    Eigen::Vector2d u_min(v_min_, delta_min_);
    Eigen::Vector2d u_max(v_max_, delta_max_);
    
    for (int k = 0; k < mpc_window_; ++k)
    {
        Eigen::Vector2d ur_k = uref.col(k);
        
        Eigen::Vector2d l_ctrl = u_min - ur_k;
        Eigen::Vector2d u_ctrl = u_max - ur_k;
        
        lower_bound_.segment(state_offset(mpc_window_ + 1) + k * control_dim_, control_dim_) = l_ctrl;
        upper_bound_.segment(state_offset(mpc_window_ + 1) + k * control_dim_, control_dim_) = u_ctrl;
    }
}


} // namespace mpc_tracker