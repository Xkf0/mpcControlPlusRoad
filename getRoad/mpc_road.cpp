#include "mpc_road.h"
C_MPC_ROAD::C_MPC_ROAD(double _d_q, double _d_r, Eigen::VectorXd _v_aymin, Eigen::VectorXd _v_aymax, int _i_Nc, int _i_Np, double _d_T, double _d_zeta, double _d_Sob, int _i_num)
    : d_q(_d_q),
      d_r(_d_r),
      v_aymin(_v_aymin),
      v_aymax(_v_aymax),
      i_num(_i_num)
{
    optData.i_Nc = _i_Nc;
    optData.i_Np = _i_Np;
    optData.d_T = _d_T;
    optData.d_zeta = _d_zeta;
    optData.d_Sob = _d_Sob;
    vd_x.assign(_i_Nc, 0.0);
}

static C_MPC_ROAD &GetOnlyCmpc(double _d_q, double _d_r, Eigen::VectorXd _v_aymin, Eigen::VectorXd _v_aymax, int _i_Nc, int _i_Np, double _d_T, double _d_zeta, double _d_Sob, int _i_num)
{
    // 懒汉模式，不适合多线程
    static C_MPC_ROAD onlyInstance(_d_q, _d_r, _v_aymin, _v_aymax, _i_Nc, _i_Np, _d_T, _d_zeta, _d_Sob, _i_num);
    return onlyInstance;
}

// 复制构造函数，若放到private里面则不允许复制
C_MPC_ROAD::C_MPC_ROAD(const C_MPC_ROAD &other)
    : d_q(other.d_q),
      d_r(other.d_r),
      v_aymin(other.v_aymin),
      v_aymax(other.v_aymax),
      vd_x(other.vd_x),
      optData(other.optData)
{
}

// 复制赋值运算符
C_MPC_ROAD &C_MPC_ROAD::operator=(const C_MPC_ROAD &other)
{
    if (this != &other) // 防止自赋值
    {
        d_q = other.d_q;
        d_r = other.d_r;
        v_aymin = other.v_aymin;
        v_aymax = other.v_aymax;
        vd_x = other.vd_x;
        i_num = other.i_num;
        optData = other.optData;
    }
    return *this; // 返回当前对象的引用
}

// 移动构造函数
C_MPC_ROAD::C_MPC_ROAD(C_MPC_ROAD &&other) noexcept
    : d_q(other.d_q),
      d_r(other.d_r),
      v_aymin(std::move(other.v_aymin)),
      v_aymax(std::move(other.v_aymax)),
      vd_x(std::move(other.vd_x)),
      i_num(std::move(other.i_num)),
      optData(std::move(other.optData))
{
}

// 移动赋值运算符
C_MPC_ROAD &C_MPC_ROAD::operator=(C_MPC_ROAD &&other) noexcept
{
    if (this != &other) // 防止自赋值
    {
        d_q = other.d_q;
        d_r = other.d_r;
        v_aymin = std::move(other.v_aymin);
        v_aymax = std::move(other.v_aymax);
        vd_x = std::move(other.vd_x);
        i_num = other.i_num;
        optData = std::move(other.optData);
    }
    return *this;
}

// 如果为private则只能new创建而不能在栈上面创建
C_MPC_ROAD::~C_MPC_ROAD()
{
}

bool C_MPC_ROAD::set_q_r(double _d_q, double _d_r)
{
    if (_d_q > 0 && _d_r > 0)
    {
        d_q = _d_q;
        d_r = _d_r;
        return true;
    }
    return false;
}

bool C_MPC_ROAD::set_aymin_aymax(Eigen::VectorXd _v_aymin, Eigen::VectorXd _v_aymax)
{
    if (_v_aymin.size() != 0 && _v_aymax.size() != 0)
    {
        v_aymin = _v_aymin;
        v_aymax = _v_aymax;
        return true;
    }
    return false;
}

bool C_MPC_ROAD::set_Nc_Np(int _i_Nc, int _i_Np)
{
    if (_i_Nc > 0 && _i_Np > 0)
    {
        optData.i_Nc = _i_Nc;
        optData.i_Np = _i_Np;
        return true;
    }
    return false;
}

bool C_MPC_ROAD::set_T(double _d_T)
{
    if (_d_T > 0)
    {
        optData.d_T = _d_T;
        return true;
    }
    return false;
}

bool C_MPC_ROAD::set_pathWithAngleRef(std::vector<robos::Pose2D> _pathsWithAngleRef)
{
    if (_pathsWithAngleRef.size() != 0)
    {
        pathsWithAngleRef = _pathsWithAngleRef;
        return true;
    }
    return false;
}

bool C_MPC_ROAD::set_zeta_Sob(double _d_zeta, double _d_Sob)
{
    if (_d_zeta > 0 && _d_Sob > 0)
    {
        optData.d_zeta = _d_zeta;
        optData.d_Sob = _d_Sob;
        return true;
    }
    return false;
}

bool C_MPC_ROAD::set_N(int _i_N)
{
    if (_i_N > 0)
    {
        optData.i_N = _i_N;
        return true;
    }
    return false;
}

bool C_MPC_ROAD::set_num(int _i_num)
{
    if (_i_num > 0)
    {
        i_num = _i_num;
        return true;
    }
    return false;
}

bool C_MPC_ROAD::set_poseNow(double d_x0, double d_y0, double d_theta0, double d_v0)
{
    poseNow.d_x0 = d_x0;
    poseNow.d_y0 = d_y0;
    poseNow.d_theta0 = d_theta0;
    poseNow.d_v0 = d_v0;
    return true;
}

bool C_MPC_ROAD::set_sideOb(Eigen::VectorXd _v_sideOb)
{
    if (_v_sideOb.size() != 0)
    {
        optData.v_sideOb = _v_sideOb;
        optData.i_N = _v_sideOb.size() / 2;
        return true;
    }
    return false;
}

Eigen::VectorXd C_MPC_ROAD::calculate_coordinateConversionToWorld(double d_x, double d_y, double d_alpha, double d_dx, double d_dy)
{
    Eigen::Matrix<double, 3, 3> _m_T_inverse;
    _m_T_inverse << cos(d_alpha), -sin(d_alpha), d_dx, sin(d_alpha), cos(d_alpha), d_dy, 0, 0, 1;
    Eigen::Matrix<double, 3, 1> coordinate_before;
    coordinate_before << d_x, d_y, 1.0;
    Eigen::Matrix<double, 3, 1> coordinate_after;
    coordinate_after = _m_T_inverse * coordinate_before;
    Eigen::VectorXd result = 0.0 * Eigen::VectorXd::Ones(2);
    result(0) = coordinate_after(0, 0);
    result(1) = coordinate_after(1, 0);
    return result;
}

Eigen::VectorXd C_MPC_ROAD::calculate_coordinateConversionFromWorld(double d_x, double d_y, double d_alpha, double d_dx, double d_dy)
{
    Eigen::Matrix<double, 3, 3> _m_T_inverse;
    _m_T_inverse << cos(d_alpha), sin(d_alpha), -d_dx * cos(d_alpha) - d_dy * sin(d_alpha), -sin(d_alpha), cos(d_alpha), d_dx * sin(d_alpha) - d_dy * cos(d_alpha), 0, 0, 1;
    Eigen::Matrix<double, 3, 1> coordinate_before;
    coordinate_before << d_x, d_y, 1.0;
    Eigen::Matrix<double, 3, 1> coordinate_after;
    coordinate_after = _m_T_inverse * coordinate_before;
    Eigen::VectorXd result = 0.0 * Eigen::VectorXd::Ones(2);
    result(0) = coordinate_after(0, 0);
    result(1) = coordinate_after(1, 0);
    return result;
}

Eigen::MatrixXd C_MPC_ROAD::calculate_Q(int i_Np, double d_q)
{
    Eigen::MatrixXd m_Q = Eigen::MatrixXd::Zero(2 * i_Np, 2 * i_Np);
    for (int i = 0; i < 2 * i_Np; i++)
    {
        if (i % 2 == 0)
        {
            m_Q(i, i) = d_q;
        }
        else
        {
            m_Q(i, i) = d_q;
        }
    }
    return m_Q;
}

Eigen::MatrixXd C_MPC_ROAD::calculate_R(int i_Nc, double d_r)
{
    Eigen::MatrixXd m_R = Eigen::MatrixXd::Zero(i_Nc, i_Nc);
    for (int i = 0; i < i_Nc; i++)
    {
        m_R(i, i) = d_r;
    }
    return m_R;
}

Eigen::Matrix<double, 2, 5> C_MPC_ROAD::calculate_C()
{
    Eigen::Matrix<double, 2, 5> _m_C;
    _m_C << 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
    return _m_C;
}

Eigen::Matrix<double, 2, 5> C_MPC_ROAD::calculate_C_XY()
{
    Eigen::Matrix<double, 2, 5> _m_C_XY;
    _m_C_XY << 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;
    return _m_C_XY;
}

void C_MPC_ROAD::calculate_coordinateDirection()
{
    double d_sum = 0.0;
    for (int index = i_nearstIndex; index < pathsWithAngleRef.size() && index < i_nearstIndex + i_num; index++)
    {
        d_sum += pathsWithAngleRef[index].theta;
    }
    double dnumMin = (pathsWithAngleRef.size() - i_nearstIndex > i_num) ? (i_num) : (pathsWithAngleRef.size() - i_nearstIndex);
    d_thetaAvg = d_sum / dnumMin;
}

void C_MPC_ROAD::calculate_Q_R_C()
{
    // 计算Q与R权重矩阵
    Eigen::MatrixXd m_Q = calculate_Q(optData.i_Np, d_q);
    Eigen::MatrixXd m_R = calculate_R(optData.i_Nc, d_r);

    // 计算C矩阵
    Eigen::Matrix<double, 2, 5> m_C = calculate_C();
    Eigen::Matrix<double, 2, 5> m_C_XY = calculate_C_XY();

    optData.m_Q = m_Q;
    optData.m_R = m_R;
    optData.m_C = m_C;
    optData.m_C_XY = m_C_XY;

    optData.m_CExpanded = Eigen::MatrixXd(2 * optData.i_Np, 5 * optData.i_Np);
    optData.m_CExpanded.setZero();
    // 在对角线上放置m_C的副本
    for (int i = 0; i < optData.i_Np; ++i)
    {
        optData.m_CExpanded.block(i * 2, i * 5, 2, 5) = m_C;
    }

    optData.m_C_XYExpanded = Eigen::MatrixXd(2 * optData.i_Np, 5 * optData.i_Np);
    optData.m_C_XYExpanded.setZero();
    // 在对角线上放置m_C_XY的副本
    for (int i = 0; i < optData.i_Np; ++i)
    {
        optData.m_C_XYExpanded.block(i * 2, i * 5, 2, 5) = m_C_XY;
    }
}

void C_MPC_ROAD::calculate_xi0()
{
    optData.v_xi0 = 0.0 * Eigen::VectorXd::Ones(5);
    optData.v_xi0(0) = poseNow.d_v0;
    optData.v_xi0(1) = 0.0;
    optData.v_xi0(2) = poseNow.d_theta0 - d_thetaAvg;
    optData.v_xi0(3) = 0.0;
    optData.v_xi0(4) = 0.0;
}

void C_MPC_ROAD::calculate_etaRef()
{
    optData.v_etaRef = 0.0 * Eigen::VectorXd::Ones(2 * optData.i_Np);
    Eigen::VectorXd temp;
    for (int index = 0; index < optData.i_Np; index++)
    {
        temp = calculate_coordinateConversionFromWorld(pathsWithAngleRef[i_nearstIndex + index].x, pathsWithAngleRef[i_nearstIndex + index].y, d_thetaAvg, poseNow.d_x0, poseNow.d_y0);
        optData.v_etaRef(2 * index + 1) = temp(1);
        optData.v_etaRef(2 * index) = pathsWithAngleRef[i_nearstIndex + index].theta - d_thetaAvg;
    }
}

void C_MPC_ROAD::calculate_sideOb()
{
    for (int index = 0; index < optData.i_N; index++)
    {
        Eigen::VectorXd temp = calculate_coordinateConversionFromWorld(optData.v_sideOb(2 * index), optData.v_sideOb(2 * index + 1), d_thetaAvg, poseNow.d_x0, poseNow.d_y0);
        optData.v_sideOb(2 * index) = temp(0);
        optData.v_sideOb(2 * index + 1) = temp(1);
    }
}

void C_MPC_ROAD::calculate_nearstIndex()
{
    std::vector<double> dis_gather(pathsWithAngleRef.size());
    for (int i = 0; i < pathsWithAngleRef.size(); i++)
    {
        dis_gather[i] = PLOT_DIS(poseNow.d_x0, poseNow.d_y0, pathsWithAngleRef[i].x, pathsWithAngleRef[i].y);
    }
    i_nearstIndex = std::min_element(dis_gather.begin(), dis_gather.end()) - dis_gather.begin();
}

// 输入：无
// 输出：中间变量d_xdotdot
double C_MPC_ROAD::calculate_xdotdot()
{
    return 0;
}

// 输入：v_ay_Np,i
// 输出：中间变量ydotdot
double C_MPC_ROAD::calculate_ydotdot(Eigen::VectorXd v_ay_Np, int i)
{
    return v_ay_Np(i);
}

// 输入：v_ay_Np,v_xi,i
// 输出：中间变量phidot
double C_MPC_ROAD::calculate_phidot(Eigen::VectorXd v_ay_Np, Eigen::VectorXd v_xi, int i)
{
    return (v_ay_Np(i) / v_xi(5 * i));
}

// 输入：v_xi,i
// 输出：中间变量Xdot
double C_MPC_ROAD::calculate_Xdot(Eigen::VectorXd v_xi, int i)
{
    return (v_xi(5 * i) * cos(v_xi(5 * i + 2)) - v_xi(5 * i + 1) * sin(v_xi(5 * i + 2)));
}

// 输入：v_xi,i
// 输出：中间变量Ydot
double C_MPC_ROAD::calculate_Ydot(Eigen::VectorXd v_xi, int i)
{
    return (v_xi(5 * i) * sin(v_xi(5 * i + 2)) + v_xi(5 * i + 1) * cos(v_xi(5 * i + 2)));
}

// 输入：v_ay_Np,v_xi0,d_T,i_Np
// 输出：v_xi
Eigen::VectorXd C_MPC_ROAD::calculate_xi(Eigen::VectorXd v_ay_Np, Eigen::VectorXd v_xi0, double d_T, int i_Np)
{
    Eigen::VectorXd v_xi = 0.0 * Eigen::VectorXd::Ones(5 * i_Np); // 初始化状态量
    v_xi(0) = v_xi0(0);
    v_xi(1) = v_xi0(1);
    v_xi(2) = v_xi0(2);
    v_xi(3) = v_xi0(3);
    v_xi(4) = v_xi0(4);

    // 下面是用于前向欧拉法的中间变量
    // 状态量的导数
    double d_xdotdot = calculate_xdotdot();
    double d_ydotdot = calculate_ydotdot(v_ay_Np, 0);
    double d_phidot = calculate_phidot(v_ay_Np, v_xi, 0);
    double d_Xdot = calculate_Xdot(v_xi, 0);
    double d_Ydot = calculate_Ydot(v_xi, 0);
    for (int i = 1; i < i_Np; i++)
    {
        v_xi(5 * i) = v_xi(5 * (i - 1)) + d_T * d_xdotdot;
        v_xi(5 * i + 1) = v_xi(5 * (i - 1) + 1) + d_T * d_ydotdot;
        v_xi(5 * i + 2) = v_xi(5 * (i - 1) + 2) + d_T * d_phidot;
        v_xi(5 * i + 3) = v_xi(5 * (i - 1) + 3) + d_T * d_Xdot;
        v_xi(5 * i + 4) = v_xi(5 * (i - 1) + 4) + d_T * d_Ydot;

        d_xdotdot = calculate_xdotdot();
        d_ydotdot = calculate_ydotdot(v_ay_Np, i);
        d_phidot = calculate_phidot(v_ay_Np, v_xi, i);
        d_Xdot = calculate_Xdot(v_xi, i);
        d_Ydot = calculate_Ydot(v_xi, i);
    }
    return v_xi;
}

// 输入：v_ay_Nc(真正变化的自变量，它包含了从1到Nc),i_Nc,i_Np
// 输出：v_ay_Np(用于作为工具计算状态量，它包含了从1到Np)
Eigen::VectorXd C_MPC_ROAD::calculate_ay(Eigen::VectorXd v_ay_Nc, int i_Np, int i_Nc)
{
    Eigen::VectorXd v_ay_Np = 0.0 * Eigen::VectorXd::Ones(i_Np); // 初始化ay
    for (int i = 0; i < i_Nc; i++)
    {
        v_ay_Np(i) = v_ay_Nc(i);
    }
    for (int i = i_Nc; i < i_Np; i++)
    {
        v_ay_Np(i) = v_ay_Nc(i_Nc - 1);
    }
    return v_ay_Np;
}

// 输入：v_ay_Nc(自变量),v_xi0,d_T,i_Np,i_Nc,m_C
// 输出：v_eta
C_MPC_ROAD::S_xi_eta C_MPC_ROAD::calculate_eta(Eigen::VectorXd v_ay_Nc, Eigen::VectorXd v_xi0, double d_T, int i_Np, int i_Nc, Eigen::MatrixXd m_CExpanded)
{
    Eigen::VectorXd v_ay_Np = calculate_ay(v_ay_Nc, i_Np, i_Nc);

    // 状态量，它的维数是5Np，它的内容是xdot,ydot,phi,X,Y一直排列下去
    Eigen::VectorXd v_xi = calculate_xi(v_ay_Np, v_xi0, d_T, i_Np);

    // 输出量，它的维数是2Np，它的内容是phi,Y一直排列下去
    Eigen::VectorXd v_eta = m_CExpanded * v_xi;

    S_xi_eta xi_eta;
    xi_eta = {v_xi, v_eta};
    return xi_eta;
}

// 输入：v_ay_Nc
// 输出：v_u
Eigen::VectorXd C_MPC_ROAD::calculate_u(Eigen::VectorXd v_ay_Nc)
{
    Eigen::VectorXd v_u = v_ay_Nc;
    return v_u;
}

// 输入：i_Np,i_N,d_Sob,d_zeta,v_xi,v_sideOb
double C_MPC_ROAD::calculate_Job(Eigen::VectorXd v_xi, int i_Np, int i_N, double d_Sob, double d_zeta, Eigen::VectorXd v_sideOb)
{
    double d_Job = 0;
    for (int j = 0; j < i_Np; j++)
    {
        for (int i = 0; i < i_N; i++)
        {
            d_Job += d_Sob * (v_xi(5 * j) * v_xi(5 * j) + v_xi(5 * j + 1) * v_xi(5 * j + 1)) /
                     ((v_sideOb(2 * i) - v_xi(5 * j + 3)) * (v_sideOb(2 * i) - v_xi(5 * j + 3)) + (v_sideOb(2 * i + 1) - v_xi(5 * j + 4)) * (v_sideOb(2 * i + 1) - v_xi(5 * j + 4)) + d_zeta);
        }
    }
    return d_Job;
}

double C_MPC_ROAD::utility(unsigned n, const double *x, double *grad, void *data)
{
    OptimizationData *optData = reinterpret_cast<OptimizationData *>(data);
    // 这里实际上v_x就是a_y，n的长度是Nc
    Eigen::Map<const Eigen::VectorXd> v_x(x, n);

    S_xi_eta xi_eta = calculate_eta(v_x, optData->v_xi0, optData->d_T, optData->i_Np, optData->i_Nc, optData->m_CExpanded);
    // 控制量，长度是Nc
    Eigen::VectorXd v_u = calculate_u(v_x);
    double d_Job = calculate_Job(xi_eta.v_xi, optData->i_Np, optData->i_N, optData->d_Sob, optData->d_zeta, optData->v_sideOb);

    Eigen::VectorXd deltaEta = xi_eta.v_eta - optData->v_etaRef;
    for (int index = 0; index < optData->i_Np; index++)
    {
        while (deltaEta[2 * index] > M_PI)
        {
            deltaEta[2 * index] -= 2 * M_PI;
        }
        while (deltaEta[2 * index] <= -M_PI)
        {
            deltaEta[2 * index] += 2 * M_PI;
        }
    }
    double result1 = (deltaEta.transpose() * optData->m_Q * deltaEta).value();
    double result2 = (v_u.transpose() * optData->m_R * v_u).value();

    double result = result1 + result2 + d_Job;

    return result;
}

Eigen::VectorXd C_MPC_ROAD::filterElements(const Eigen::VectorXd &original, int i_N)
{
    std::vector<double> temp;
    temp.reserve(original.size()); // 优化，预先分配足够的空间

    for (int index = 0; index < i_N; ++index)
    {
        // 检查条件是否满足
        if (!(original(2 * index) < -0.5 || original(2 * index + 1) > 2.0 || original(2 * index + 1) < -2.0))
        {
            // 如果不需要删除，添加到临时vector中
            if (2 * index < original.size())
            {
                temp.push_back(original(2 * index));
            }
            if (2 * index + 1 < original.size())
            {
                temp.push_back(original(2 * index + 1));
            }
        }
        // 如果条件满足，这里就跳过了当前和下一个元素，相当于删除
    }

    // 将temp的内容复制到新的VectorXd
    Eigen::VectorXd result = Eigen::Map<Eigen::VectorXd>(temp.data(), temp.size());
    return result;
}

Eigen::VectorXd C_MPC_ROAD::get_road_best()
{
    // 计算中间变量
    calculate_Q_R_C();

    calculate_nearstIndex();
    calculate_coordinateDirection();
    calculate_xi0();
    calculate_etaRef();
    calculate_sideOb();

    optData.v_sideOb = filterElements(optData.v_sideOb, optData.i_N);
    optData.i_N = optData.v_sideOb.size() / 2;

    double tol = 1e-8;

    double f_min = INF;

    nlopt::opt opter(nlopt::algorithm::LN_COBYLA, optData.i_Nc);

    std::vector<double> vd_aymin(v_aymin.data(), v_aymin.data() + v_aymin.size());
    std::vector<double> vd_aymax(v_aymax.data(), v_aymax.data() + v_aymax.size());
    opter.set_lower_bounds(vd_aymin);
    opter.set_upper_bounds(vd_aymax);

    opter.set_min_objective(utility, &optData);

    opter.set_xtol_rel(tol);
    opter.set_ftol_abs(tol);

    opter.set_force_stop(tol);

    nlopt::result result = opter.optimize(vd_x, f_min);

    if (result)
    {
        Eigen::VectorXd v_ay_Nc = Eigen::VectorXd::Map(vd_x.data(), vd_x.size());
        Eigen::VectorXd v_ay_Np = calculate_ay(v_ay_Nc, optData.i_Np, optData.i_Nc);
        Eigen::VectorXd v_xi = calculate_xi(v_ay_Np, optData.v_xi0, optData.d_T, optData.i_Np);
        Eigen::VectorXd v_XY = optData.m_C_XYExpanded * v_xi;
        for (int index = 0; index < optData.i_Np; index++) // 需要修改
        {
            Eigen::VectorXd temp = calculate_coordinateConversionToWorld(v_XY(2 * index), v_XY(2 * index + 1), d_thetaAvg, poseNow.d_x0, poseNow.d_y0);
            v_XY(2 * index) = temp(0);
            v_XY(2 * index + 1) = temp(1);
        }
        return v_XY;
    }
}
