#include "mpc_control.h"
C_MPC_CONTROLROAD::C_MPC_CONTROLROAD(int _i_n, int _i_m, double _i_Nc)
    : i_n(_i_n),
      i_m(_i_m),
      i_Nc(_i_Nc)
{
    p_u = new double[i_m];
    p_chi = new double[i_n];
    p_uRef = new double[i_m];
    p_chiRef = new double[i_n];
    p_Ut = new double[2];
    p_Ut[0] = 0;
    p_Ut[1] = 0;

    v_deltaUt = Eigen::VectorXd::Zero(i_m * i_Nc);
    v_Ut = Eigen::VectorXd::Zero(i_m * i_Nc);
}
C_MPC_CONTROLROAD::C_MPC_CONTROLROAD(int _i_n, int _i_m, int _i_Np, int _i_Nc, double _d_k, double _d_T, double _d_qWeightStateOffset, double _d_rWeightControlIncrement, double _d_epsilonRelaxationFactor, double _d_rhoWeightCoefficient, double _d_l,
                                     Eigen::VectorXd _v_deltaUmin, Eigen::VectorXd _v_deltaUmax, Eigen::VectorXd _v_Umin, Eigen::VectorXd _v_Umax, double *_p_u = nullptr, double *_p_chi = nullptr, double *_p_uRef = nullptr, double *_p_chiRef = nullptr)
    : i_n(_i_n),
      i_m(_i_m),
      i_Np(_i_Np),
      i_Nc(_i_Nc),
      d_k(_d_k),
      d_T(_d_T),
      d_qWeightStateOffset(_d_qWeightStateOffset),
      d_rWeightControlIncrement(_d_rWeightControlIncrement),
      d_epsilonRelaxationFactor(_d_epsilonRelaxationFactor),
      d_rhoWeightCoefficient(_d_rhoWeightCoefficient),
      d_l(_d_l),
      v_deltaUmin(_v_deltaUmin),
      v_deltaUmax(_v_deltaUmax),
      v_Umin(_v_Umin),
      v_Umax(_v_Umax)
{
    if (_p_u != nullptr)
    {
        p_u = new double[i_m]; // 改为变量
        memcpy(p_u, _p_u, sizeof(double) * i_m);
    }
    else
    {
        p_u = nullptr;
    }

    if (_p_chi != nullptr)
    {
        p_chi = new double[2 * i_n];
        memcpy(p_chi, _p_chi, sizeof(double) * 2 * i_n);
    }
    else
    {
        p_chi = nullptr;
    }

    if (_p_uRef != nullptr)
    {
        p_uRef = new double[i_m];
        memcpy(p_uRef, _p_uRef, sizeof(double) * i_m);
    }
    else
    {
        p_uRef = nullptr;
    }

    if (_p_chiRef != nullptr)
    {
        p_chiRef = new double[2 * i_n];
        memcpy(p_chiRef, _p_chiRef, sizeof(double) * 2 * i_n);
    }
    else
    {
        p_chiRef = nullptr;
    }

    p_Ut = new double[2];
    p_Ut[0] = 0;
    p_Ut[1] = 0;

    v_deltaUt = Eigen::VectorXd::Zero(i_m * i_Nc);
    v_Ut = Eigen::VectorXd::Zero(i_m * i_Nc);
}

static C_MPC_CONTROLROAD &GetOnlyCmpc(int _i_n, int _i_m, int _i_Np, int _i_Nc, double _d_k, double _d_T, double _d_qWeightStateOffset, double _d_rWeightControlIncrement, double _d_epsilonRelaxationFactor, double _d_rhoWeightCoefficient, double _d_l,
                                      Eigen::VectorXd _v_deltaUmin, Eigen::VectorXd _v_deltaUmax, Eigen::VectorXd _v_Umin, Eigen::VectorXd _v_Umax, double *_p_u = nullptr, double *_p_chi = nullptr, double *_p_uRef = nullptr, double *_p_chiRef = nullptr)
{
    // 懒汉模式，不适合多线程
    static C_MPC_CONTROLROAD onlyInstance(_i_n, _i_m, _i_Np, _i_Nc, _d_k, _d_T, _d_qWeightStateOffset, _d_rWeightControlIncrement, _d_epsilonRelaxationFactor, _d_rhoWeightCoefficient, _d_l, _v_deltaUmin, _v_deltaUmax, _v_Umin, _v_Umax, _p_u, _p_chi, _p_uRef, _p_chiRef);
    return onlyInstance;
}

// 复制构造函数，若放到private里面则不允许复制
C_MPC_CONTROLROAD::C_MPC_CONTROLROAD(const C_MPC_CONTROLROAD &other)
    : i_n(other.i_n),
      i_m(other.i_m),
      i_Np(other.i_Np),
      i_Nc(other.i_Nc),
      d_k(other.d_k),
      d_T(other.d_T),
      d_qWeightStateOffset(other.d_qWeightStateOffset),
      d_rWeightControlIncrement(other.d_rWeightControlIncrement),
      d_epsilonRelaxationFactor(other.d_epsilonRelaxationFactor),
      d_rhoWeightCoefficient(other.d_rhoWeightCoefficient),
      d_l(other.d_l),
      v_deltaUmin(other.v_deltaUmin),
      v_deltaUmax(other.v_deltaUmax),
      v_Umin(other.v_Umin),
      v_Umax(other.v_Umax),
      v_deltaUt(other.v_deltaUt),
      v_Ut(other.v_Ut),
      m_QWeightStateOffset(other.m_QWeightStateOffset),
      m_RWeightControlIncrement(other.m_RWeightControlIncrement),
      m_C(other.m_C),
      m_A1(other.m_A1),
      m_B1(other.m_B1),
      m_A2(other.m_A2),
      m_B2(other.m_B2),
      m_EDeviation(other.m_EDeviation),
      m_psi(other.m_psi),
      m_theta(other.m_theta),
      m_xiCurrentState(other.m_xiCurrentState),
      m_H(other.m_H),
      v_f(other.v_f),
      m_I(other.m_I)
{
    // 为动态分配的内存执行深复制
    if (other.p_u != nullptr)
    {
        p_u = new double[i_m];
        memcpy(p_u, other.p_u, sizeof(double) * i_m);
    }
    else
    {
        p_u = nullptr;
    }

    if (other.p_chi != nullptr)
    {
        p_chi = new double[2 * i_n];
        memcpy(p_chi, other.p_chi, sizeof(double) * 2 * i_n);
    }
    else
    {
        p_chi = nullptr;
    }

    if (other.p_uRef != nullptr)
    {
        p_uRef = new double[i_m];
        memcpy(p_uRef, other.p_uRef, sizeof(double) * i_m);
    }
    else
    {
        p_uRef = nullptr;
    }

    if (other.p_chiRef != nullptr)
    {
        p_chiRef = new double[2 * i_n];
        memcpy(p_chiRef, other.p_chiRef, sizeof(double) * 2 * i_n);
    }
    else
    {
        p_chiRef = nullptr;
    }

    // 由于 p_Ut 是一个固定大小的数组，可以直接复制
    p_Ut = new double[2];
    memcpy(p_Ut, other.p_Ut, sizeof(double) * 2);
}

// 复制赋值运算符
C_MPC_CONTROLROAD &C_MPC_CONTROLROAD::operator=(const C_MPC_CONTROLROAD &other)
{
    if (this != &other) // 防止自赋值
    {
        // 释放当前对象的资源
        delete[] p_u;
        delete[] p_chi;
        delete[] p_uRef;
        delete[] p_chiRef;
        delete[] p_Ut;

        // 复制非动态分配的成员
        i_n = other.i_n;
        i_m = other.i_m;
        i_Np = other.i_Np;
        i_Nc = other.i_Nc;
        d_k = other.d_k;
        d_T = other.d_T;
        d_qWeightStateOffset = other.d_qWeightStateOffset;
        d_rWeightControlIncrement = other.d_rWeightControlIncrement;
        d_epsilonRelaxationFactor = other.d_epsilonRelaxationFactor;
        d_rhoWeightCoefficient = other.d_rhoWeightCoefficient;
        d_l = other.d_l;
        v_deltaUmin = other.v_deltaUmin;
        v_deltaUmax = other.v_deltaUmax;
        v_Umin = other.v_Umin;
        v_Umax = other.v_Umax;
        v_deltaUt = other.v_deltaUt;
        v_Ut = other.v_Ut;
        m_QWeightStateOffset = other.m_QWeightStateOffset;
        m_RWeightControlIncrement = other.m_RWeightControlIncrement;
        m_C = other.m_C;
        m_A1 = other.m_A1;
        m_B1 = other.m_B1;
        m_A2 = other.m_A2;
        m_B2 = other.m_B2;
        m_EDeviation = other.m_EDeviation;
        m_psi = other.m_psi;
        m_theta = other.m_theta;
        m_xiCurrentState = other.m_xiCurrentState;
        m_H = other.m_H;
        v_f = other.v_f;
        m_I = other.m_I;

        // 为动态分配的内存执行深复制
        if (other.p_u != nullptr)
        {
            p_u = new double[i_m];
            memcpy(p_u, other.p_u, sizeof(double) * i_m);
        }
        else
        {
            p_u = nullptr;
        }

        if (other.p_chi != nullptr)
        {
            p_chi = new double[2 * i_n];
            memcpy(p_chi, other.p_chi, sizeof(double) * 2 * i_n);
        }
        else
        {
            p_chi = nullptr;
        }

        if (other.p_uRef != nullptr)
        {
            p_uRef = new double[i_m];
            memcpy(p_uRef, other.p_uRef, sizeof(double) * i_m);
        }
        else
        {
            p_uRef = nullptr;
        }

        if (other.p_chiRef != nullptr)
        {
            p_chiRef = new double[2 * i_n];
            memcpy(p_chiRef, other.p_chiRef, sizeof(double) * 2 * i_n);
        }
        else
        {
            p_chiRef = nullptr;
        }

        // 由于 p_Ut 是一个固定大小的数组，可以直接复制
        p_Ut = new double[2];
        memcpy(p_Ut, other.p_Ut, sizeof(double) * 2);
    }

    return *this; // 返回当前对象的引用
}

// 移动构造函数
C_MPC_CONTROLROAD::C_MPC_CONTROLROAD(C_MPC_CONTROLROAD &&other) noexcept
    : i_n(other.i_n),
      i_m(other.i_m),
      i_Np(other.i_Np),
      i_Nc(other.i_Nc),
      d_k(other.d_k),
      d_T(other.d_T),
      d_qWeightStateOffset(other.d_qWeightStateOffset),
      d_rWeightControlIncrement(other.d_rWeightControlIncrement),
      d_epsilonRelaxationFactor(other.d_epsilonRelaxationFactor),
      d_rhoWeightCoefficient(other.d_rhoWeightCoefficient),
      d_l(other.d_l),
      v_deltaUmin(std::move(other.v_deltaUmin)),
      v_deltaUmax(std::move(other.v_deltaUmax)),
      v_Umin(std::move(other.v_Umin)),
      v_Umax(std::move(other.v_Umax)),
      v_deltaUt(std::move(other.v_deltaUt)),
      v_Ut(std::move(other.v_Ut)),
      m_QWeightStateOffset(std::move(other.m_QWeightStateOffset)),
      m_RWeightControlIncrement(std::move(other.m_RWeightControlIncrement)),
      m_C(std::move(other.m_C)),
      m_A1(std::move(other.m_A1)),
      m_B1(std::move(other.m_B1)),
      m_A2(std::move(other.m_A2)),
      m_B2(std::move(other.m_B2)),
      m_EDeviation(std::move(other.m_EDeviation)),
      m_psi(std::move(other.m_psi)),
      m_theta(std::move(other.m_theta)),
      m_xiCurrentState(std::move(other.m_xiCurrentState)),
      m_H(std::move(other.m_H)),
      v_f(std::move(other.v_f)),
      m_I(std::move(other.m_I)),
      p_u(other.p_u),
      p_chi(other.p_chi),
      p_uRef(other.p_uRef),
      p_chiRef(other.p_chiRef),
      p_Ut(other.p_Ut)
{
    other.p_u = nullptr;
    other.p_chi = nullptr;
    other.p_uRef = nullptr;
    other.p_chiRef = nullptr;
    other.p_Ut = nullptr;
}

// 移动赋值运算符
C_MPC_CONTROLROAD &C_MPC_CONTROLROAD::operator=(C_MPC_CONTROLROAD &&other) noexcept
{
    if (this != &other)
    {
        delete[] p_u;
        delete[] p_chi;
        delete[] p_uRef;
        delete[] p_chiRef;
        delete[] p_Ut;

        i_n = other.i_n;
        i_m = other.i_m;
        i_Np = other.i_Np;
        i_Nc = other.i_Nc;
        d_k = other.d_k;
        d_T = other.d_T;
        d_qWeightStateOffset = other.d_qWeightStateOffset;
        d_rWeightControlIncrement = other.d_rWeightControlIncrement;
        d_epsilonRelaxationFactor = other.d_epsilonRelaxationFactor;
        d_rhoWeightCoefficient = other.d_rhoWeightCoefficient;
        d_l = other.d_l;
        v_deltaUmin = std::move(other.v_deltaUmin); // 使用std::move比直接=更快
        v_deltaUmax = std::move(other.v_deltaUmax);
        v_Umin = std::move(other.v_Umin);
        v_Umax = std::move(other.v_Umax);
        v_deltaUt = std::move(other.v_deltaUt);
        v_Ut = std::move(other.v_Ut);
        m_QWeightStateOffset = std::move(other.m_QWeightStateOffset);
        m_RWeightControlIncrement = std::move(other.m_RWeightControlIncrement);
        m_C = std::move(other.m_C);
        m_A1 = std::move(other.m_A1);
        m_B1 = std::move(other.m_B1);
        m_A2 = std::move(other.m_A2);
        m_B2 = std::move(other.m_B2);
        m_EDeviation = std::move(other.m_EDeviation);
        m_psi = std::move(other.m_psi);
        m_theta = std::move(other.m_theta);
        m_xiCurrentState = std::move(other.m_xiCurrentState);
        m_H = std::move(other.m_H);
        v_f = std::move(other.v_f);
        m_I = std::move(other.m_I);
        p_u = other.p_u;
        p_chi = other.p_chi;
        p_uRef = other.p_uRef;
        p_chiRef = other.p_chiRef;
        p_Ut = other.p_Ut;

        other.p_u = nullptr; // 将此前的销毁
        other.p_chi = nullptr;
        other.p_uRef = nullptr;
        other.p_chiRef = nullptr;
        other.p_Ut = nullptr;
    }
    return *this;
}

// 如果为private则只能new创建而不能在栈上面创建
C_MPC_CONTROLROAD::~C_MPC_CONTROLROAD()
{
    delete[] p_u;
    delete[] p_chi;
    delete[] p_uRef;
    delete[] p_chiRef;
    delete[] p_Ut;
}

bool C_MPC_CONTROLROAD::set_n(int _i_n)
{
    if (_i_n > 0)
    {
        i_n = _i_n;
        return true;
    }
    return false;
}

bool C_MPC_CONTROLROAD::set_m(int _i_m)
{
    if (_i_m > 0)
    {
        i_m = _i_m;
        return true;
    }
    return false;
}

bool C_MPC_CONTROLROAD::set_Nc_Np(int _i_Nc, int _i_Np)
{
    if (_i_Nc > 0 && _i_Np > 0)
    {
        i_Nc = _i_Nc;
        i_Np = _i_Np;
        return true;
    }
    return false;
}

bool C_MPC_CONTROLROAD::set_k(double _d_k)
{
    d_k = _d_k;
    return true;
}

bool C_MPC_CONTROLROAD::set_T(double _d_T)
{
    if (_d_T > 0)
    {
        d_T = _d_T;
        return true;
    }
    return false;
}

bool C_MPC_CONTROLROAD::set_q_r(double _d_q_weightStateOffset, double _d_r_weightControlIncrement)
{
    if (_d_q_weightStateOffset > 0 && _d_r_weightControlIncrement > 0)
    {
        d_qWeightStateOffset = _d_q_weightStateOffset;
        d_rWeightControlIncrement = _d_r_weightControlIncrement;
        return true;
    }
    return false;
}

bool C_MPC_CONTROLROAD::set_epsilon_rho(double _d_epsilonRelaxationFactor, double _d_rhoWeightCoefficient)
{
    if (_d_epsilonRelaxationFactor > 0 && _d_rhoWeightCoefficient > 0)
    {
        d_epsilonRelaxationFactor = _d_epsilonRelaxationFactor;
        d_rhoWeightCoefficient = _d_rhoWeightCoefficient;
        return true;
    }
    return false;
}

bool C_MPC_CONTROLROAD::set_l(double _d_l)
{
    if (_d_l > 0)
    {
        d_l = _d_l;
        return true;
    }
    return false;
}

bool C_MPC_CONTROLROAD::set_deltaUmin_deltaUmax_Umin_Umax(Eigen::VectorXd _v_deltaUmin, Eigen::VectorXd _v_deltaUmax, Eigen::VectorXd _v_Umin, Eigen::VectorXd _v_Umax)
{
    if (_v_deltaUmin.size() != 0 && _v_deltaUmax.size() != 0 && _v_Umin.size() != 0 && _v_Umax.size() != 0)
    {
        v_deltaUmin = _v_deltaUmin;
        v_deltaUmax = _v_deltaUmax;
        v_Umin = _v_Umin;
        v_Umax = _v_Umax;
        return true;
    }
    return false;
}

bool C_MPC_CONTROLROAD::set_u(double *_p_u)
{
    if (_p_u != nullptr)
    {
        p_u = _p_u;
        return true;
    }
    return false;
}

bool C_MPC_CONTROLROAD::set_uRef(double *_p_uRef)
{
    if (_p_uRef != nullptr)
    {
        p_uRef = _p_uRef;
        return true;
    }
    return false;
}

bool C_MPC_CONTROLROAD::set_chi(double *_p_chi)
{
    if (_p_chi != nullptr)
    {
        p_chi = _p_chi;
        return true;
    }
    return false;
}

bool C_MPC_CONTROLROAD::set_chiRef(double *_p_chiRef)
{
    if (_p_chiRef != nullptr)
    {
        p_chiRef = _p_chiRef;
        return true;
    }
    return false;
}

// 获得一个i_row * 2 * i_n行2 * i_n列的全零矩阵
Eigen::MatrixXd C_MPC_CONTROLROAD::get_zero(int _i_row)
{
    Eigen::MatrixXd m_res;
    for (int index = 1; index <= _i_row; ++index)
    {
        if (index == 1)
        {
            m_res = Eigen::MatrixXd::Zero(2 * i_n, 2 * i_n);
        }
        else
        {
            Eigen::MatrixXd m_temp = Eigen::MatrixXd::Zero(index * 2 * i_n, 2 * i_n);
            m_temp.block(0, 0, m_res.rows(), m_res.cols()) = m_res;
            m_res = m_temp;
        }
    }
    return m_res;
}

// 函数的功能是实现矩阵的C次幂（为一个幂乘功能，多次使用，因此单独写出来）
Eigen::MatrixXd C_MPC_CONTROLROAD::Matrix_power(Eigen::MatrixXd &_M, int _c, int _n)
{
    if (_c == 0)
    {
        return Eigen::MatrixXd::Identity(_n, _n); // MatrixXd为返回值类型，因为不确定维度，所以用这个
    }
    Eigen::MatrixXd res = _M;    // 定义一个不指定维度的矩阵res，M的维度就是res的维度
    for (int i = 1; i < _c; ++i) // C代表要乘几次，从0开始循环，到C-1停止
    {
        res = res * _M;
    }
    return res;
}

Eigen::Matrix<double, 2, 2> C_MPC_CONTROLROAD::get_A(int _index, double *_p_uRef, double *_p_chiRef)
{
    Eigen::Matrix<double, 2, 2> m_A;
    m_A << 0, _p_uRef[2 * _index] * cos(_p_chiRef[2 * _index + 1]), 0, 0;
    return m_A;
}

Eigen::Matrix<double, 2, 2> C_MPC_CONTROLROAD::get_B(int _index, double *_p_uRef, double *_p_chiRef, double _d_l, double _d_k)
{
    Eigen::Matrix<double, 2, 2> m_B;
    m_B << sin(_p_chiRef[2 * _index + 1]), 0, cos(_p_chiRef[2 * _index + 1]) * _d_k - tan(_p_uRef[2 * _index + 1]) / _d_l, -_p_uRef[2 * _index] / (_d_l * cos(_p_uRef[2 * _index + 1]) * cos(_p_uRef[2 * _index + 1]));
    return m_B;
}

// 计算psi
Eigen::MatrixXd C_MPC_CONTROLROAD::get_psi(Eigen::MatrixXd &_m_A2, int _i_Np) // 输入没有C，是因为C为固定值，可以写在函数里面，而A2和Np不是固定值
{
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_psi; // 和MatrixXd一个意思，都表示动态维度
    for (int i = 1; i <= _i_Np; ++i)                             // 从1到Np次方
    {
        auto _m_temp = Matrix_power(_m_A2, i, 4 * i_n);
        Eigen::MatrixXd _m_psi = m_C * _m_temp;
        if (i == 1) // 这里的if else 目的是把_psi拼接到psi后面
        {
            m_psi = _m_psi;
        }
        else
        {
            Eigen::MatrixXd m_temp;
            m_temp.resize(i * 2 * i_n, 4 * i_n);
            m_temp << m_psi, _m_psi;
            m_psi = m_temp;
        }
    }
    return m_psi;
}

// 计算theta
Eigen::MatrixXd C_MPC_CONTROLROAD::get_theta(Eigen::MatrixXd &_m_A2, Eigen::MatrixXd &_m_B2, int _i_Nc, int _i_Np)
{
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_row;
    for (int i = 1; i <= _i_Nc; ++i) // 对于行数来说，j从1到Nc
    {
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> _m_row;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_col;
        for (int j = 0; j <= _i_Np - i; ++j)
        {
            auto _temp = Matrix_power(_m_A2, j, 4 * i_n);
            Eigen::MatrixXd _m_col = m_C * _temp * _m_B2; // C*A2的0到Np-1次幂*B2
            if (j == 0)
            {
                m_col = _m_col; // 第一次时候行数没有尺寸，需要初始化=第一个CAB
            }
            else
            {
                Eigen::MatrixXd m_temp;                        // 创造一个临时矩阵，大小为之前的行数+现在的CAB
                m_temp.resize(j * 2 * i_n + 2 * i_n, 2 * i_n); // 之前的行数为i*3，现在+3是为现在的行数
                m_temp << m_col, _m_col;                       // 每一次循环一个新的CAB，添加到后面，循环完之后，得到一个CAB从0到Np-1次幂，竖着拼一起
                m_col = m_temp;
            }
        }
        if (i == 1)
        {
            m_row = m_col;
        }
        else
        {
            _m_row.resize(2 * i_n * _i_Np, 2 * i_n); // 把列的尺寸补成和行一样，然后和已经存在的行拼在一起
            Eigen::MatrixXd zero = get_zero(i - 1);
            _m_row << get_zero(i - 1), m_col;
            Eigen::MatrixXd m_temp(_i_Np * 2 * i_n, 2 * i_n * i);
            m_temp << m_row, _m_row;
            m_row = m_temp;
        }
    }
    return m_row;
}

// 计算xi
Eigen::MatrixXd C_MPC_CONTROLROAD::get_xi(double *_p_u, double *_p_chi, double *_p_uRef, double *_p_chiRef)
{
    Eigen::MatrixXd m_xi(4 * i_n, 1);
    for (int i = 0; i < i_n; i++)
    {
        double deltaPhi = p_chi[2 * i + 1] - _p_chiRef[2 * i + 1];
        while (deltaPhi > M_PI)
        {
            deltaPhi -= 2 * M_PI;
        }
        while (deltaPhi <= -M_PI)
        {
            deltaPhi += 2 * M_PI;
        }
        m_xi(4 * i, 0) = _p_chi[2 * i] - _p_chiRef[2 * i];
        m_xi(4 * i + 1, 0) = deltaPhi;
        m_xi(4 * i + 2, 0) = _p_u[2 * i] - _p_uRef[2 * i];
        m_xi(4 * i + 3, 0) = _p_u[2 * i + 1] - _p_uRef[2 * i + 1];
    }

    // 打开文件以追加模式
    std::ofstream file("output.txt", std::ios::app);
    if (file.is_open())
    {
        // 按指定格式将数据写入文件
        file << "v: " << _p_u[0] << ", vRef: " << _p_uRef[0] << ", delta: " << _p_u[1] << ", deltaRef: " << _p_uRef[1] << std::endl;

        // 关闭文件
        file.close();
    }
    else
    {
        std::cerr << "无法打开文件" << std::endl;
    }

    return m_xi;
}

// lqr
void C_MPC_CONTROLROAD::calculate_Q_R()
{
    m_QWeightStateOffset = Eigen::MatrixXd::Zero(2 * i_n * i_Np, 2 * i_n * i_Np);
    for (int i = 0; i < 2 * i_n * i_Np; i++)
    {
        if (i % 2 == 0)
        {
            m_QWeightStateOffset(i, i) = 10 * d_qWeightStateOffset;
        }
        else
        {
            m_QWeightStateOffset(i, i) = 20 * d_qWeightStateOffset;
        }
    }
    m_RWeightControlIncrement = Eigen::MatrixXd::Zero(2 * i_n * i_Nc, 2 * i_n * i_Nc);
    for (int i = 0; i < 2 * i_n * i_Nc; i++)
    {
        if (i % 2 == 0)
        {
            m_RWeightControlIncrement(i, i) = 50 * d_rWeightControlIncrement;
        }
        else
        {
            m_RWeightControlIncrement(i, i) = 80 * d_rWeightControlIncrement;
        }
    }
}

void C_MPC_CONTROLROAD::calculate_C()
{
    m_C = Eigen::MatrixXd::Zero(2 * i_n, 4 * i_n);
    Eigen::MatrixXd m_C0(2, 4);
    m_C0 << 1, 0, 0, 0,
        0, 1, 0, 0;

    for (int i = 0; i < i_n; ++i)
    {
        m_C.block(2 * i, 4 * i, 2, 4) = m_C0;
    }
}

// 添加一个公有成员函数，计算并存储结果在成员变量中
void C_MPC_CONTROLROAD::calculate_A1_B1()
{
    // 获得A和B的值
    std::vector<Eigen::MatrixXd> m_As(i_n, Eigen::MatrixXd(2, 2));
    for (int i = 0; i < i_n; ++i)
    {
        m_As[i] = get_A(i, p_uRef, p_chiRef);
    }
    Eigen::MatrixXd m_A = Eigen::MatrixXd::Zero(2 * i_n, 2 * i_n);
    for (int i = 0; i < i_n; ++i)
    {
        m_A.block(2 * i, 2 * i, 2, 2) = m_As[i];
    }

    std::vector<Eigen::MatrixXd> m_Bs(i_n, Eigen::MatrixXd(2, 2));
    for (int i = 0; i < i_n; ++i)
    {
        m_Bs[i] = get_B(i, p_uRef, p_chiRef, d_l, d_k);
    }
    Eigen::MatrixXd m_B = Eigen::MatrixXd::Zero(2 * i_n, 2 * i_n);
    for (int i = 0; i < i_n; ++i)
    {
        m_B.block(2 * i, 2 * i, 2, 2) = m_Bs[i];
    }

    // 计算 A_1 和 B_1 的值，并存储在成员变量中
    m_I = Eigen::MatrixXd::Identity(2 * i_n, 2 * i_n);
    m_A1 = d_T * m_A + m_I;
    m_B1 = d_T * m_B;
}

void C_MPC_CONTROLROAD::calculate_A2_B2()
{
    m_A2 = Eigen::MatrixXd::Zero(4 * i_n, 4 * i_n);

    // 将A1放置在左上角
    m_A2.block(0, 0, 2 * i_n, 2 * i_n) = m_A1;

    // 将B1放置在右上角
    m_A2.block(0, 2 * i_n, 2 * i_n, 2 * i_n) = m_B1;

    // 将单位矩阵I放置在右下角
    m_A2.block(2 * i_n, 2 * i_n, 2 * i_n, 2 * i_n) = m_I;

    m_B2 = Eigen::MatrixXd::Zero(4 * i_n, 2 * i_n);
    m_B2.block(0, 0, 2 * i_n, 2 * i_n) = m_B1;
    m_B2.block(2 * i_n, 0, 2 * i_n, 2 * i_n) = m_I;
}

void C_MPC_CONTROLROAD::calculate_psi_theta_xi()
{
    // 计算psi
    m_psi = get_psi(m_A2, i_Np);

    // 计算theta
    m_theta = get_theta(m_A2, m_B2, i_Nc, i_Np);

    // 计算xi
    m_xiCurrentState = get_xi(p_u, p_chi, p_uRef, p_chiRef);
}

void C_MPC_CONTROLROAD::calculate_E()
{
    m_EDeviation = m_psi * m_xiCurrentState;
}

void C_MPC_CONTROLROAD::calculate_H()
{
    m_H.resize(2 * i_n * i_Nc + 1, 2 * i_n * i_Nc + 1);
    Eigen::MatrixXd m_H1 = m_theta.transpose() * m_QWeightStateOffset * m_theta + m_RWeightControlIncrement;
    Eigen::VectorXd v_H2 = Eigen::VectorXd::Zero(2 * i_n * i_Nc);
    m_H << m_H1, v_H2, v_H2.transpose(), d_rhoWeightCoefficient;
}

void C_MPC_CONTROLROAD::calculate_f()
{
    Eigen::MatrixXd m_f = Eigen::MatrixXd(1, 2 * i_n * i_Nc + 1);
    m_f << 2 * m_EDeviation.transpose() * m_QWeightStateOffset * m_theta, 0;
    v_f = m_f.row(0);
}

bool C_MPC_CONTROLROAD::update_delta_Ut(Eigen::VectorXd _v_deltaUt)
{
    if (_v_deltaUt.size() != 0)
    {
        v_deltaUt.resize(_v_deltaUt.size());
        v_deltaUt = _v_deltaUt;
        return true;
    }
    return false;
}

bool C_MPC_CONTROLROAD::updata_Ut()
{
    if (v_deltaUt.size() != 2 * i_Nc)
    {
        return false;
    }

    v_Ut.resize(v_deltaUt.size());
    for (int i = 0; i < i_Nc; i++)
    {
        v_Ut(2 * i) = p_Ut[0];
        v_Ut(2 * i + 1) = p_Ut[1];
    }
    v_Ut = v_Ut + v_deltaUt;

    if (v_Ut(0) > v_Umax(0))
    {
        v_Ut(0) = v_Umax(0);
        v_deltaUt(0) = v_Umax(0) - p_Ut[0];
    }
    if (v_Ut(0) < v_Umin(0))
    {
        v_Ut(0) = v_Umin(0);
        v_deltaUt(0) = v_Umin(0) - p_Ut[0];
    }
    if (v_Ut(1) > v_Umax(1))
    {
        v_Ut(1) = v_Umax(1);
        v_deltaUt(1) = v_Umax(1) - p_Ut[1];
    }
    if (v_Ut(1) < v_Umin(1))
    {
        v_Ut(1) = v_Umin(1);
        v_deltaUt(1) = v_Umin(1) - p_Ut[1];
    }

    p_Ut[0] = v_Ut(0);
    p_Ut[1] = v_Ut(1);
    return true;
}

void C_MPC_CONTROLROAD::calculate_process()
{
    // 计算Q与R权重矩阵
    calculate_Q_R();
    // 计算C矩阵
    calculate_C();

    // 计算A_1和B_1
    calculate_A1_B1();
    // 计算A_2和B_2
    calculate_A2_B2();
    // 利用A_1,B_1,A_2,B_2去计算psi, theta, Xi
    calculate_psi_theta_xi();

    // 计算E
    calculate_E();
}

bool C_MPC_CONTROLROAD::get_control_best()
{
    // 计算中间变量
    calculate_process();

    // 计算H
    calculate_H();

    // 计算f
    calculate_f();

    // 求出J的最小值
    C_min_j c_min_j(i_m, i_Nc, d_epsilonRelaxationFactor, m_H, v_f, v_deltaUmin, v_deltaUmax, v_Umin, v_Umax, v_Ut, v_deltaUt);

    auto _v_deltaUt = c_min_j.calculate_deltaUt_Jmin();

    bool b_update_deltaUt = update_delta_Ut(_v_deltaUt);
    bool b_update_Ut = updata_Ut();
    return b_update_deltaUt && b_update_Ut;
    return 1;
}

Eigen::VectorXd C_MPC_CONTROLROAD::get_Ut()
{
    return v_Ut;
}

Eigen::VectorXd C_MPC_CONTROLROAD::get_deltaUt()
{
    return v_deltaUt;
}
