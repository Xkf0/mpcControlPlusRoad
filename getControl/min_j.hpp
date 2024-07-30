#ifndef MIN_J_H
#define MIN_J_H
#include <iostream>
#include <cmath>
#include <nlopt.hpp>
#include <eigen3/Eigen/Dense>
#include <inf.h>

class C_min_j
{
private:
    int m;

    int Nc;
    double epsilon;
    Eigen::MatrixXd H;
    Eigen::VectorXd f;
    Eigen::VectorXd deltaUmin;
    Eigen::VectorXd deltaUmax;
    Eigen::VectorXd Umin;
    Eigen::VectorXd Umax;
    Eigen::VectorXd Ut;
    Eigen::VectorXd deltaUt;

    struct OptimizationData
    {
        Eigen::MatrixXd H;
        Eigen::VectorXd f;
    };
    OptimizationData optData;
    std::vector<double> std_lb;
    std::vector<double> std_ub;
    std::vector<double> std_x;
    Eigen::VectorXd eigen_x;

public:
    C_min_j(int _m, int _Nc, double _epsilon, Eigen::VectorXd &_deltaUt)
        : m(_m),
        Nc(_Nc),
        epsilon(_epsilon)
    {
        std_lb.resize(m * Nc + 1);
        std::copy(deltaUmin.data(), deltaUmin.data() + m * Nc, std_lb.begin());
        std_lb[m * Nc] = - INF;

        std_ub.resize(m * Nc + 1);
        std::copy(deltaUmax.data(), deltaUmax.data() + m * Nc, std_ub.begin());
        std_ub[m * Nc] = INF;

        eigen_x.resize(m * Nc + 1);
        eigen_x << _deltaUt, _epsilon;
        for (int i = 0; i < m * Nc + 1; i++)
        {
            std_x.push_back(eigen_x(i));
        }
    }
    C_min_j(int _m, int _Nc, double _epsilon, Eigen::MatrixXd &_H, Eigen::VectorXd &_f, Eigen::VectorXd &_deltaUmin, Eigen::VectorXd &_deltaUmax, Eigen::VectorXd &_Umin, Eigen::VectorXd &_Umax, Eigen::VectorXd &_Ut, Eigen::VectorXd &_deltaUt)
        : m(_m),
        Nc(_Nc),
        epsilon(_epsilon),
        H(_H),
        f(_f),
        deltaUmin(_deltaUmin),
        deltaUmax(_deltaUmax),
        Umax(_Umax),
        Umin(_Umin),
        Ut(_Ut)
    {
        std_lb.resize(m * Nc + 1);
        std::copy(deltaUmin.data(), deltaUmin.data() + m * Nc, std_lb.begin());
        std_lb[m * Nc] = - INF;

        std_ub.resize(m * Nc + 1);
        std::copy(deltaUmax.data(), deltaUmax.data() + m * Nc, std_ub.begin());
        std_ub[m * Nc] = INF;

        eigen_x.resize(m * Nc + 1);
        eigen_x << _deltaUt, _epsilon;
        for (int i = 0; i < m * Nc + 1; i++)
        {
            std_x.push_back(eigen_x(i));
        }
    }

    static C_min_j& GetOnlyCminj(int _m, int _Nc, double _epsilon, Eigen::MatrixXd &_H, Eigen::VectorXd &_f, Eigen::VectorXd &_deltaUmin, Eigen::VectorXd &_deltaUmax, Eigen::VectorXd &_Umin, Eigen::VectorXd &_Umax, Eigen::VectorXd &_Ut, Eigen::VectorXd &_deltaUt)
    {
        static C_min_j onlyInstance(_m, _Nc, _epsilon, _H, _f, _deltaUmin, _deltaUmax, _Umin, _Umax, _Ut, _deltaUt);
        return onlyInstance;
    }

    C_min_j(const C_min_j& other)
        : m(other.m),
        Nc(other.Nc),
        epsilon(other.epsilon),
        H(other.H),
        f(other.f),
        deltaUmin(other.deltaUmin),
        deltaUmax(other.deltaUmax),
        Umin(other.Umin),
        Umax(other.Umax),
        Ut(other.Ut),
        deltaUt(other.deltaUt),
        optData(other.optData),
        std_lb(other.std_lb),
        std_ub(other.std_ub),
        std_x(other.std_x),
        eigen_x(other.eigen_x)
    {
    }

    C_min_j& operator=(const C_min_j& other)
    {
        if (this != &other) // 检查自赋值
        {
            // 对于这个类，我们没有动态分配的资源，所以我们只需要复制值
            Nc = other.Nc;
            epsilon = other.epsilon;
            H = other.H;
            f = other.f;
            deltaUmin = other.deltaUmin;
            deltaUmax = other.deltaUmax;
            Umin = other.Umin;
            Umax = other.Umax;
            Ut = other.Ut;
            deltaUt = other.deltaUt;
            optData = other.optData;
            std_lb = other.std_lb;
            std_ub = other.std_ub;
            std_x = other.std_x;
            eigen_x = other.eigen_x;
        }
        return *this;
    }

    C_min_j(C_min_j&& other) noexcept
        : m(other.m),
        Nc(other.Nc),
        epsilon(other.epsilon),
        H(std::move(other.H)),
        f(std::move(other.f)),
        deltaUmin(std::move(other.deltaUmin)),
        deltaUmax(std::move(other.deltaUmax)),
        Umin(std::move(other.Umin)),
        Umax(std::move(other.Umax)),
        Ut(std::move(other.Ut)),
        deltaUt(std::move(other.deltaUt)),
        optData(std::move(other.optData)),
        std_lb(std::move(other.std_lb)),
        std_ub(std::move(other.std_ub)),
        std_x(std::move(other.std_x)),
        eigen_x(std::move(other.eigen_x))
    {
    }

    C_min_j& operator=(C_min_j&& other) noexcept
    {
        if (this != &other) // 检查自赋值
        {
            // 对于这个类，直接移动所有成员
            Nc = other.Nc;
            epsilon = other.epsilon;
            H = std::move(other.H);
            f = std::move(other.f);
            deltaUmin = std::move(other.deltaUmin);
            deltaUmax = std::move(other.deltaUmax);
            Umin = std::move(other.Umin);
            Umax = std::move(other.Umax);
            Ut = std::move(other.Ut);
            deltaUt = std::move(other.deltaUt);
            optData = std::move(other.optData);
            std_lb = std::move(other.std_lb);
            std_ub = std::move(other.std_ub);
            std_x = std::move(other.std_x);
            eigen_x = std::move(other.eigen_x);
        }
        return *this;
    }

    ~C_min_j()
    {
    }

    static double utility(unsigned n, const double *x, double *grad, void *data)
    {
        OptimizationData *optData = reinterpret_cast<OptimizationData *>(data);
        Eigen::Map<const Eigen::VectorXd> x_vec(x, n);

        double result = (x_vec.transpose() * optData->H * x_vec).value() + 2 * (optData->f.transpose() * x_vec).value();

        // 计算梯度
        if (grad)
        {
            Eigen::Map<Eigen::VectorXd> gradient(grad, n);
            gradient = optData->H * x_vec + optData->f;
        }

//        std::cout << "Utility value = " << result << std::endl;
        return result;
    }

/*    static double matrix_inconstraint(unsigned n, const double *x, double *grad, void *data)
    {
        double *Ab = static_cast<double *>(data);

        double result = Ab[n];
        for (unsigned index = 0; index < n; ++index)
        {
            result += x[index] * Ab[index];
        }
        return result;
    } */

    std::vector<std::vector<double>> get_A(int _m, int _Nc)
    {
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(_m, _m);
        Eigen::MatrixXd M(_m * _Nc, _m * _Nc);
        for (size_t index_1 = 0; index_1 < _Nc; index_1++)
        {
            for (size_t index_2 = 0; index_2 < _Nc; index_2++)
            {
                M.block(_m * index_1, _m * index_2, _m, _m) = I;
            }
        }
        for (size_t index_1 = 0; index_1 < _m * _Nc; index_1++)
        {
            for (size_t index_2 = 0; index_2 < _m * _Nc; index_2++)
            {
                if (index_1 < index_2)
                {
                    M.block(index_1, index_2, 1, 1)(0, 0) = 0;
                }
            }
        }
        Eigen::MatrixXd eigen_A(2 * _m * _Nc, _m * _Nc + 1);
        Eigen::VectorXd zero = Eigen::VectorXd::Zero(_m * _Nc);
        eigen_A << M, zero, -M, zero;
        std::vector<std::vector<double>> std_A(2 * _m * _Nc, std::vector<double>(_m * _Nc + 1));
        for (int i = 0; i < 2 * _m * _Nc; ++i)
        {
            for (int j = 0; j < _m * _Nc + 1; ++j)
            {
                std_A[i][j] = eigen_A(i, j);
            }
        }
        return std_A;
    }

    std::vector<double> get_b(int _m, int _Nc, Eigen::VectorXd _Umax, Eigen::VectorXd _Umin, Eigen::VectorXd _Ut)
    {
        Eigen::VectorXd eigen_b(2 * _m * _Nc);
        std::vector<double> std_b(2 * _m * _Nc);
        eigen_b << _Umax - _Ut, -_Umin + _Ut;
        Eigen::Map<Eigen::VectorXd>(&std_b[0], 2 * _m * _Nc) = eigen_b;
        return std_b;
    }

    std::vector<std::vector<double>> get_Ab(std::vector<std::vector<double>> _std_A, std::vector<double> _std_b, int _m, int _Nc)
    {
        std::vector<std::vector<double>> std_Ab(2 * _m * _Nc);
        for (int i = 0; i < 2 * _m * _Nc; ++i)
        {
            std_Ab[i] = _std_A[i];
            std_Ab[i].push_back(-_std_b[i]);
        }
        return std_Ab;
    }

    Eigen::VectorXd get_deltaUt(int _m, int _Nc, std::vector<double> _std_x)
    {
        Eigen::VectorXd deltaUt(_m * _Nc);
        for (int i = 0; i < _m * _Nc; ++i)
        {
            deltaUt(i) = _std_x[i];
        }
//        std::cout<< " deltaU: " << deltaUt(1) << " ";
        return deltaUt;
    }

    Eigen::VectorXd calculate_deltaUt_Jmin()
    {
/*        const int m = 2;
        int Nc = _Nc;
        double epsilon = _epsilon;
        Eigen::MatrixXd H = _H;
        Eigen::VectorXd f = _f;
        Eigen::VectorXd deltaUmin = _deltaUmin;
        Eigen::VectorXd deltaUmax = _deltaUmax;
        Eigen::VectorXd Umin = _Umin;
        Eigen::VectorXd Umax = _Umax;
        Eigen::VectorXd Ut = _Ut;

        std::vector<double> std_lb(m * Nc + 1);
        std::copy(deltaUmin.data(), deltaUmin.data() + m * Nc, std_lb.begin());
        std_lb[m * Nc] = - INF;

        std::vector<double> std_ub(m * Nc + 1);
        std::copy(deltaUmax.data(), deltaUmax.data() + m * Nc, std_ub.begin());
        std_ub[m * Nc] = INF;

        Eigen::VectorXd eigen_x(m * Nc + 1);
        eigen_x << _deltaUt, epsilon;
        std::vector<double> std_x;
        for (int i = 0; i < m * Nc + 1; i++)
        {
            std_x.push_back(eigen_x(i));
        } */

        // 将H和f存储在OptimizationData中
        optData = {H, f};

        // 构造A
        auto std_A = get_A(m, Nc);

        // 构造b
        auto std_b = get_b(m, Nc, Umax, Umin, Ut);

        // 合并A和b
        auto std_Ab = get_Ab(std_A, std_b, m, Nc);

        double tol = 1e-8;

        double f_min = INF;

        // 设置优化器
        nlopt::opt opter(nlopt::algorithm::LN_COBYLA, m * Nc + 1);
//        nlopt::opt opter(nlopt::algorithm::LD_SLSQP, m * Nc + 1);


        // 设置上下界
        opter.set_lower_bounds(std_lb);
        opter.set_upper_bounds(std_ub);

        auto utility_lambda = [](unsigned n, const double *x, double *grad, void *data) -> double
        {
            OptimizationData *optData = reinterpret_cast<OptimizationData *>(data);
            Eigen::Map<const Eigen::VectorXd> x_vec(x, n);

            double result = (x_vec.transpose() * optData->H * x_vec).value() + 2 * (optData->f.transpose() * x_vec).value();

//            std::cout << "Utility value = " << result << std::endl;
            return result;
        };
        // 设置目标函数
        opter.set_min_objective(utility_lambda, &optData);

        auto matrix_inconstraint_lambda = [](unsigned n, const double *x, double *grad, void *data) -> double
        {
            double *Ab = static_cast<double *>(data);

            double result = Ab[n];
            for (unsigned index = 0; index < n; ++index)
            {
                result += x[index] * Ab[index];
            }
            return result;
        };
        // 添加矩阵形式的不等式约束
        for (int i = 0; i < 2 * m * Nc; i++)
        {
            opter.add_inequality_constraint(matrix_inconstraint_lambda, &std_Ab[i][0], tol);
        }

        // 设置停止条件
        opter.set_xtol_rel(tol);
        opter.set_ftol_abs(tol);

        opter.set_force_stop(tol);
//        opter.set_maxtime(0.5);
//        opter.set_maxeval();


        // 优化
        nlopt::result result = opter.optimize(std_x, f_min);

//        if (result)
//        {
//            std::cout << "Minimum utility = " << f_min << ", std_x = (";
//            for (int i = 0; i < m * Nc + 1; i++)
//            {
//                std::cout << std_x[i];
//                if (i != m * Nc + 1 - 1)
//                {
//                    std::cout << ", ";
//                }
//            }
//            std::cout << ")" << std::endl;
//        }
        if(result)
        {
            std::cout << " Minimum utility = " << f_min << ", ";
        }

        auto deltaUt = get_deltaUt(m, Nc, std_x);

        return deltaUt;
    }
};
#endif
