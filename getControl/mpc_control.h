#ifndef MPC_CONTROL_H
#define MPC_CONTROL_H
#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include "min_j.hpp"
#include <thread>
#include <future>
#include <chrono>
#include <fstream>
#include <iostream>

class C_MPC_CONTROLROAD
{
private:
    // 输入
    int i_n;
    int i_m;
    int i_Np;
    int i_Nc;
    double d_k;
    double d_T;
    double d_qWeightStateOffset;
    double d_rWeightControlIncrement;
    double d_epsilonRelaxationFactor;
    double d_rhoWeightCoefficient;
    double d_l;
    Eigen::VectorXd v_deltaUmin;
    Eigen::VectorXd v_deltaUmax;
    Eigen::VectorXd v_Umin;
    Eigen::VectorXd v_Umax;
    // p_u[0] = v1, p_u[1] = delta1, p_u[2] = v2, p_u[3] = delta2, ...
    double *p_u;
    // p_chi[0] = d1, p_chi[1] = theta1, p_chi[2] = d2, p_chi[3] = theta2, ...
    double *p_chi;
    // p_uRef[0] = vRef1, p_uRef[1] = deltaRef1, p_uRef[2] = vRef2, p_uRef[3] = deltaRef2, ...
    double *p_uRef;
    // p_chiRef[0] = dRef1, p_chiRef[1] = thetaRef1, p_chiRef[2] = dRef2, p_chiRef[3] = thetaRef2, ...
    double *p_chiRef;

    // 需要输出的
    Eigen::VectorXd v_deltaUt;
    Eigen::VectorXd v_Ut;
    double *p_Ut;

    // 中间变量
    Eigen::MatrixXd m_QWeightStateOffset;
    Eigen::MatrixXd m_RWeightControlIncrement;
    Eigen::MatrixXd m_C;
    Eigen::MatrixXd m_A1;
    Eigen::MatrixXd m_B1;
    Eigen::MatrixXd m_A2;
    Eigen::MatrixXd m_B2;
    Eigen::MatrixXd m_EDeviation;
    Eigen::MatrixXd m_psi;
    Eigen::MatrixXd m_theta;
    Eigen::Matrix<double, 4, 1> m_xiCurrentState;

    // 需要输入到min_j.h的中间变量
    Eigen::MatrixXd m_H;
    Eigen::VectorXd v_f;

    // 方法中用到
    Eigen::MatrixXd m_I; // 2nx2n的的单位矩阵

private:
    Eigen::MatrixXd get_zero(int _i_n);
    Eigen::MatrixXd Matrix_power(Eigen::MatrixXd &_M, int _c, int _n);
    Eigen::Matrix<double, 2, 2> get_A(int _index, double *_p_uRef, double *_p_chiRef);
    Eigen::Matrix<double, 2, 2> get_B(int _index, double *_p_uRef, double *_p_chiRef, double _d_l, double _d_k);
    Eigen::MatrixXd get_psi(Eigen::MatrixXd &_m_A2, int _i_Np);
    Eigen::MatrixXd get_theta(Eigen::MatrixXd &_m_A2, Eigen::MatrixXd &_m_B2, int _i_Nc, int _i_Np);
    Eigen::MatrixXd get_xi(double *_p_u, double *_p_chi, double *_p_uRef, double *_p_chiRef);
    void calculate_Q_R();
    void calculate_C();
    void calculate_A1_B1();
    void calculate_A2_B2();
    void calculate_psi_theta_xi();
    void calculate_E();
    void calculate_H();
    void calculate_f();
    bool update_delta_Ut(Eigen::VectorXd _v_deltaUt);
    bool updata_Ut();
    void calculate_process();

//    Eigen::MatrixXd get_psi0(Eigen::Matrix<double, 5, 5> &_m_A2, int _i_Np);
//    Eigen::MatrixXd get_theta0(Eigen::Matrix<double, 5, 5> &_m_A2, Eigen::Matrix<double, 5, 2> &_m_B2, int _i_Nc, int _i_Np);
//    void calculate_Q_R0();
//    Eigen::MatrixXd get_zero0(int _i_n);

public:
    C_MPC_CONTROLROAD(int _i_n, int _i_m, double _i_Nc);
    C_MPC_CONTROLROAD(int _i_n, int _i_m, int _i_Np, int _i_Nc, double d_k, double _d_T, double _d_qWeightStateOffset, double _d_rWeightControlIncrement, double _d_epsilonRelaxationFactor, double _d_rhoWeightCoefficient, double _d_l,
          Eigen::VectorXd _v_deltaUmin, Eigen::VectorXd _v_deltaUmax, Eigen::VectorXd _v_Umin, Eigen::VectorXd _v_Umax, double *_p_u, double *_p_chi, double *_p_uRef, double *_p_chiRef);
    C_MPC_CONTROLROAD(const C_MPC_CONTROLROAD& other);
    C_MPC_CONTROLROAD& operator=(const C_MPC_CONTROLROAD& other);
    C_MPC_CONTROLROAD(C_MPC_CONTROLROAD&& other) noexcept;
    C_MPC_CONTROLROAD& operator=(C_MPC_CONTROLROAD&& other) noexcept;
    ~C_MPC_CONTROLROAD();

    bool set_n(int _i_n);
    bool set_m(int _i_m);
    bool set_Nc_Np(int _i_Nc, int _i_Np);
    bool set_k(double _d_k);
    bool set_T(double _d_T);
    bool set_q_r(double _d_q_weightStateOffset, double _d_r_weightControlIncrement);
    bool set_epsilon_rho(double _d_epsilonRelaxationFactor, double _d_rhoWeightCoefficient);
    bool set_l(double _d_l);
    bool set_deltaUmin_deltaUmax_Umin_Umax(Eigen::VectorXd _v_deltaUmin, Eigen::VectorXd _v_deltaUmax, Eigen::VectorXd _v_Umin, Eigen::VectorXd _v_Umax);
    bool set_u(double *_p_u);
    bool set_uRef(double *_p_uRef);
    bool set_chi(double *_p_chi);
    bool set_chiRef(double *_p_chiRef);

    bool get_control_best();
    Eigen::VectorXd get_Ut();
    Eigen::VectorXd get_deltaUt();

    static C_MPC_CONTROLROAD& GetOnlyCmpc(int _i_n, int _i_m, int _i_Np, int _i_Nc, double d_k, double _d_T, double _d_qWeightStateOffset, double _d_rWeightControlIncrement, double _d_epsilonRelaxationFactor, double _d_rhoWeightCoefficient, double _d_l,
                              Eigen::VectorXd _v_deltaUmin, Eigen::VectorXd _v_deltaUmax, Eigen::VectorXd _v_Umin, Eigen::VectorXd _v_Umax, double *_p_u, double *_p_chi, double *_p_uRef, double *_p_chiRef);
};


#endif // MPC_CONTROL_H
