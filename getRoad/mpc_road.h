#ifndef MPC_ROAD_H
#define MPC_ROAD_H
#define PLOT_DIS(x1, y1, x2, y2) std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) // 两点的距离
#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <nlopt.hpp>
#include <inf.h>
#include <robos/navi_msgs.h>

class C_MPC_ROAD
{
private:
    struct OptimizationData
    {
        int i_Nc;
        int i_Np;
        double d_T;

        // 状态量初始值
        Eigen::VectorXd v_xi0;

        Eigen::MatrixXd m_Q;
        // 输出量的参考值，长度为2Np，是phiRef,YRef组成的
        Eigen::VectorXd v_etaRef;

        Eigen::MatrixXd m_R;
        Eigen::Matrix<double, 2, 5> m_C;
        Eigen::MatrixXd m_CExpanded;
        Eigen::Matrix<double, 2, 5> m_C_XY;
        Eigen::MatrixXd m_C_XYExpanded;

        // 障碍物的位置，它的长度就是障碍物的个数的两倍，它的内容是Xob,Yob一直排列下去
        Eigen::VectorXd v_sideOb;
        // 用于防止分母为零
        double d_zeta;
        double d_Sob;
        // 障碍物的个数
        int i_N;
    };

    struct S_xi_eta
    {
        Eigen::VectorXd v_xi;
        Eigen::VectorXd v_eta;
    };

    struct S_poseNow
    {
        double d_x0;
        double d_y0;
        double d_theta0;
        double d_v0;
    };

    double d_q;
    double d_r;

    Eigen::VectorXd v_aymin;
    Eigen::VectorXd v_aymax;

    std::vector<double> vd_x;

    std::vector<robos::Pose2D> pathsWithAngleRef;
    int i_nearstIndex;
    // 滑窗坐标系方向
    double d_thetaAvg;
    // 滑窗坐标系取点个数
    int i_num;

    OptimizationData optData;
    S_poseNow poseNow;

private:
    Eigen::VectorXd calculate_coordinateConversionToWorld(double d_x, double d_y, double d_alpha, double d_dx, double d_dy);
    Eigen::VectorXd calculate_coordinateConversionFromWorld(double d_x, double d_y, double d_alpha, double d_dx, double d_dy);
    Eigen::MatrixXd calculate_Q(int i_Np, double d_q);
    Eigen::MatrixXd calculate_R(int i_Nc, double d_r);
    Eigen::Matrix<double, 2, 5> calculate_C();
    Eigen::Matrix<double, 2, 5> calculate_C_XY();

    void calculate_coordinateDirection();
    void calculate_Q_R_C();
    void calculate_xi0();
    void calculate_etaRef();
    void calculate_sideOb();
    void calculate_nearstIndex();

    static double calculate_xdotdot();
    static double calculate_ydotdot(Eigen::VectorXd v_ay_Np, int i);
    static double calculate_phidot(Eigen::VectorXd v_ay_Np, Eigen::VectorXd v_xi, int i);
    static double calculate_Xdot(Eigen::VectorXd v_xi, int i);
    static double calculate_Ydot(Eigen::VectorXd v_xi, int i);
    static Eigen::VectorXd calculate_xi(Eigen::VectorXd v_ay_Np, Eigen::VectorXd v_xi0, double d_T, int i_Np);
    static Eigen::VectorXd calculate_ay(Eigen::VectorXd v_ay_Nc, int i_Np, int i_Nc);
    static S_xi_eta calculate_eta(Eigen::VectorXd v_ay_Nc, Eigen::VectorXd v_xi0, double d_T, int i_Np, int i_Nc, Eigen::MatrixXd m_CExpanded);
    static Eigen::VectorXd calculate_u(Eigen::VectorXd v_ay_Nc);
    static double calculate_Job(Eigen::VectorXd v_xi, int i_Np, int i_N, double d_Sob, double d_zeta, Eigen::VectorXd v_sideOb);
    static double utility(unsigned n, const double *x, double *grad, void *data);

    Eigen::VectorXd filterElements(const Eigen::VectorXd &original, int i_N);

public:
    C_MPC_ROAD(double _d_q, double _d_r, Eigen::VectorXd _v_aymin, Eigen::VectorXd _v_aymax, int _i_Nc, int _i_Np, double _d_T, double _d_zeta, double _d_Sob, int _i_num);
    C_MPC_ROAD(const C_MPC_ROAD &other);
    C_MPC_ROAD &operator=(const C_MPC_ROAD &other);
    C_MPC_ROAD(C_MPC_ROAD &&other) noexcept;
    C_MPC_ROAD &operator=(C_MPC_ROAD &&other) noexcept;
    ~C_MPC_ROAD();

    // 初始化时set
    bool set_q_r(double _d_q, double _d_r);
    bool set_aymin_aymax(Eigen::VectorXd _v_aymin, Eigen::VectorXd _v_aymax);
    bool set_Nc_Np(int _i_Nc, int _i_Np);
    bool set_T(double _d_T);
    bool set_pathWithAngleRef(std::vector<robos::Pose2D> _pathsWithAngleRef);
    bool set_zeta_Sob(double _d_zeta, double _d_Sob);
    bool set_N(int _i_N);
    bool set_num(int _i_num);

    // 一边运行一边set
    bool set_poseNow(double d_x0, double d_y0, double d_theta0, double d_v0);
    bool set_sideOb(Eigen::VectorXd _v_sideOb);

    Eigen::VectorXd get_road_best();

    static C_MPC_ROAD &GetOnlyCmpc(double _d_q, double _d_r, Eigen::VectorXd _v_aymin, Eigen::VectorXd _v_aymax, int _i_Nc, int _i_Np, double _d_T, Eigen::VectorXd _v_xi0,
                                   Eigen::VectorXd _v_etaRef, Eigen::VectorXd _v_sideOb, double _d_zeta, double _d_Sob, int _i_N);
};

#endif // MPC_ROAD_H
