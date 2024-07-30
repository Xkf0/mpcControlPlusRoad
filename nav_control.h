#pragma once
#include <robos/navi_msgs.h>
#include <queue>
#include <eigen3/Eigen/Dense>

#ifdef WIN32
#ifdef NAV_MPC_LIBRARY
#define NAV_MPC __declspec(dllexport)
#else
#define NAV_MPC __declspec(dllimport)
#endif

#else
#if defined(NAV_MPC_LIBRARY)
#  define NAV_MPC __attribute ((visibility("default")))
#else
#  define NAV_MPC
#endif

#endif

namespace navigation
{
	struct CtrlOutput
	{
		double angularV = 0.0;//角速度，瞬心在左边为负，在右边为正
		double lineV = 0.0;//线速度，向前走为正，向后走为负
		double angle = 0.0;//转角，弧度制，左边为负值，右边为正值
		int reduce_model = 0;   //减速模型：0:正常行走、1:快速减速、2:直接停止	
		int move_model = 0;	//是否横向移动, 0不改变轮子状态 1  为左横移  2为右横移 3 直接行走
	};
}
class C_MPC_CONTROLROAD;
class C_MPC_ROAD;
class NavControl
{
public:
    NavControl();
    ~NavControl();
    void setPath(std::vector<robos::Point2d>& path);
    bool getCtrlResult(robos::Pose2D curpose, navigation::CtrlOutput& ctr_output, const int isMain);

private:
    void distancesAdd(double distance);
    void deltaPhisAdd(double distance);
    void distancesAdd30(double distance);
    void deltaPhisAdd30(double distance);
    void direction5Add(double distance);
    void direction30Add(double distance);
    std::vector<robos::Point2d> getCirclePoint(const std::vector<robos::Point2d>& path, int nearst_index, int interval);
    bool initMPC();
    void setNavPathList(const robos::Pose2D& curpose, const std::vector<robos::Point2d> path);
    bool navRun(const robos::Pose2D& robot_pose, const robos::Pose2D& robot_pose_Ref, navigation::CtrlOutput& ctr_output, int m, double deltaRef, const double l, double cur_distance, const int isMain);
    void setTargetSpeed(double speed);
    //获取最近的点的下标
    int getNearstPathPointIndex(const robos::Pose2D& robot_pose, std::vector<robos::Point2d> path);

	//获取过零点的下标
    bool getZeroPathPointIndex(const robos::Pose2D& robot_pose, const std::vector<robos::Point2d>& path, int& index);
    void createMPC();

private:
    C_MPC_CONTROLROAD *c_mpc;
    std::vector<robos::Pose2D> pathsWithAngle;
    //当前最近点
    robos::Pose2D curposeRef;
    //当前最近点，即期望值
    int nearst_index;
    //用于求出rRef的三个点，三点定圆
    std::vector<robos::Point2d> circlePoint;
    //期望偏转角
    double deltaRef;
    std::vector<robos::Point2d> pathPoints;

    int n;
    int m;
    int Nc;
    int Np;
    double l;
    double T;
    double k;
    double qWeightStateOffset;
    double rWeightControlIncrement;
    double epsilonRelaxationFactor;
    double rhoWeightCoefficient;
    bool getMpcValue;
    std::vector<robos::Point2d> paths;
    double vMax;
    double deltaMax;
    double deltaDeltaMax;
    std::queue<double> distances;
    std::queue<double> deltaPhis;
    size_t maxSize = 5; // 队列的最大长度
    std::queue<double> distances30;
    std::queue<double> deltaPhis30;
    size_t maxSize30 = 30; // 队列的最大长度
    std::queue<double> direction5queue;
    std::queue<double> direction30queue;
    double direction5;
    double direction30;

    //for cout
    int count = 1;
    double phi0 = 0;
    double angularVchange = 0;
    double lineA = 0;

    double target_speed_;
};

class NavRoad
{
public:
    NavRoad();
    ~NavRoad();
    void setPath(const std::vector<robos::Point2d>& refer_path);
    std::vector<robos::Point2d> getRoad(Eigen::VectorXd sideOb, robos::Pose2D curpose, double v);
private:
    std::vector<robos::Point2d> convertToPoints(const Eigen::VectorXd& vec);
private:
    C_MPC_ROAD *c_mpc;
    std::vector<robos::Pose2D> pathsWithAngleRef;
    Eigen::VectorXd sideOb;
    int N;
};
