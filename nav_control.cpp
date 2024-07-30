#include <algorithm>
#include <thread>
#include <future>
#include <chrono>
#include <fstream>
#include <iostream>
#include "nav_control.h"
#include "math_formula.hpp"
#include "getControl/mpc_control.h"
#include "getRoad/mpc_road.h"
#include "params/params_option.hpp"
#define MAX_DIS_ERROR_REFP 2

void NavControl::distancesAdd(double distance)
{
    // 当添加新元素时，首先移除最早加入的元素
    distances.pop();
    distances.push(distance);
}

void NavControl::deltaPhisAdd(double distance)
{
    // 当添加新元素时，首先移除最早加入的元素
    deltaPhis.pop();
    deltaPhis.push(distance);
}

void NavControl::distancesAdd30(double distance)
{
    // 当添加新元素时，首先移除最早加入的元素
    distances30.pop();
    distances30.push(distance);
}

void NavControl::deltaPhisAdd30(double distance)
{
    // 当添加新元素时，首先移除最早加入的元素
    deltaPhis30.pop();
    deltaPhis30.push(distance);
}

void NavControl::direction5Add(double distance)
{
    // 当添加新元素时，首先移除最早加入的元素
    direction5queue.pop();
    direction5queue.push(distance);
}

void NavControl::direction30Add(double distance)
{
    // 当添加新元素时，首先移除最早加入的元素
    direction30queue.pop();
    direction30queue.push(distance);
}

bool NavControl::getCtrlResult(robos::Pose2D curpose, navigation::CtrlOutput& ctr_output, const int isMain)
{
    std::this_thread::sleep_for(std::chrono::microseconds(100));

    // 记录开始时间
    auto start = std::chrono::high_resolution_clock::now();

//    nearst_index = getNearstPathPointIndex(curpose, paths);
//    int interval = 1;
//    double weightSum = 0.0;
//    double _deltaRef;
//    double deltaRefSum = 0.0;
//    double weight = 1.0;
////    while(weight > 0)
////    {
////        if(nearst_index + 3 * interval > paths.size())
////        {
////            break;
////        }
////        circlePoint = getCirclePoint(paths, nearst_index, interval);
////        _deltaRef = getDelta(l, circlePoint);
////        deltaRefSum += _deltaRef * weight;
////        weightSum += weight;
////        interval++;
////        weight -= 0.1;
////    }
//    interval = 1;
//    weight = 1.0;
////    while(weight > 0)
////    {
////        if(nearst_index - 3 * interval + 3 < 0)
////        {
////            break;
////        }
////        circlePoint = getCirclePoint(paths, nearst_index - 3 * interval + 3, interval);
////        _deltaRef = getDelta(l, circlePoint);
////        deltaRefSum += _deltaRef * weight;
////        weightSum += weight;
////        interval++;
////        weight -= 0.1;
////    }
//    int _start_index = nearst_index;
//    interval = 1;
//    weight = 1.0;
//    while(weight > 0)
//    {
//        if(_start_index + 2 >= paths.size())
//        {
//            break;
//        }
//        circlePoint = getCirclePoint(paths, _start_index, interval);
//        _deltaRef = getDelta(l, circlePoint);
//        deltaRefSum += _deltaRef * weight;
//        weightSum += weight;
//        _start_index += 3;
//        weight -= 0.3;
//    }
//    _start_index = nearst_index;
//    weight = 1.0;
////    while(weight > 0)
////    {
////        if(_start_index < 0)
////        {
////            break;
////        }
////        circlePoint = getCirclePoint(paths, _start_index, interval);
////        _deltaRef = getDelta(l, circlePoint);
////        deltaRefSum += _deltaRef * weight;
////        weightSum += weight;
////        _start_index--;
////        weight -= 0.1;
////    }
//    deltaRef = deltaRefSum / weightSum;

    nearst_index = getNearstPathPointIndex(curpose, paths);

    double cur_distance = getd(curpose, nearst_index, paths);

    int interval = 1;
    circlePoint = getCirclePoint(paths, nearst_index, interval);
    deltaRef = getDelta(l, circlePoint);
    if(deltaRef > deltaMax)
    {
        deltaRef = deltaMax;
    }
    else if(deltaRef < -deltaMax)
    {
        deltaRef = -deltaMax;
    }
    double radius = getCircleRadius(circlePoint);
    int direction = getDeltaDirection(circlePoint);
    if(radius != 0)
    {
        k = - direction / radius;
    }
    else
    {
        k = 0.0;
    }
//    std::cout << " radius: " << radius << " ";
//    std::cout << " k: " << k << " ";


    curposeRef = pathsWithAngle[nearst_index];

    //for cout
    phi0 = NormalizedAngle(curpose.theta * 180.0 / M_PI, AngleType::SIGNDEGREE);

    std::cout << "pose[" << count << "]: " << "x: " << curpose.x << ", y: " << curpose.y << ", phi: " << phi0 << std::endl;
    std::cout << "                                                        xRef: " << curposeRef.x << ", yRef: " << curposeRef.y;

    std::ofstream logFile("cout.txt", std::ios::app);

    // 确保文件成功打开
    if (!logFile.is_open())
    {
        std::cerr << "无法打开文件" << std::endl;
    }

    // 在程序的其他部分，直接使用logFile对象写入，不需要每次检查是否打开
    logFile << std::endl << "pose[" << count << "]: " << "x: " << curpose.x << ", y: " << curpose.y << ", phi: " << phi0 << std::endl;
    logFile << "                                                        xRef: " << curposeRef.x << ", yRef: " << curposeRef.y;

    if(getMpcValue)
    {
        getMpcValue = navRun(curpose, curposeRef, ctr_output, m, deltaRef, l, cur_distance, isMain);
        count++;
    }
    if(getMpcValue == false)
    {
        return 0;
    }

    //for cout
    angularVchange = c_mpc->get_deltaUt()(1) * 180.0 / (M_PI * T);
    if((angularVchange < 1e-10) && (angularVchange > -1e-10))
    {
        angularVchange = 0.0;
    }
    lineA = c_mpc->get_deltaUt()(0) / T;
    if((lineA < 1e-10) && (lineA > -1e-10))
    {
        lineA = 0.0;
    }

    // 注：这里的lineA是当前时刻相对于上一时刻的加速度，不是当前时刻瞬间的加速度
    std::cout<< "                  control[" << count - 1 << "--" << count << "]: " << "lineV: " << ctr_output.lineV << ", delta: " << (ctr_output.angle * 180.0 / M_PI)
             << ", angularVcar: " << (ctr_output.angularV * 180.0 / M_PI) << " , lineA: " << lineA << " , angularVchange(per second): " << angularVchange;
    //        std::cout << "pose[" << count << "]: " << "x: " << curpose.x << ", y: " << curpose.y << ", phi: " << (curpose.theta * 180.0 / M_PI) << std::endl;

    logFile << "                  control[" << count - 1 << "--" << count << "]: " << "lineV: " << ctr_output.lineV << ", delta: " << (ctr_output.angle * 180.0 / M_PI)
            << ", angularVcar: " << (ctr_output.angularV * 180.0 / M_PI) << " , lineA: " << lineA << " , angularVchange(per second): " << angularVchange;
    logFile.close();

    // 记录结束时间
    auto end = std::chrono::high_resolution_clock::now();

    // 计算时间差，转换为秒，并转换为double类型
    double duration = std::chrono::duration<double, std::milli>(end - start).count();

    // 打开文件以追加模式
    std::ofstream file("output.txt", std::ios::app);
    if (file.is_open())
    {
        // 将时间差写入文件，精确到小数点后三位
        file << std::fixed << std::setprecision(3) << "Function Execution Time: " << duration << " milliseconds" << std::endl;

        // 关闭文件
        file.close();
    }
    else
    {
        std::cerr << "无法打开文件" << std::endl;
    }
    std::cout << ", Function Execution Time: " << duration << " milliseconds" << std::endl;
    logFile << ", Function Execution Time: " << duration << " milliseconds" << std::endl;

    return 1;
}

void NavControl::createMPC()
{
    if(c_mpc != nullptr)
    {
        delete c_mpc;
    }
    c_mpc = new C_MPC_CONTROLROAD(n, m, Nc);
}

NavControl::NavControl()
{
    NavControlParams navControlParams;
    std::string file_path = "./nav_params";
    navControlParams = CreateControlTaskDecOptions(file_path);

    n = navControlParams.n;
    m = navControlParams.m;
    Nc = navControlParams.Nc;
    Np = navControlParams.Np;
    k = 0.0;
    l = navControlParams.l;
    T = navControlParams.T;
    qWeightStateOffset = navControlParams.qWeightStateOffset;
    rWeightControlIncrement = navControlParams.rWeightControlIncrement;
    epsilonRelaxationFactor = navControlParams.epsilonRelaxationFactor;
    rhoWeightCoefficient = navControlParams.rhoWeightCoefficient;
    vMax = navControlParams.vMax;
    deltaMax = navControlParams.deltaMax;
    deltaDeltaMax = navControlParams.deltaDeltaMax;
    c_mpc = nullptr;
    for(size_t i = 0; i < maxSize; ++i)
    {
        distances.push(1.0); // 将0.0添加到队列中
    }
    for(size_t i = 0; i < maxSize; ++i)
    {
        deltaPhis.push(1.0); // 将0.0添加到队列中
    }
    for(size_t i = 0; i < maxSize30; ++i)
    {
        distances30.push(1.0); // 将0.0添加到队列中
    }
    for(size_t i = 0; i < maxSize30; ++i)
    {
        deltaPhis30.push(1.0); // 将0.0添加到队列中
    }
    for(size_t i = 0; i < maxSize; ++i)
    {
        direction5queue.push(0.0); // 将0.0添加到队列中
    }
    for(size_t i = 0; i < maxSize30; ++i)
    {
        direction30queue.push(0.0); // 将0.0添加到队列中
    }

    createMPC();
    getMpcValue = initMPC();
}

NavControl::~NavControl()
{
    delete c_mpc;
}

std::vector<robos::Point2d> NavControl::getCirclePoint(const std::vector<robos::Point2d>& path, int nearst_index, int interval)
{
    std::vector<robos::Point2d> circlePoint;
    for(int i = 0; i < 3 * interval; i++)
    {
        circlePoint.push_back(path[nearst_index + i * interval]);
    }
    return circlePoint;
}

bool NavControl::initMPC()
{
    Eigen::VectorXd v_deltaUmin = - Eigen::VectorXd::Ones(m * Nc);
    Eigen::VectorXd v_deltaUmax = Eigen::VectorXd::Ones(m * Nc);
    Eigen::VectorXd v_Umin = - Eigen::VectorXd::Ones(m * Nc);
    Eigen::VectorXd v_Umax = Eigen::VectorXd::Ones(m * Nc);
    for(int index = 1; index < Nc * m * n; index += m)
    {
        v_Umin(index - 1) = 0.99 * vMax;
        v_Umax(index - 1) = vMax;
        v_deltaUmin(index - 1) = - vMax;
        v_deltaUmax(index - 1) = vMax;

        v_deltaUmin(index) = - deltaDeltaMax;
        v_deltaUmax(index) = deltaDeltaMax;
        v_Umin(index) = - deltaMax;
        v_Umax(index) = deltaMax;
    }

    bool _set_Nc_Np = c_mpc->set_Nc_Np(Nc, Np);
    bool _set_k = c_mpc->set_k(0.0);
    bool _set_T = c_mpc->set_T(T);
    bool _set_q_r = c_mpc->set_q_r(qWeightStateOffset, rWeightControlIncrement);
    bool _set_epsilon_rho = c_mpc->set_epsilon_rho(epsilonRelaxationFactor, rhoWeightCoefficient);
    bool _set_l = c_mpc->set_l(l);
    bool _set_deltaUmin_deltaUmax_Umin_Umax = c_mpc->set_deltaUmin_deltaUmax_Umin_Umax(v_deltaUmin, v_deltaUmax, v_Umin, v_Umax);

    return _set_k && _set_Nc_Np && _set_T && _set_q_r && _set_epsilon_rho && _set_l && _set_deltaUmin_deltaUmax_Umin_Umax;
}

void NavControl::setPath(std::vector<robos::Point2d>& refer_path)
{
    paths = refer_path;
    int path_size = refer_path.size();
    pathsWithAngle.resize(path_size);
    for (int i = 0; i < path_size; ++i)
    {
        double angle = (i == 0) ? Get2PointSlop(refer_path[0], refer_path[1])
                                : Get2PointSlop(refer_path[i - 1], refer_path[i]);
        angle = NormalizedAngle(angle, AngleType::RADIAN);
        pathsWithAngle[i] = robos::Pose2D(refer_path[i].x, refer_path[i].y, angle);
    }
    pathPoints = getPathPoints(paths, pathsWithAngle);
}

void NavControl::setNavPathList(const robos::Pose2D& curpose, const std::vector<robos::Point2d> path)
{
}

bool NavControl::navRun(const robos::Pose2D& robot_pose, const robos::Pose2D& robot_pose_Ref, navigation::CtrlOutput& ctr_output, int m, double deltaRef, const double l, double cur_distance, const int isMain)
{
    double p_u[m];
    p_u[0] = ctr_output.lineV;
    p_u[1] = ctr_output.angle;
    double p_chi[2];
    p_chi[0] = cur_distance;
    p_chi[1] = robot_pose_Ref.theta - robot_pose.theta;
    double p_uRef[m];
    if(isMain)
    {
        p_uRef[0] = getVRef(pathPoints, robot_pose, vMax, pathsWithAngle, nearst_index);
    }
    else
    {
        p_uRef[0] = getVRefWithoutCount(pathPoints, robot_pose, vMax, pathsWithAngle, nearst_index);
    }

//    p_uRef[0] = vMax;
    if(p_uRef[0] == 0)
    {
        return false;
    }
    std::cout << "vRef: " << p_uRef[0];

    std::ofstream logFile("cout.txt", std::ios::app);

    logFile << "vRef: " << p_uRef[0];

    double d_euclideanDistance = getEuclideanDistance(robot_pose, robot_pose_Ref);
    double d_deltaMin = getDeltaMin(deltaRef, deltaMax, d_euclideanDistance, distances, deltaPhis, direction5, direction30, distances30, deltaPhis30);
    double d_deltaMax = getDeltaMax(deltaRef, deltaMax, d_euclideanDistance, distances, deltaPhis, direction5, direction30, distances30, deltaPhis30);
    if(d_deltaMin == 100.0 && d_deltaMax < 99.0)
    {
        d_deltaMin = d_deltaMax - 0.00001;
    }
    if(d_deltaMax == 100.0 && d_deltaMin < 99.0)
    {
        d_deltaMax = d_deltaMin + 0.00001;
    }

    Eigen::VectorXd v_deltaUmin = - Eigen::VectorXd::Ones(m * Nc);
    Eigen::VectorXd v_deltaUmax = Eigen::VectorXd::Ones(m * Nc);
    Eigen::VectorXd v_Umin = - Eigen::VectorXd::Ones(m * Nc);
    Eigen::VectorXd v_Umax = Eigen::VectorXd::Ones(m * Nc);
    for(int index = 1; index < Nc * m; index += m)
    {
        v_Umin(index - 1) = 0.99 * p_uRef[0];
        v_Umax(index - 1) = p_uRef[0];
        v_deltaUmin(index - 1) = - p_uRef[0];
        v_deltaUmax(index - 1) = p_uRef[0];

        v_deltaUmin(index) = - deltaDeltaMax;
        v_deltaUmax(index) = deltaDeltaMax;

        v_Umin(index) = d_deltaMin;
        v_Umax(index) = d_deltaMax;
//        v_Umin(index) = - deltaMax;
//        v_Umax(index) = deltaMax;

//        v_Umin(index) = (paths.size() > 100) ? d_deltaMin : (- deltaMax);
//        v_Umax(index) = (paths.size() > 100) ? d_deltaMax : deltaMax;
    }
    bool _set_deltaUmin_deltaUmax_Umin_Umax = c_mpc->set_deltaUmin_deltaUmax_Umin_Umax(v_deltaUmin, v_deltaUmax, v_Umin, v_Umax);

    if(!_set_deltaUmin_deltaUmax_Umin_Umax)
    {
        return false;
    }

    p_uRef[1] = deltaRef;
    std::cout << ", deltaRefMax: " << d_deltaMax * 180 / M_PI << "  ";
    std::cout << ", deltaRefMin: " << d_deltaMin * 180 / M_PI << "  ";
    logFile << ", deltaRefMax: " << d_deltaMax * 180 / M_PI << "  ";
    logFile << ", deltaRefMin: " << d_deltaMin * 180 / M_PI << "  ";
    std::cout << ", deltaRef(unsigned): " << - deltaRef * 180 / M_PI << "  ";
    logFile << ", deltaRef(unsigned): " << - deltaRef * 180 / M_PI << "  ";
    double p_chiRef[2];
    p_chiRef[0] = 0.0;
    p_chiRef[1] = 0.0;
    std::cout << ", cur_distance: " << cur_distance << "";
    logFile << ", cur_distance: " << cur_distance << "";
    distancesAdd(cur_distance);
    distancesAdd30(cur_distance);
    double d_deltaPhi = robot_pose_Ref.theta - robot_pose.theta;
    while (d_deltaPhi > M_PI)
    {
        d_deltaPhi -= 2 * M_PI;
    }
    while (d_deltaPhi <= -M_PI)
    {
        d_deltaPhi += 2 * M_PI;
    }
    d_deltaPhi = fabs(d_deltaPhi);

    std::cout << ", theta_error: " << d_deltaPhi << ",";
    logFile << ", theta_error: " << d_deltaPhi << ",";
    deltaPhisAdd(d_deltaPhi);
    deltaPhisAdd30(d_deltaPhi);


    bool _set_k = c_mpc->set_k(k);
    bool _set_u = c_mpc->set_u(p_u);
    bool _set_uRef = c_mpc->set_uRef(p_uRef);
    bool _set_chi = c_mpc->set_chi(p_chi);
    bool _set_chiRef = c_mpc->set_chiRef(p_chiRef);
    bool _get_control_best;

    Eigen::VectorXd v_Ut;

    if(_set_k && _set_u && _set_uRef && _set_chi && _set_chiRef)
    {
//        std::future<bool> future = std::async(std::launch::async, [&]{
//            return c_mpc.get_control_best();
//        });

//        std::chrono::microseconds span(1000);
//        if(future.wait_for(span) == std::future_status::timeout)
//        {
//            v_Ut = Eigen::VectorXd::Zero(m * 5);
//            std::cout << "Error: getUt";
//        }
//        else
//        {
//            v_Ut = c_mpc.get_Ut();
//        }
        _get_control_best = c_mpc->get_control_best();
        v_Ut = c_mpc->get_Ut();
    }

    if(_get_control_best)
    {
        ctr_output.lineV = v_Ut(0);
        ctr_output.angle = - v_Ut(1);

//        ctr_output.angle = 0.01745; // 右转一度
//        ctr_output.angle = -0.01745; // 左转一度

//        ctr_output.angle = 0.01745 * 3; // 右转3度
//        ctr_output.angle = -0.01745 * 3; // 左转3度

//        ctr_output.angle = 0.01745 * 5; // 右转5度
//        ctr_output.angle = -0.01745 * 5; // 左转5度

//        ctr_output.angle = 0.0; // 直行

        ctr_output.angularV = getAngularV(ctr_output.lineV, -ctr_output.angle, l);
        direction5Add(v_Ut(1) - deltaRef);
        direction30Add(v_Ut(1) - deltaRef);
        direction5 = sumOfQueue(direction5queue);
        direction30 = sumOfQueue(direction30queue);
        return true;
    }

    return false;
}

void NavControl::setTargetSpeed(double speed)
{
	target_speed_ = speed;
}

int NavControl::getNearstPathPointIndex(const robos::Pose2D& robot_pose, std::vector<robos::Point2d> path)
{
	std::vector<double> dis_gather(path.size());
	for (int i = 0; i < path.size(); i++)
	{
		dis_gather[i] = PLOT_DIS(robot_pose.x, robot_pose.y, path[i].x, path[i].y);
	}
	int nearst_index = std::min_element(dis_gather.begin(), dis_gather.end()) - dis_gather.begin();
	return nearst_index;
}

NavRoad::NavRoad()
{
    NavRoadParams navParams;
    std::string file_path = "./nav_params";
    navParams = CreateRoadTaskDecOptions(file_path);
    Eigen::VectorXd aymin = Eigen::VectorXd::Constant(navParams.Nc, navParams.aymin);
//    aymin(0) = 1.0;
    Eigen::VectorXd aymax = Eigen::VectorXd::Constant(navParams.Nc, navParams.aymax);
    c_mpc = nullptr;
    c_mpc = new C_MPC_ROAD(navParams.q, navParams.r, aymin, aymax, navParams.Nc, navParams.Np, navParams.T, navParams.zeta, navParams.Sob, navParams.num);
}

NavRoad::~NavRoad()
{
    delete c_mpc;
}

void NavRoad::setPath(const std::vector<robos::Point2d>& refer_path)
{
    int path_size = refer_path.size();
    pathsWithAngleRef.resize(path_size);
    for (int i = 0; i < path_size; ++i)
    {
        double angle = (i == 0) ? Get2PointSlop(refer_path[0], refer_path[1])
                                : Get2PointSlop(refer_path[i - 1], refer_path[i]);
        angle = NormalizedAngle(angle, AngleType::RADIAN);
        pathsWithAngleRef[i] = robos::Pose2D(refer_path[i].x, refer_path[i].y, angle);
    }
    c_mpc->set_pathWithAngleRef(pathsWithAngleRef);
}

std::vector<robos::Point2d> NavRoad::convertToPoints(const Eigen::VectorXd& vec)
{
    if (vec.size() % 2 != 0)
    {
        throw std::invalid_argument("The length of the Eigen::VectorXd must be an even number.");
    }

    std::vector<robos::Point2d> points;
    points.reserve(vec.size() / 2); // 预先分配足够的空间

    for (int i = 0; i < vec.size(); i += 2)
    {
        points.emplace_back(vec[i], vec[i + 1]);
    }

    return points;
}

std::vector<robos::Point2d> NavRoad::getRoad(Eigen::VectorXd sideOb, robos::Pose2D curpose, double v)
{
    // 记录开始时间
    auto start = std::chrono::high_resolution_clock::now();

    c_mpc->set_sideOb(sideOb);
    c_mpc->set_poseNow(curpose.x, curpose.y, curpose.theta, v);
    Eigen::VectorXd road = c_mpc->get_road_best();
    if(road.size() == 0)
    {
        std::vector<robos::Point2d> emptyRoad;

        // 记录结束时间
        auto end = std::chrono::high_resolution_clock::now();

        // 计算时间差，转换为秒，并转换为double类型
        double duration = std::chrono::duration<double, std::milli>(end - start).count();
        std::cout << std::endl << "Road Execution Time: " << duration << " milliseconds" << std::endl;

        return emptyRoad;
    }
    std::vector<robos::Point2d> roadResult = convertToPoints(road);

    // 记录结束时间
    auto end = std::chrono::high_resolution_clock::now();

    // 计算时间差，转换为秒，并转换为double类型
    double duration = std::chrono::duration<double, std::milli>(end - start).count();
    std::cout << std::endl << "Road Execution Time: " << duration << " milliseconds" << std::endl;

    return roadResult;
}

//输入车体目标速度、经验值大小、地图分辨率 
//输出追踪个数大小
static int getTrackPointCounts(double experience_value, double target_speed, double map_resolution)
{
	return (int)(experience_value * target_speed / map_resolution);
}

static bool getPathResult(const robos::Pose2D& robot_pose, robos::Line2D robot_xy_vector,
	std::vector<robos::Point2d> local_dynpath, int iterator_size,
	int& error_ref_point_index_T, int& zero_index_for_dynpath)
{
	for (std::vector<robos::Point2d> ::iterator it = local_dynpath.begin(); it != local_dynpath.end(); ++it)
	{
		std::vector<robos::Point2d> ::iterator it_next = it + 1;
		if (it_next == local_dynpath.begin() + iterator_size)
		{
			error_ref_point_index_T = 0;//不能进行控制，需要重新搜路
			zero_index_for_dynpath = 0;//清零
			return {};
		}
		double cross1 = Cross(robot_xy_vector.start_point, robot_xy_vector.end_point, *it_next);
		double cross2 = Cross(robot_xy_vector.start_point, robot_xy_vector.end_point, *it);;
		double error = NormalizedAngle((Get2PointSlop(*it, *it_next) - robot_pose.theta), AngleType::SIGNRADIAN);
		if ((cross1 * cross2 <= 0) && fabs(error) < M_PI / 2)
		{
			return true;
		}
		error_ref_point_index_T += 1;
		zero_index_for_dynpath += 1;
	}
	error_ref_point_index_T = 0;//不能进行控制，需要重新搜路
	zero_index_for_dynpath = 0;//清零
	return false;
}
//小车X正方向的向量<起点，终点>
static robos::Line2D getRobotXYVector(const robos::Pose2D& robot_pose, bool NY_flag = false)
{
	robos::Line2D xyVector;
	//取负y方向后，归一化，根据截距式获取小车基坐标系X轴正方形的向量
	double angle = NY_flag ? robot_pose.theta - M_PI_2 : robot_pose.theta;
	double carAngle_NY =
		GetDoubleSpitLen(NormalizedAngle(angle, AngleType::SIGNRADIAN));
	xyVector.start_point = robos::Point2d(robot_pose.x, robot_pose.y);
	if (fabs(carAngle_NY - RADIANS(90.0)) <= RADIANS(1.0))
	{
		xyVector.end_point = robos::Point2d(robot_pose.x, robot_pose.y + 100.0);
	}
	else if (fabs(carAngle_NY - RADIANS(-90.0)) <= RADIANS(1.0))
	{
		xyVector.end_point = robos::Point2d(robot_pose.x, robot_pose.y - 100.0);
	}
	else
	{
		double k = tan(carAngle_NY);
		double b = robot_pose.y - k * robot_pose.x;
		int num = fabs(carAngle_NY) <= RADIANS(90.0) ? 1 : -1;
		xyVector.end_point = robos::Point2d(robot_pose.x + 100.0 * num, k * (robot_pose.x + 100.0 * num) + b);
	}
	return std::move(xyVector);
}

//根据小车的位姿、追踪个数、获取过零点
static bool getWaitForZeroIndex(const double& target_speed, const std::vector<robos::Point2d>& global_path, const robos::Pose2D& robot_pose,
    int before_error_index, int& error_ref_point_index)
{
    if (global_path.empty())
        return false;
    double min_track_num = 5;
    double map_resolution = 0.05;
    double full_forward_track_counts = getTrackPointCounts(3, target_speed, map_resolution);

    double full_back_track_counts = getTrackPointCounts(2, target_speed, map_resolution);
    std::vector <robos::Point2d> local_dynpath;
    int zero_index_for_dynpath = 0;
    local_dynpath = global_path;
    //起始时刻与初始位置比较，在4个分辨率之内均可
    if (PLOT_DIS(global_path[0].x, global_path[0].y, robot_pose.x, robot_pose.y) <= min_track_num * map_resolution)
    {
        error_ref_point_index = 0;
        return true;
    }
    //当前点不在起始点时
    robos::Line2D robot_xy_vector = getRobotXYVector(robot_pose, true);//小车X正方向的向量<起点，终点>
    int error_ref_point_index_T = before_error_index;
    //在起始位置很差的时候，局部路径搜-全局路径搜-起始点距离判断，均不满足的情况下，则重新规划路径
    if (error_ref_point_index_T == -1)//初始化赋值
    {
        bool flag = getPathResult(robot_pose, robot_xy_vector,
            local_dynpath, local_dynpath.size(),
            error_ref_point_index_T, zero_index_for_dynpath);
        error_ref_point_index = error_ref_point_index_T;
        if (flag)
            return true;
        //全局未找到，则找起始点
        if (PLOT_DIS(global_path[0].x, global_path[0].y, robot_pose.x, robot_pose.y) > MAX_DIS_ERROR_REFP)
            return false;
        error_ref_point_index = 0;
        return  true;
    }
    int backward_counts = error_ref_point_index_T >= full_back_track_counts ?
        full_back_track_counts : error_ref_point_index_T;
    error_ref_point_index_T -= backward_counts; //注意对应的下标也要相减

    int iteration_count = full_forward_track_counts + full_back_track_counts;
    if ((size_t)iteration_count > local_dynpath.size()) iteration_count = local_dynpath.size();
    bool flag = getPathResult(robot_pose, robot_xy_vector,
        local_dynpath, iteration_count, error_ref_point_index_T, zero_index_for_dynpath);
    error_ref_point_index = error_ref_point_index_T;
    return flag;
}

bool NavControl::getZeroPathPointIndex(const robos::Pose2D& robot_pose, const std::vector<robos::Point2d>& path,
                                       int& index)
{
    static int before_error_index = 0;
    bool flag = getWaitForZeroIndex(target_speed_, path, robot_pose, before_error_index, index);
    before_error_index = index;
    return flag;
}

