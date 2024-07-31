// #include <Windows.h>
#include "nav_control.h"
#include "math_formula.hpp"
#include "bezier_smoothing/bezier_smoothing.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>
#include <fstream>
#include <condition_variable>

#define CENTER_ABOVE true
#define CENTER_BELOW false

NavControl navControl;
NavControl navControlLeft;
NavControl navControlRight;
NavRoad navRoad;
NavRoad navRoadLeft;
NavRoad navRoadRight;
std::vector<robos::Point2d> pathsRef;
std::vector<robos::Point2d> pathsRefLeft;
std::vector<robos::Point2d> pathsRefRight;
Eigen::VectorXd sideOb;
// 定义机器人状态量并初始化
robos::Pose2D curpose = robos::Pose2D(0.0, 0.0, 1.57);
robos::Pose2D curposeLeft = robos::Pose2D(-1.0, -1.0, 1.57);
robos::Pose2D curposeRight = robos::Pose2D(1.0, -1.0, 1.57);
double v = 1.0;
navigation::CtrlOutput ctr_output;
navigation::CtrlOutput ctr_outputLeft;
navigation::CtrlOutput ctr_outputRight;
bool result;
bool is_main = true;
bool no_main = false;

// 共享数据和互斥锁
std::mutex pathsMutex;
std::vector<robos::Point2d> pathsTemp;      // 共享变量，存储避障线程计算的结果
std::vector<robos::Point2d> pathsTempLeft;  // 共享变量，存储避障线程计算的结果
std::vector<robos::Point2d> pathsTempRight; // 共享变量，存储避障线程计算的结果
std::condition_variable pathsCondition;
bool readyForNewPath = false; // 控制是否准备好计算新路径

Eigen::VectorXd calculate_coordinateConversionFromWorld(double d_x, double d_y, double d_alpha, double d_dx, double d_dy)
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

robos::Pose2D updatePose(const robos::Pose2D &curpose, const navigation::CtrlOutput &ctroutput)
{
    double Vx = ctroutput.lineV * cos(ctroutput.angle);
    double Vy = ctroutput.lineV * sin(ctroutput.angle);

    double dt = 0.1, dx = 0.0, dy = 0.0, R = 0.0, half_angle = 0.0;
    double car_angle = NormalizedAngle(curpose.theta, AngleType::RADIAN);
    if (fabs(ctroutput.angularV) > 1e-5)
    {
        half_angle = ctroutput.angularV * dt / 2;
        R = fabs(ctroutput.lineV) / ctroutput.angularV;
        dx = 2 * R * sin(half_angle) * cos(half_angle);
        dy = -2 * R * sin(half_angle) * sin(half_angle);
    }
    else
    {
        dx = fabs(Vx) * dt;
        dy = fabs(Vy) * dt;
    }
    robos::Pose2D result;
    result.x = cos(car_angle) * dx - sin(car_angle) * dy + curpose.x;
    result.y = sin(car_angle) * dx + cos(car_angle) * dy + curpose.y;
    result.theta = NormalizedAngle((car_angle - half_angle * 2), AngleType::RADIAN);
    return result;
}

// 路径分割  按地图分辨率进行分割
std::vector<robos::Point2d> segLinePath(const robos::Point2d &start_point,
                                        const robos::Point2d &end_point, const double &map_resolution)
{
    std::vector<robos::Point2d> point_list;
    double angle = Get2PointSlop(start_point, end_point);
    double dist = PLOT_DIS(start_point.x, start_point.y, end_point.x, end_point.y);
    int n = 0;
    while (n * map_resolution < dist)
    {
        robos::Point2d point;
        point.x = start_point.x + n * map_resolution * cos(angle);
        point.y = start_point.y + n * map_resolution * sin(angle);
        point_list.push_back(point);
        n++;
    }
    if (!point_list.empty())
    {
        point_list.pop_back();
        point_list.push_back(robos::Point2d(end_point.x, end_point.y));
    }
    return point_list;
}

std::vector<robos::Point2d> segLinePathAll(const std::vector<robos::Point2d> pathRaw, const double &map_resolution)
{
    std::vector<robos::Point2d> segmentedPath;
    for (size_t i = 0; i < pathRaw.size() - 1; ++i)
    {
        // 对于路径上的每一对相邻点，使用segLinePath进行分割
        std::vector<robos::Point2d> segment = segLinePath(pathRaw[i], pathRaw[i + 1], map_resolution);
        // 将分割得到的点添加到总路径中（除了最后一个点，以避免重复）
        segmentedPath.insert(segmentedPath.end(), segment.begin(), segment.end() - 1);
    }
    // 添加最后一个点
    if (!pathRaw.empty())
    {
        segmentedPath.push_back(pathRaw.back());
    }
    return segmentedPath;
}

std::vector<robos::Point2d> quarterEllipsePath(const robos::Point2d &start_point,
                                               const robos::Point2d &end_point,
                                               const double map_resolution,
                                               const bool center_above)
{
    std::vector<robos::Point2d> point_list;

    double x1 = start_point.x;
    double y1 = start_point.y;
    double x2 = end_point.x;
    double y2 = end_point.y;

    // 计算长轴和短轴
    double a = std::abs(x2 - x1); // 长轴的一半
    double b = std::abs(y2 - y1); // 短轴的一半

    // 计算椭圆中心点
    robos::Point2d center_point;
    if (!center_above)
    {
        if (x1 < x2 && y1 < y2) // 情况A
        {
            center_point.x = x2;
            center_point.y = y1;
        }
        else if (x1 < x2 && y1 > y2) // 情况B
        {
            center_point.x = x1;
            center_point.y = y2;
        }
        else if (x1 > x2 && y1 < y2) // 情况C
        {
            center_point.x = x2;
            center_point.y = y1;
        }
        else if (x1 > x2 && y1 > y2) // 情况D
        {
            center_point.x = x1;
            center_point.y = y2;
        }
    }
    else
    {
        if (x1 < x2 && y1 < y2) // 情况A
        {
            center_point.x = x1;
            center_point.y = y2;
        }
        else if (x1 < x2 && y1 > y2) // 情况B
        {
            center_point.x = x2;
            center_point.y = y1;
        }
        else if (x1 > x2 && y1 < y2) // 情况C
        {
            center_point.x = x1;
            center_point.y = y2;
        }
        else if (x1 > x2 && y1 > y2) // 情况D
        {
            center_point.x = x2;
            center_point.y = y1;
        }
    }

    // 确定起始角度和结束角度
    double start_angle = 0;
    double end_angle = M_PI / 2; // 四分之一椭圆

    // 计算椭圆周长近似值
    double perimeter = M_PI * (3 * (a + b) - sqrt((3 * a + b) * (a + 3 * b)));
    double angular_step = map_resolution / perimeter * 2 * M_PI;
    int steps = static_cast<int>(end_angle / angular_step);

    // 生成椭圆点
    for (int i = 0; i <= steps; ++i)
    {
        double angle = start_angle + angular_step * i;
        robos::Point2d point;

        // 根据不同情况生成点
        if (!center_above)
        {
            if (x1 < x2 && y1 < y2) // 情况A
            {
                point.x = center_point.x - a * cos(angle);
                point.y = center_point.y + b * sin(angle);
            }
            else if (x1 < x2 && y1 > y2) // 情况B
            {
                point.x = center_point.x + a * sin(angle);
                point.y = center_point.y + b * cos(angle);
            }
            else if (x1 > x2 && y1 < y2) // 情况C
            {
                point.x = center_point.x + a * cos(angle);
                point.y = center_point.y + b * sin(angle);
            }
            else if (x1 > x2 && y1 > y2) // 情况D
            {
                point.x = center_point.x - a * sin(angle);
                point.y = center_point.y + b * cos(angle);
            }
        }
        else
        {
            if (x1 < x2 && y1 < y2) // 情况A
            {
                point.x = center_point.x + a * sin(angle);
                point.y = center_point.y - b * cos(angle);
            }
            else if (x1 < x2 && y1 > y2) // 情况B
            {
                point.x = center_point.x - a * cos(angle);
                point.y = center_point.y - b * sin(angle);
            }
            else if (x1 > x2 && y1 < y2) // 情况C
            {
                point.x = center_point.x - a * sin(angle);
                point.y = center_point.y - b * cos(angle);
            }
            else if (x1 > x2 && y1 > y2) // 情况D
            {
                point.x = center_point.x + a * cos(angle);
                point.y = center_point.y - b * sin(angle);
            }
        }

        point_list.push_back(point);
    }

    return point_list;
}

std::vector<robos::Point2d> arcPath(const robos::Point2d &start_point,
                                    const robos::Point2d &end_point,
                                    const double map_resolution,
                                    const double arc_angle,
                                    const bool center_above)
{
    std::vector<robos::Point2d> point_list;

    // 计算圆心和半径
    double dx = end_point.x - start_point.x;
    double dy = end_point.y - start_point.y;
    double dist = sqrt(dx * dx + dy * dy);
    double radius = dist / (2 * sin(arc_angle / 2));

    // 计算圆心坐标
    double mid_x = (start_point.x + end_point.x) / 2;
    double mid_y = (start_point.y + end_point.y) / 2;
    double angle_to_midpoint = atan2(dy, dx);

    robos::Point2d center_point;
    double offset = sqrt(radius * radius - (dist / 2) * (dist / 2));
    if (center_above)
    {
        center_point.x = mid_x + offset * cos(angle_to_midpoint + M_PI / 2);
        center_point.y = mid_y + offset * sin(angle_to_midpoint + M_PI / 2);
    }
    else
    {
        center_point.x = mid_x - offset * cos(angle_to_midpoint + M_PI / 2);
        center_point.y = mid_y - offset * sin(angle_to_midpoint + M_PI / 2);
    }

    // 计算起始角度和结束角度
    double start_angle = atan2(start_point.y - center_point.y, start_point.x - center_point.x);
    double end_angle = atan2(end_point.y - center_point.y, end_point.x - center_point.x);

    // 步长和步数
    double angular_step = map_resolution / radius;
    int steps = static_cast<int>(arc_angle / angular_step);

    // 生成圆弧点
    if (cos(start_angle) < cos(end_angle))
    {
        if (center_above)
        {
            for (int i = 0; i <= steps; ++i)
            {
                robos::Point2d point;
                double angle = start_angle + angular_step * i;
                point.x = center_point.x + radius * cos(angle);
                point.y = center_point.y + radius * sin(angle);
                point_list.push_back(point);
            }
        }
        else
        {
            for (int i = 0; i <= steps; ++i)
            {
                robos::Point2d point;
                double angle = start_angle - angular_step * i;
                point.x = center_point.x + radius * cos(angle);
                point.y = center_point.y + radius * sin(angle);
                point_list.push_back(point);
            }
        }
    }
    else
    {
        if (center_above)
        {
            for (int i = 0; i <= steps; ++i)
            {
                robos::Point2d point;
                double angle = start_angle - angular_step * i;
                point.x = center_point.x + radius * cos(angle);
                point.y = center_point.y + radius * sin(angle);
                point_list.push_back(point);
            }
        }
        else
        {
            for (int i = 0; i <= steps; ++i)
            {
                robos::Point2d point;
                double angle = start_angle + angular_step * i;
                point.x = center_point.x + radius * cos(angle);
                point.y = center_point.y + radius * sin(angle);
                point_list.push_back(point);
            }
        }
    }

    return point_list;
}

void initializePathsTemp()
{
    // 这里给pathsTemp赋一个初始值，可以是空的，也可以是具有初始路径的
    pathsTemp = pathsRef;
}

void computePaths()
{
    while (true)
    {
        {
            std::unique_lock<std::mutex> lock(pathsMutex);
            // 等待直到runPaths处理完当前路径
            while (!readyForNewPath)
            {
                pathsCondition.wait(lock);
            }
        }
        std::vector<robos::Point2d> rawPaths = navRoad.getRoad(sideOb, curpose, v);
        if (rawPaths.size() == 0)
        {
            rawPaths = pathsRef;
        }
        std::cout << std::endl
                  << "getRoad " << rawPaths.size() << std::endl;
        //        std::vector<robos::Point2d> newPaths = segLinePathAll(rawPaths, 0.05);
        {
            std::lock_guard<std::mutex> lock(pathsMutex);
            pathsTemp = rawPaths;    // 更新paths
            readyForNewPath = false; // 重置标志
        }
        // 线程休眠0.2秒，放在这里确保至少等待这段时间后再计算新路径
        //        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void runPaths()
{
    while (true)
    {
        std::vector<robos::Point2d> currentPaths;
        {
            std::lock_guard<std::mutex> lock(pathsMutex);
            currentPaths = pathsTemp; // 获取最新的paths
        }

        navControl.setPath(currentPaths);
        bool result = navControl.getCtrlResult(curpose, ctr_output);
        if (result)
        {
            curpose = updatePose(curpose, ctr_output);
            {
                std::lock_guard<std::mutex> lock(pathsMutex);
                readyForNewPath = true; // 设置标志，让computePaths可以计算新路径
            }
            pathsCondition.notify_all(); // 通知computePaths
        }
        else
        {
            break; // 或者处理错误
        }
        // 线程休眠0.05秒，放在这里确保至少等待这段时间后再计算新路径
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void computePathsTest(NavRoad &_navRoad, robos::Pose2D _curpose, std::vector<robos::Point2d> _pathsRef, std::vector<robos::Point2d> &_pathsTemp)
{
    std::vector<robos::Point2d> rawPaths = _navRoad.getRoad(sideOb, _curpose, v);
    if (rawPaths.size() == 0)
    {
        rawPaths = _pathsRef;
    }
    std::cout << std::endl
              << "getRoad " << rawPaths.size() << std::endl;
    _pathsTemp = rawPaths; // 更新paths
}

bool runPathsTest(NavControl &_navControl, robos::Pose2D &_curpose, navigation::CtrlOutput &_ctr_output, bool _ismain, std::vector<robos::Point2d> &_pathsTemp)
{
    std::vector<robos::Point2d> currentPaths;
    currentPaths = _pathsTemp; // 获取最新的paths
    _navControl.setPath(currentPaths);
    bool result = _navControl.getCtrlResult(_curpose, _ctr_output, _ismain);
    if (result)
    {
        _curpose = updatePose(_curpose, _ctr_output);
        return true;
    }
    else
    {
        return false;
    }
}

void getPathsRef(std::vector<robos::Point2d> &pathsRef, std::vector<robos::Point2d> path_points)
{
    std::vector<robos::Point2d> temp_path;

    //    temp_path = segLinePath(path_points[0], path_points[1], 0.05);
    //    pathsRef.insert(pathsRef.end(), temp_path.cbegin(), temp_path.cend());

    //    temp_path = arcPath(path_points[1], path_points[2], 0.05, 1.57, CENTER_BELOW);
    //    pathsRef.insert(pathsRef.end(), temp_path.cbegin(), temp_path.cend());

    //    temp_path = segLinePath(path_points[2], path_points[3], 0.05);
    //    pathsRef.insert(pathsRef.end(), temp_path.cbegin(), temp_path.cend());

    //    temp_path = arcPath(path_points[3], path_points[4], 0.05, 1.57, CENTER_BELOW);
    //    pathsRef.insert(pathsRef.end(), temp_path.cbegin(), temp_path.cend());

    //    temp_path = segLinePath(path_points[4], path_points[5], 0.05);
    //    pathsRef.insert(pathsRef.end(), temp_path.cbegin(), temp_path.cend());

    temp_path = arcPath(path_points[0], path_points[1], 0.05, 1.57, CENTER_BELOW);
    pathsRef.insert(pathsRef.end(), temp_path.cbegin(), temp_path.cend());

    temp_path = arcPath(path_points[1], path_points[2], 0.05, 1.57, CENTER_ABOVE);
    pathsRef.insert(pathsRef.end(), temp_path.cbegin(), temp_path.cend());

    temp_path = segLinePath(path_points[2], path_points[3], 0.05);
    pathsRef.insert(pathsRef.end(), temp_path.cbegin(), temp_path.cend());
}

void getPathsRightLeftRef(std::vector<robos::Point2d> &pathsRef, std::vector<robos::Point2d> path_points)
{
    std::vector<robos::Point2d> temp_path;

    //    temp_path = segLinePath(path_points[0], path_points[1], 0.05);
    //    pathsRef.insert(pathsRef.end(), temp_path.cbegin(), temp_path.cend());

    //    temp_path = arcPath(path_points[1], path_points[2], 0.05, 1.57, CENTER_BELOW);
    //    pathsRef.insert(pathsRef.end(), temp_path.cbegin(), temp_path.cend());

    //    temp_path = segLinePath(path_points[2], path_points[3], 0.05);
    //    pathsRef.insert(pathsRef.end(), temp_path.cbegin(), temp_path.cend());

    //    temp_path = arcPath(path_points[3], path_points[4], 0.05, 1.57, CENTER_BELOW);
    //    pathsRef.insert(pathsRef.end(), temp_path.cbegin(), temp_path.cend());

    //    temp_path = segLinePath(path_points[4], path_points[5], 0.05);
    //    pathsRef.insert(pathsRef.end(), temp_path.cbegin(), temp_path.cend());

    temp_path = quarterEllipsePath(path_points[0], path_points[1], 0.05, CENTER_BELOW);
    pathsRef.insert(pathsRef.end(), temp_path.cbegin(), temp_path.cend());

    temp_path = quarterEllipsePath(path_points[1], path_points[2], 0.05, CENTER_ABOVE);
    pathsRef.insert(pathsRef.end(), temp_path.cbegin(), temp_path.cend());

    temp_path = segLinePath(path_points[2], path_points[3], 0.05);
    pathsRef.insert(pathsRef.end(), temp_path.cbegin(), temp_path.cend());
}

void getPathPoints(const double scale, std::vector<robos::Point2d> &path_points, std::vector<robos::Point2d> &path_pointsLeft, std::vector<robos::Point2d> &path_pointsRight)
{
    path_points.push_back(robos::Point2d(0, 0));
    path_points.push_back(robos::Point2d(10 * scale, 10 * scale));
    path_points.push_back(robos::Point2d(20 * scale, 20 * scale));
    path_points.push_back(robos::Point2d(20 * scale, 21 * scale));

    path_pointsLeft.push_back(robos::Point2d(-1 * scale, -1 * scale));
    path_pointsLeft.push_back(robos::Point2d(9 * scale, 11 * scale));
    path_pointsLeft.push_back(robos::Point2d(19 * scale, 19 * scale));
    path_pointsLeft.push_back(robos::Point2d(19 * scale, 20 * scale));

    path_pointsRight.push_back(robos::Point2d(1 * scale, -1 * scale));
    path_pointsRight.push_back(robos::Point2d(9 * scale, 9 * scale));
    path_pointsRight.push_back(robos::Point2d(21 * scale, 19 * scale));
    path_pointsRight.push_back(robos::Point2d(21 * scale, 20 * scale));

    //    path_points.push_back(robos::Point2d(0, 0));
    //    path_points.push_back(robos::Point2d(0, 5 * scale));
    //    path_points.push_back(robos::Point2d(5 * scale, 10 * scale));
    //    path_points.push_back(robos::Point2d(10 * scale, 10 * scale));
    //    path_points.push_back(robos::Point2d(15 * scale, 5 * scale));
    //    path_points.push_back(robos::Point2d(15 * scale, 0));

    //    path_pointsLeft.push_back(robos::Point2d(-1 * scale, -1 * scale));
    //    path_pointsLeft.push_back(robos::Point2d(-1 * scale, 5 * scale));
    //    path_pointsLeft.push_back(robos::Point2d(5 * scale, 11 * scale));
    //    path_pointsLeft.push_back(robos::Point2d(10 * scale, 11 * scale));
    //    path_pointsLeft.push_back(robos::Point2d(16 * scale, 5 * scale));
    //    path_pointsLeft.push_back(robos::Point2d(16 * scale, 1 * scale));

    //    path_pointsRight.push_back(robos::Point2d(1 * scale, -1 * scale));
    //    path_pointsRight.push_back(robos::Point2d(1 * scale, 5 * scale));
    //    path_pointsRight.push_back(robos::Point2d(5 * scale, 9 * scale));
    //    path_pointsRight.push_back(robos::Point2d(10 * scale, 9 * scale));
    //    path_pointsRight.push_back(robos::Point2d(14 * scale, 5 * scale));
    //    path_pointsRight.push_back(robos::Point2d(14 * scale, 1 * scale));
}

void init_ctr_output(navigation::CtrlOutput &ctr_output)
{
    ctr_output.angle = 0.0;
    ctr_output.angularV = 0.0;
    ctr_output.lineV = 0.0;
    ctr_output.move_model = 0;
    ctr_output.reduce_model = 0;
}

void runAllSameTime(double scale, NavControl &navControl, NavControl &navControlLeft, NavControl &navControlRight, std::vector<std::vector<robos::Point2d>> pathAll, bool result, std::vector<robos::Pose2D> curposeAll, std::vector<navigation::CtrlOutput> ctr_outputAll)
{
    while (1)
    {
        double scaleV = 0.1;
        navControl.setPath(pathAll[0]);
        navControlLeft.setPath(pathAll[1]);
        navControlRight.setPath(pathAll[2]);
        double d = scale * sqrt(2.0);
        result = 1;
        result *= navControl.getCtrlResult(curposeAll[0], ctr_outputAll[0], is_main);
        result *= navControlLeft.getCtrlResult(curposeAll[1], ctr_outputAll[1], no_main);
        result *= navControlRight.getCtrlResult(curposeAll[2], ctr_outputAll[2], no_main);
        Eigen::VectorXd coordinateLeft = calculate_coordinateConversionFromWorld(curposeAll[1].x, curposeAll[1].y, curposeAll[0].theta, curposeAll[0].x, curposeAll[0].y);
        double d1 = coordinateLeft(0) + 1.0;
        Eigen::VectorXd coordinateRight = calculate_coordinateConversionFromWorld(curposeAll[2].x, curposeAll[2].y, curposeAll[0].theta, curposeAll[0].x, curposeAll[0].y);
        double d2 = coordinateRight(0) + 1.0;
        if (result)
        {
            curposeAll[0] = updatePose(curposeAll[0], ctr_outputAll[0]);
            curposeAll[1] = updatePose(curposeAll[1], ctr_outputAll[1]);
            curposeAll[2] = updatePose(curposeAll[2], ctr_outputAll[2]);
        }
        else
        {
            break;
        }
    }
}

void runOne(int i, NavControl &navControl, std::vector<std::vector<robos::Point2d>> pathAll, bool result, std::vector<robos::Pose2D> curposeAll, std::vector<navigation::CtrlOutput> ctr_outputAll)
{
    while (1)
    {
        navControl.setPath(pathAll[i]);
        result = 1;
        result *= navControl.getCtrlResult(curposeAll[i], ctr_outputAll[i], is_main);
        if (result)
        {
            curposeAll[i] = updatePose(curposeAll[i], ctr_outputAll[i]);
        }
        else
        {
            break;
        }
    }
}

void runAllNoSameTime(int zero, int one, int two, NavControl &navControl, NavControl &navControlLeft, NavControl &navControlRight, std::vector<std::vector<robos::Point2d>> pathAll, bool result, std::vector<robos::Pose2D> curposeAll, std::vector<navigation::CtrlOutput> ctr_outputAll)
{
    runOne(zero, navControl, pathAll, result, curposeAll, ctr_outputAll);
    //    runOne(one, navControlLeft, pathAll, result, curposeAll, ctr_outputAll);
    //    runOne(two, navControlRight, pathAll, result, curposeAll, ctr_outputAll);
}

void runOneWithObstacles(double _threshold, NavRoad &_navRoad, std::vector<robos::Point2d> &_pathsTemp, bool &_control, robos::Pose2D &_curpose, std::vector<robos::Point2d> &_pathsRef, NavControl &_navControl, navigation::CtrlOutput &_ctr_output, bool _ismain)
{
    while (1)
    {
        computePathsTest(_navRoad, _curpose, _pathsRef, _pathsTemp);
        for (int i = 0; i < 5; i++)
        {
            _control = runPathsTest(_navControl, _curpose, _ctr_output, _ismain, _pathsTemp);
            if (!_control)
            {
                break;
            }
        }
        if (!_control)
        {
            break;
        }
        if (_curpose.y > _threshold)
        {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main()
{
    //    BezierSmoothing bezirSmoothing;

    // 机器人跟踪的路径点
    std::vector<robos::Point2d> path_points;
    std::vector<robos::Point2d> path_pointsLeft;
    std::vector<robos::Point2d> path_pointsRight;

    //    double scale = 0.5;
    double scale = 1.0;
    //    double scale = 2.0;

    getPathPoints(scale, path_points, path_pointsLeft, path_pointsRight);

    // 对路径点进行分割
    //    for (size_t i = 0; i < path_points.size() - 1; i++)
    //    {
    //        std::vector<robos::Point2d> temp_path
    //            = segLinePath(path_points[i], path_points[i + 1], 0.05);
    //        paths.insert(paths.end(), temp_path.cbegin(), temp_path.cend());
    //    }

    getPathsRef(pathsRef, path_points);
    if (!pathsRef.empty())
    {
        pathsRef.erase(pathsRef.begin());
    }
    getPathsRightLeftRef(pathsRefLeft, path_pointsLeft);
    getPathsRightLeftRef(pathsRefRight, path_pointsRight);

    std::vector<std::vector<robos::Point2d>> pathAll;

    pathAll.push_back(pathsRef);
    pathAll.push_back(pathsRefLeft);
    pathAll.push_back(pathsRefRight);

    // 对路径点进行平滑
    //    paths = bezirSmoothing.getGlobalBezier(paths);
    // 获得角度
    navRoad.setPath(pathAll[0]);
    navRoadLeft.setPath(pathAll[1]);
    navRoadRight.setPath(pathAll[2]);

    // 障碍物
    sideOb.resize(4);
    sideOb << 0.0, 2.0, 10.0, 10.0;
    //    sideOb.resize(0);

    robos::Pose2D curposeTest = robos::Pose2D(0.000354136, 0.495, 1.57);
    std::vector<robos::Pose2D> curposeAll;
    curposeAll.push_back(curpose);
    curposeAll.push_back(curposeLeft);
    curposeAll.push_back(curposeRight);

    // 定义机器人当前控制量并初始化
    init_ctr_output(ctr_output);
    init_ctr_output(ctr_outputLeft);
    init_ctr_output(ctr_outputRight);

    std::vector<navigation::CtrlOutput> ctr_outputAll;
    ctr_outputAll.push_back(ctr_output);
    ctr_outputAll.push_back(ctr_outputLeft);
    ctr_outputAll.push_back(ctr_outputRight);

    // 获得控制量是否成功
    bool control = true;

    initializePathsTemp(); // 确保pathsTemp在多线程启动前被初始化

    //    std::thread roadThread(computePaths);
    //    std::thread controlThread(runPaths);
    //    roadThread.join();
    //    controlThread.join();

    //    runAllSameTime(scale, navControl, navControlLeft, navControlRight, pathAll, result, curposeAll, ctr_outputAll);

    int zero = 0;
    int one = 1;
    int two = 2;

    //    runAllNoSameTime(zero, one, two, navControl, navControlLeft, navControlRight, pathAll, result, curposeAll, ctr_outputAll);

    double threshold = 21.0;

    runOneWithObstacles(threshold, navRoad, pathsTemp, control, curpose, pathsRef, navControl, ctr_output, is_main);
    runOneWithObstacles(threshold - 1, navRoadLeft, pathsTempLeft, control, curposeLeft, pathsRefLeft, navControlLeft, ctr_outputLeft, no_main);
    runOneWithObstacles(threshold - 1, navRoadRight, pathsTempRight, control, curposeRight, pathsRefRight, navControlRight, ctr_outputRight, no_main);

    system("pause");
    return 0;
}
