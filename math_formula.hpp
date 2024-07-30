#pragma once
#include <fstream>
#include <iostream>
#include <robos/navi_msgs.h>
#include <queue>
#include <vector>
#define RADIANS(angle)         ((angle) * M_PI/  180) //角度制转弧度制
#define REV_RADIANS(radian)     ((radian) * 180 / M_PI) //弧度制转角度制
#define PLOT_DIS(x1,y1,x2,y2)  std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))//两点的距离
#define CROSS(x1,y1,x2,y2) ((x1) * (y2) - (x2) * (y1))//差乘公式
#define MANHANT_DIS(x1,y1,x2,y2)  (fabs(x2 - x1) + fabs(y2 - y1))
#define MIN_POSITIVE 0.001
//角度(0-360) 弧度(0-2PI)、半弧度（0-PI）、符号角度(-180-180)、符号弧度(-PI-PI)
enum class AngleType
{
    DEGREE,
    RADIAN,
    HALFRADIAN,
    SIGNDEGREE,
    SIGNRADIAN
};
static double NormalizedAngle(double angle, const AngleType& angle_type)
{
    switch (angle_type)
    {
    case AngleType::DEGREE://度0-360
        while (angle >= 360.0)
        {
            angle -= 360.0;
        }
        while (angle < 0.0)
        {
            angle += 360.0;
        }
        break;
    case AngleType::RADIAN://弧度0-2PI
        while (angle >= 2 * M_PI)
        {
            angle -= 2 * M_PI;
        }
        while (angle < 0.0)
        {
            angle += 2 * M_PI;
        }
        break;
    case AngleType::HALFRADIAN://0-PI
        while (angle > M_PI)
        {
            angle = 2 * M_PI - angle;
        }
        break;
    case AngleType::SIGNDEGREE: //-180-180
        while (angle > 180.0)
        {
            angle -= 360.0;
        }
        while (angle <= -180.0)
        {
            angle += 360.0;
        }
        break;
    case AngleType::SIGNRADIAN:
        while (angle > M_PI)
        {
            angle -= 2 * M_PI;
        }
        while (angle <= -M_PI)
        {
            angle += 2 * M_PI;
        }
        break;
    default:
        break;
    }
    return angle;
}

//获取两点的斜率
static double Get2PointSlop(robos::Point2d startp, robos::Point2d endp)
{
    return atan2(endp.y - startp.y, endp.x - startp.x);
}
//设定有效小数个数的两倍
static double GetDoubleSpitLen(const double& d, const int& sign_diget_len = 6)
{
    double t = (double)(d * pow(10, sign_diget_len) + 0.5);//四舍五入
    return (double)(t / pow(10, sign_diget_len));
}
static double Cross(const std::pair<robos::Point2d, robos::Point2d>& sv,
    const std::pair<robos::Point2d, robos::Point2d>& ev)
{
    return GetDoubleSpitLen(CROSS(sv.second.x - sv.first.x, sv.second.y - sv.first.y,
        ev.second.x - ev.first.x, ev.second.y - ev.first.y));
}

static double Cross(const robos::Point2d& svp, const robos::Point2d& evp, const robos::Point2d& judge_point)
{
    return GetDoubleSpitLen(CROSS(evp.x - svp.x, evp.y - svp.y,
        judge_point.x - svp.x, judge_point.y - svp.y));
}

//最小二乘法拟合直线
/* robos::Line2D GetTargetStraightLine(const std::vector<robos::Point2d>& fit_points)
{
    size_t fit_num = fit_points.size();
    if (fit_num < 2)
        return robos::Line2D(robos::Point2d(0.0, 0.0), robos::Point2d(0.0, 0.0));
    double sumX = 0, sumY = 0, sumX2 = 0, sumXY = 0, b_den = 0,
        b_num = 0, k = 0.0, b = 0.0;
    robos::Point2d start_point = fit_points.front();
    robos::Point2d end_point = fit_points.back();
    robos::Line2D pull_line(start_point, end_point);
    for (const auto& it : fit_points)
    {
        sumX += it.x;
        sumY += it.y;
        sumX2 += it.x * it.x;
        sumXY += it.x * it.y;
    }
    b_den = fit_num * sumX2 - pow(sumX, 2);
    b_num = sumX2 * sumY - sumX * sumXY;
    //b_den绝对值 < 0.001 认为是垂直于X轴的线
    if (fabs(b_den) < MIN_POSITIVE)		return pull_line;

    k = (fit_num * sumXY - sumX * sumY) / (b_den);

    if (fabs(k) > tan(RADIANS(89)))		return pull_line;

    b = b_num / (b_den);
    // 计算相关系数r
    double Xmean = sumX / fit_num;
    double Ymean = sumY / fit_num;
    double tempSumXX = 0.0, tempSumYY = 0.0, E = 0.0, F = 0.0;
    for (const auto& it : fit_points)
    {
        tempSumXX += (it.x - Xmean) * (it.x - Xmean);
        tempSumYY += (it.y - Ymean) * (it.y - Ymean);
        E += (it.x - Xmean) * (it.y - Ymean);
    }
    F = sqrt(tempSumXX) * sqrt(tempSumYY);
    if (fabs(F) < MIN_POSITIVE) return pull_line;

    double r = E / F;//相关系数
    if (fabs(r) < 0.2) return pull_line;

    robos::Point2d p1(start_point.x, k * (start_point.x) + b);
    robos::Point2d p2(end_point.x, k * (end_point.x) + b);
    if (MANHANT_DIS(p1.x, p1.y, start_point.x, start_point.y)
        < MANHANT_DIS(p2.x, p2.y, start_point.x, start_point.y))
        return robos::Line2D(p1, p2);
    return robos::Line2D(p2, p1);
} */

static double getCircleRadius(const std::vector<robos::Point2d>& circlePoint)
{
    double epsilon = 1e-9;
    double a = PLOT_DIS(circlePoint[0].x, circlePoint[0].y, circlePoint[1].x, circlePoint[1].y);
    double b = PLOT_DIS(circlePoint[1].x, circlePoint[1].y, circlePoint[2].x, circlePoint[2].y);
    double c = PLOT_DIS(circlePoint[2].x, circlePoint[2].y, circlePoint[0].x, circlePoint[0].y);
    if((a + b - c < epsilon) || (b + c - a < epsilon) || (c + a - b < epsilon))
    {
        return 0;
    }
    double cosABC = (a * a + c * c - b * b)/(2 * a * c);
    double angleABC = acos(cosABC);
    double angleADC = M_PI - angleABC;
    double circleRadius = b/(2 * sin(angleADC));
//    std::cout << "  radius: " << circleRadius << " ";
    return circleRadius;
}

static int getDeltaDirection(const std::vector<robos::Point2d>& circlePoints)
{
    // 计算向量AB和BC
    robos::Point2d AB = {circlePoints[1].x - circlePoints[0].x, circlePoints[1].y - circlePoints[0].y};
    robos::Point2d BC = {circlePoints[2].x - circlePoints[1].x, circlePoints[2].y - circlePoints[1].y};

    // 计算叉积
    double crossProduct = AB.x * BC.y - AB.y * BC.x;

    if (crossProduct > 0)
    {
        return -1;
    }
    return 1;
}

static double getDelta(const double l, const std::vector<robos::Point2d>& circlePoint)
{
    double circleRadius = getCircleRadius(circlePoint);
    if(circleRadius == 0)
    {
        return 0;
    }
    double delta = - getDeltaDirection(circlePoint) * atan(l / circleRadius);
    return delta;
}

static double getAngularV(const double lineV, const double angle, double l)
{
    if(angle == 0)
    {
        return 0;
    }
    double circleRadius = l / (tan(angle));
    double angularV = - lineV / circleRadius;
    return angularV;
}

static int count = 0;

static std::vector<robos::Point2d> getPathPoints(std::vector<robos::Point2d> paths, std::vector<robos::Pose2D> pathsWithAngle)
{
    std::vector<robos::Point2d> path_points;
    path_points.push_back(paths[0]);
    for(int i = 0; i < paths.size() - 1; i++)
    {
        while((fabs(pathsWithAngle[i].theta - pathsWithAngle[i + 1].theta) <= 10e-7
                || fabs(pathsWithAngle[i].theta - pathsWithAngle[i + 1].theta + 6.283185) <= 10e-7
                || fabs(pathsWithAngle[i].theta - pathsWithAngle[i + 1].theta - 6.283185) <= 10e-7)
               && i < paths.size() - 1)
        {
            i++;
        }
        if(i < paths.size())
        {
            path_points.push_back(paths[i]);
        }
        while((fabs(pathsWithAngle[i].theta - pathsWithAngle[i + 1].theta) >= 10e-7
                && fabs(pathsWithAngle[i].theta - pathsWithAngle[i + 1].theta + 6.283185) >= 10e-7
                && fabs(pathsWithAngle[i].theta - pathsWithAngle[i + 1].theta - 6.283185) >= 10e-7)
               && i < paths.size() - 1)
        {
            i++;
        }
        if(i < paths.size())
        {
            path_points.push_back(paths[i]);
        }
    }
    path_points.pop_back();
    return path_points;
}
static int curve = 0;
static double getVRef(std::vector<robos::Point2d> path_points, const robos::Pose2D& robot_pose, const double vMax, const std::vector<robos::Pose2D> pathsWithAngle, const int nearst_index)
{
    double v;
    double distance0 = 4;
    double distance = 0.0;
    double distanceMax = 0.0;

//    if(count >= path_points.size() - 1)
//    {
//        count = 0;
//        return 0;
//    }
//    distance = PLOT_DIS(robot_pose.x, robot_pose.y, path_points[count].x, path_points[count].y);
//    distanceMax = PLOT_DIS(path_points[count].x, path_points[count].y, path_points[count + 1].x, path_points[count + 1].y);
//    if((count == 0) && (nearst_index < 10))
//    {
//        if((fabs(pathsWithAngle[nearst_index].theta - pathsWithAngle[nearst_index + 1].theta) >= 10e-6
//             && fabs(pathsWithAngle[nearst_index].theta - pathsWithAngle[nearst_index + 1].theta + 6.283185) >= 10e-6
//             && fabs(pathsWithAngle[nearst_index].theta - pathsWithAngle[nearst_index + 1].theta - 6.283185) >= 10e-6))
//        {
//            curve = 1;
//        }
//    }
//    if(distance >= distanceMax)
//    {
//        count++;
//        if((fabs(pathsWithAngle[nearst_index].theta - pathsWithAngle[nearst_index + 1].theta) >= 10e-6
//             && fabs(pathsWithAngle[nearst_index].theta - pathsWithAngle[nearst_index + 1].theta + 6.283185) >= 10e-6
//             && fabs(pathsWithAngle[nearst_index].theta - pathsWithAngle[nearst_index + 1].theta - 6.283185) >= 10e-6))
//        {
//            curve = 1;
//        }
//        else
//        {
//            curve = 0;
//        }
//    }

    int adaptiveV = 0;

    if(adaptiveV)
    {
        if(!curve)
        {
            if(distanceMax > distance0)
            {
                if(distance < (distance0 / 2))
                {
                    v = 0.5 * (1 - ((2 * distance / distance0) - 1) * ((2 * distance / distance0) - 1)) + 0.5;
                }
                else if(distanceMax - distance < (distance0 / 2))
                {
                    v = 0.5 * (1 - ((2 * (distanceMax - distance) / distance0) - 1) * ((2 * (distanceMax - distance) / distance0) - 1)) + 0.5;
                }
                else
                {
                    v = 1.0;
                }
            }
            else if(distanceMax > 2.5)
            {
                v = 0.5 * (1 - ((2 * distance / distanceMax) - 1) * ((2 * distance / distanceMax) - 1)) + 0.5;
            }
            else
            {
                v = 0.5;
            }
        }
        else
        {
            v = 0.5;
        }
    }
    else
    {
        v = 1.0;
    }

    v *= vMax;
    std::cout << ", count: " << count << ", ";
    std::cout << "distanceMax: " << distanceMax << " ,";
    std::ofstream logFile("cout.txt", std::ios::app);
    logFile << ", count: " << count << ", ";
    logFile << "distanceMax: " << distanceMax << " ,";
    return v;
}

static double getVRefWithoutCount(std::vector<robos::Point2d> path_points, const robos::Pose2D& robot_pose, const double vMax, const std::vector<robos::Pose2D> pathsWithAngle, const int nearst_index)
{
    double v;
    double distance0 = 4;
    double distance = 0.0;
    double distanceMax = 0.0;

    int adaptiveV = 0;

    if(adaptiveV)
    {
        if(!curve)
        {
            if(distanceMax > distance0)
            {
                if(distance < (distance0 / 2))
                {
                    v = 0.5 * (1 - ((2 * distance / distance0) - 1) * ((2 * distance / distance0) - 1)) + 0.5;
                }
                else if(distanceMax - distance < (distance0 / 2))
                {
                    v = 0.5 * (1 - ((2 * (distanceMax - distance) / distance0) - 1) * ((2 * (distanceMax - distance) / distance0) - 1)) + 0.5;
                }
                else
                {
                    v = 1.0;
                }
            }
            else if(distanceMax > 2.5)
            {
                v = 0.5 * (1 - ((2 * distance / distanceMax) - 1) * ((2 * distance / distanceMax) - 1)) + 0.5;
            }
            else
            {
                v = 0.5;
            }
        }
        else
        {
            v = 0.5;
        }
    }
    else
    {
        v = 1.0;
    }

    v *= vMax;
    std::cout << ", count: " << count << ", ";
    std::cout << "distanceMax: " << distanceMax << " ,";
    std::ofstream logFile("cout.txt", std::ios::app);
    logFile << ", count: " << count << ", ";
    logFile << "distanceMax: " << distanceMax << " ,";
    return v;
}

// 判断队列是否满足特定条件的函数
static bool isSpecialOrderValid(const std::queue<double>& q, double threshold)
{
    // 确保队列恰好有30个元素
    if (q.size() != 30)
    {
        return false;
    }

    std::queue<double> tempQueue = q; // 复制队列以保持原队列不变
    double sumFirst10 = 0, sumMiddle10 = 0, sumLast10 = 0; // 分别存储三组10个元素的和
    double current = 0; // 当前元素值

    // 计算最早入队的10个元素的总和
    for (int i = 0; i < 10; ++i)
    {
        current = tempQueue.front();
        sumFirst10 += current;
        tempQueue.pop();
    }

    // 计算中间入队的10个元素的总和
    for (int i = 0; i < 10; ++i)
    {
        current = tempQueue.front();
        sumMiddle10 += current;
        tempQueue.pop();
    }

    // 计算最后入队的10个元素的总和
    for (int i = 0; i < 10; ++i)
    {
        current = tempQueue.front();
        sumLast10 += current;
        tempQueue.pop();
    }

    // 计算三组元素的平均值
    double avgFirst10 = sumFirst10 / 10;
    double avgMiddle10 = sumMiddle10 / 10;
    double avgLast10 = sumLast10 / 10;

    // 检查是否满足平均值的条件以及最后一个元素是否大于阈值
    return (avgLast10 > avgMiddle10) && (avgMiddle10 > avgFirst10) && (current > threshold) && (avgLast10 - avgMiddle10 > 0.01) && (avgMiddle10 - avgFirst10 > 0.01) && (avgLast10 - avgMiddle10 > avgMiddle10 - avgFirst10);
}

static bool isLastTenGreaterThanMiddleTen(const std::queue<double>& q)
{
    if (q.size() != 30)
    {
        return false; // 确保队列恰好有30个元素
    }

    std::queue<double> tempQueue = q; // 复制队列以保持原队列不变
    double sumMiddle10 = 0, sumLast10 = 0; // 分别存储中间和最后10个元素的和

    // 跳过最早入队的10个元素
    for (int i = 0; i < 10; ++i)
    {
        tempQueue.pop();
    }

    // 计算中间入队的10个元素的总和
    for (int i = 0; i < 10; ++i)
    {
        sumMiddle10 += tempQueue.front();
        tempQueue.pop();
    }

    // 计算最后入队的10个元素的总和
    for (int i = 0; i < 10; ++i)
    {
        sumLast10 += tempQueue.front();
        tempQueue.pop();
    }

    // 计算并比较平均值
    return (sumLast10 / 10) > (sumMiddle10 / 10);
}

static bool isMonotonicallyIncreasingAndLastGreaterThan(const std::queue<double>& q, double threshold)
{
    if (q.empty())
    {
        return false; // 如果队列是空的，则直接返回false
    }

    std::queue<double> tempQueue = q; // 复制队列，以保持原队列不变
    double prev = tempQueue.front(); // 获取第一个元素
    tempQueue.pop(); // 移除第一个元素（在副本中）

    while (!tempQueue.empty())
    {
        double current = tempQueue.front();
        tempQueue.pop();

        if (current < prev)
        {
            return false; // 如果发现任何不是递增的情况，则返回false
        }

        prev = current; // 更新prev以用于下一次比较
    }

    return prev > threshold; // 检查最后一个元素是否大于给定的阈值
}

static bool lastGreaterThan(const std::queue<double>& q, double threshold)
{
    if (q.empty())
    {
        return false; // 如果队列是空的，则直接返回false
    }

    std::queue<double> tempQueue = q; // 复制队列，以保持原队列不变

    double prev;
    while (!tempQueue.empty())
    {
        double current = tempQueue.front();
        tempQueue.pop();
        prev = current;
    }

    return prev > threshold; // 检查最后一个元素是否大于给定的阈值
}

static bool compareLastTwoElements(const std::queue<double>& q)
{
    if (q.size() < 2)
    {
        std::cerr << "队列中的元素少于两个，无法比较。" << std::endl;
        return false;
    }

    std::vector<double> temp;
    std::queue<double> copy = q; // 复制队列以保持原队列不变

    // 将队列元素复制到vector中
    while (!copy.empty())
    {
        temp.push_back(copy.front());
        copy.pop();
    }

    // 比较vector中的最后两个元素，如果最后一个元素更大，说明误差依然在变大，也就是说，return true意味着误差还在变大
    return temp[temp.size() - 1] > temp[temp.size() - 2];
}

static int keep = 0;
static int keep_direction1 = 0;
static int keep_direction2 = 0;
static bool reason1 = false;
static bool reason2 = false;

// 检查队列中的所有元素是否都等于1.0
static bool areAllElementsOne(const std::queue<double>& q)
{
    std::queue<double> tempQueue = q; // 复制队列以保持原队列不变

    while (!tempQueue.empty())
    {
        if (tempQueue.front() != 1.0)
        {
            return false; // 发现不等于1.0的元素，立即返回false
        }
        tempQueue.pop(); // 移除已检查的元素
    }

    return true; // 遍历完整个队列，所有元素都等于1.0
}

static double sumOfQueue(const std::queue<double>& q)
{
    std::queue<double> tempQueue = q; // 创建队列的副本以保持原队列不变
    double sum = 0; // 用于存储和的变量

    // 遍历队列中的所有元素，累加它们的值
    while (!tempQueue.empty())
    {
        sum += tempQueue.front(); // 将队首元素的值加到sum上
        tempQueue.pop(); // 移除队首元素
    }

    return sum; // 返回总和
}

static double getDeltaMin(double deltaRef, double deltaMax, double euclideanDistance, std::queue<double> distances, std::queue<double> deltaPhis, double direction5, double direction30, std::queue<double> distances30, std::queue<double> deltaPhis30)
{
    double threshold1 = 0.03;
    double threshold2 = 0.1;
    double threshold1_30 = 0.1;
    double threshold2_30 = 0.15;
    bool flag1 = isMonotonicallyIncreasingAndLastGreaterThan(distances, threshold1) && isMonotonicallyIncreasingAndLastGreaterThan(deltaPhis, -1.0);
    bool flag2 = isSpecialOrderValid(distances30, threshold1_30) && isSpecialOrderValid(deltaPhis30, -1.0);
    if((flag1 || flag2) && !areAllElementsOne(distances))
    {
        if(!keep)
        {
            keep = 6;
            if(flag1)
            {
                reason1 = true;
            }
            else
            {
                reason2 = false;
            }
        }
        if(((direction5 < 0.0 && flag1) || (direction30 < 0.0 && flag2)) && !keep_direction2)
        {
            keep_direction1 = 1;
            return 100.0;
        }
    }
    bool flag3 = compareLastTwoElements(distances) && !isMonotonicallyIncreasingAndLastGreaterThan(deltaPhis, -1.0) && !lastGreaterThan(distances, threshold2);
    bool flag4 = isLastTenGreaterThanMiddleTen(distances30) && !isSpecialOrderValid(deltaPhis30, -1.0) && !isSpecialOrderValid(distances30, threshold2_30);
    if(keep == 1 && keep_direction1)
    {
        if((reason1 && flag3) || (reason2 && flag4))
        {
            return deltaRef;
        }
        else if((flag1 || lastGreaterThan(distances, threshold2)) || (flag2 || lastGreaterThan(distances30, threshold2_30)))
        {
            return 100.0;
        }
        else
        {
            keep--;
            keep_direction1 = 0;
            reason1 = false;
            reason2 = false;
        }
    }

    if(keep > 1)
    {
        if(keep_direction1)
        {
            return 100.0;
        }
    }
    else
    {
        keep_direction1 = 0;
    }
    double flag = 0.0;
    flag = (euclideanDistance < 0.5) ? (2 * euclideanDistance) : 1.0;
    if(euclideanDistance > 1.0)
    {
        flag = euclideanDistance;
    }
    if(flag < 0.2)
    {
        flag = 0.2;
    }

    double d_deltaMin;
    if(deltaRef == 0)
    {
        return -0.0873 * flag;
    }
    else if(deltaRef > 0)
    {
        d_deltaMin = (deltaMax < deltaRef) ? (deltaMax - 0.0873 * 1.5 * flag) : (deltaRef - 0.0873 * 1.5 * flag);
        return d_deltaMin;
    }
    else if (deltaRef < 0)
    {
        d_deltaMin = ((- deltaMax) > (deltaRef - 0.0873 * 1.5 * flag)) ? (- deltaMax) : (deltaRef - 0.0873 * 1.5 * flag);
        return d_deltaMin;
    }
}

static double getDeltaMax(double deltaRef, double deltaMax, double euclideanDistance, std::queue<double> distances, std::queue<double> deltaPhis, double direction5, double direction30, std::queue<double> distances30, std::queue<double> deltaPhis30)
{
    double threshold1 = 0.03;
    double threshold2 = 0.1;
    double threshold1_30 = 0.1;
    double threshold2_30 = 0.2;
    // 这个if是触发器条件，同时满足的情况
    bool flag1 = isMonotonicallyIncreasingAndLastGreaterThan(distances, threshold1) && isMonotonicallyIncreasingAndLastGreaterThan(deltaPhis, -1.0);
    bool flag2 = isSpecialOrderValid(distances30, threshold1_30) && isSpecialOrderValid(deltaPhis30, -1.0);
    if((flag1 || flag2) && !areAllElementsOne(distances))
    {
        if(!keep)
        {
            keep = 6;
            if(flag1)
            {
                reason1 = true;
            }
            else
            {
                reason2 = false;
            }
        }
        if(((direction5 > 0.0 && flag1) || (direction30 > 0 && flag2)) && !keep_direction1)
        {
            keep_direction2 = 1;
            return 100.0;
        }
    }
    bool flag3 = compareLastTwoElements(distances)  && !isMonotonicallyIncreasingAndLastGreaterThan(deltaPhis, -1.0) && !lastGreaterThan(distances, threshold2);
    bool flag4 = isLastTenGreaterThanMiddleTen(distances30) && !isSpecialOrderValid(deltaPhis30, -1.0) && !isSpecialOrderValid(distances30, threshold2_30);
    if(keep == 1 && keep_direction2)
    {
        if((reason1 && flag3) || (reason2 && flag4))
        {
            return deltaRef;
        }
        else if((flag1 || lastGreaterThan(distances, threshold2)) || (flag2 || lastGreaterThan(distances30, threshold2_30)))
        {
            return 100.0;
        }
        else
        {
            keep--;
            keep_direction2 = 0;
            reason1 = false;
            reason2 = false;
        }
    }

    if(keep > 1)
    {
        keep--;
        if(keep_direction2)
        {
            return 100.0;
        }
    }
    else
    {
        keep_direction2 = 0;
    }
    double flag = 0.0;
    flag = (euclideanDistance < 0.5) ? (2 * euclideanDistance) : 1.0;
    if(euclideanDistance > 1.0)
    {
        flag = euclideanDistance;
    }
    if(flag < 0.2)
    {
        flag = 0.2;
    }
    std::cout << ", flag: " << flag << "";

    double d_deltaMax;
    if(deltaRef == 0)
    {
        return 0.0873 * flag;
    }
    else if(deltaRef > 0)
    {
        d_deltaMax = (deltaMax < (deltaRef + 0.0873 * 1.5 * flag)) ? deltaMax : (deltaRef + 0.0873 * 1.5 * flag);
        return d_deltaMax;
    }
    else if (deltaRef < 0)
    {
        d_deltaMax = (- deltaMax > deltaRef) ? (- deltaMax + 0.0873 * 1.5 * flag) : (deltaRef + 0.0873 * 1.5 * flag);
        return d_deltaMax;
    }
}

static double getEuclideanDistance(const robos::Pose2D& robot_pose, const robos::Pose2D& robot_pose_Ref)
{
    double euclideanDistance = PLOT_DIS(robot_pose.x, robot_pose.y, robot_pose_Ref.x, robot_pose_Ref.y);
    return euclideanDistance;
}

static double getd(robos::Pose2D curpose, int nearst_index, std::vector<robos::Point2d> paths)
{
    int start_index = 0;
    if(nearst_index - 1 > start_index)
    {
        start_index = nearst_index - 1;
    }
    int end_index = paths.size();
    if(nearst_index + 1 < paths.size())
    {
        end_index = nearst_index + 1;
    }
    double x0 = paths[nearst_index].x;
    double y0 = paths[nearst_index].y;
    double x1 = paths[start_index].x;
    double y1 = paths[start_index].y;
    double x2 = paths[end_index].x;
    double y2 = paths[end_index].y;
    double x = curpose.x;
    double y = curpose.y;

    double A, B, C;

    // 计算斜率
    double dx = x2 - x1;
    double dy = y2 - y1;

    if (dx == 0)
    {
        // 斜率无穷大的情况，直线方程为 x = x0
        A = 1;
        B = 0;
        C = -x0;
    }
    else
    {
        double k = dy / dx;

        if (std::abs(k) <= 1)
        {
            // |k| <= 1: 使用 y = kx + b 形式，转化为 kx - y + b = 0
            double b = y0 - k * x0;
            A = k;
            B = -1;
            C = b;
        }
        else
        {
            // |k| > 1: 使用 x = ky + b 形式，转化为 x - ky - b = 0
            double b = x0 - (1/k) * y0;
            A = 1;
            B = -1/k;
            C = -b;
        }
    }

    // 使用点到直线距离公式
    double distance = std::abs(A * x + B * y + C) / std::sqrt(A * A + B * B);
    return distance;
}
