#ifndef BezierSmoothing_H
#define BezierSmoothing_H
#include <robos/navi_msgs.h>

// 定义杨辉三角最大阶数，即贝塞尔曲线最多的平滑点数，当大于1000时，杨辉三角会出现值溢出的情况
#define maxYanghuiMatOrder 800

class BezierSmoothing
{
public:
    BezierSmoothing();
    ~BezierSmoothing();
    // 对局部路径规划后的点进行平滑，inputPathPoints的长度原则上不超过maxYanghuiMatOrder,平滑时按照inputPathPoints实际点数进行平滑，超过的部分也可处理，按照//
    // crossLength的长度进行处理，由用户指定，这里建议值为400,用户指定时数值必须是偶数且必须小于maxYanghuiMatOrder//
    std::vector<robos::Point2d> getLocalBezier(std::vector<robos::Point2d> input_points, int cross_length = 400);
    // 该函数可对任意长度的点集进行贝塞尔平滑，inputPathPoints 为待平滑点，mapResolution为栅格地图的分辨率，smothingLength为一次平滑的最大长度，由用户指定//
    // ，crossLength为待平滑点数超过用户指定长度后，分段贝塞尔曲线的交叉部分的长度，也由用户自定义，该值不能超过smothingLength的长度且必须设置为偶数//
    std::vector<robos::Point2d> getGlobalBezier(std::vector<robos::Point2d> input_points,
                                                const size_t &smothing_length = 200, const int &cross_length = 100);
    // 该函数可对任意长度的点集进行贝塞尔平滑，inputPathPoints 为待平滑点，mapResolution为栅格地图的分辨率，其与上述函数的区别在于smothingLength和crossLength//
    // 两个参数不再由用户指定，而是系统根据点数、栅格地图分辨率和相关经验值对着两个值进行动态规划，即按照5cm分辨率的情况下，一次平滑最大点数（smothingLength）为400//
    // 交差点数（crossLength）为200的经验值进行平滑，当分辨率变化时，相关参数跟着等比例变化//
    std::vector<robos::Point2d> getGlobalBezier(std::vector<robos::Point2d> input_points, double map_resolution);
    // 通用贝塞尔曲线的获取接口，inputPathPoints必须小于maxYanghuiMatOrder//
    std::vector<robos::Point2d> getBezier(std::vector<robos::Point2d> input_points);

public:
private:
    // 获取杨辉三角指定阶数上的参数//
    std::vector<double> getYanghuiMatPar(int yanghui_mat);

private:
    double **yanghui_mats; // 杨辉三角矩阵，存储800阶杨辉三角所有参数
};

#endif // BEZIERSMOTHING_H
