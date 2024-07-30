#include "bezier_smoothing.h"

BezierSmoothing::BezierSmoothing()
{
    yanghui_mats = new double*[maxYanghuiMatOrder];//定义杨辉三角动态数组（矩阵），并指定行数为阶数
    for(int i = 0;i < maxYanghuiMatOrder;++i)
    {
        yanghui_mats[i] = new double[maxYanghuiMatOrder - i];//开辟列
    }
    for(int i = 0;i < maxYanghuiMatOrder;++i)
    {
        yanghui_mats[0][i] = 1;
        yanghui_mats[i][0] = 1;
    }
    for(int i = 1;i < maxYanghuiMatOrder - 1;++i)
    {

        for(int j = 1;j < maxYanghuiMatOrder - i;++j)
        {
            yanghui_mats[i][j] = yanghui_mats[i - 1][j] + yanghui_mats[i][j - 1];
        }
    }
}

BezierSmoothing::~BezierSmoothing()
{
    //************删除杨辉三角矩阵*************//
    if (yanghui_mats)
    {
        for (int i = 0; i < maxYanghuiMatOrder; ++i)
        {
            if (yanghui_mats[i])
            {
                delete[] yanghui_mats[i];
                yanghui_mats[i] = nullptr;
            }
        }
        delete[] yanghui_mats;
        yanghui_mats = nullptr;
    }
}
//获取特定行数的杨辉三角结果
std::vector<double> BezierSmoothing::getYanghuiMatPar(int yanghui_mat)
{
    std::vector<double> yanghuiMatPar;
    for(int i = 0;i < yanghui_mat;++i)
    {
        int j = yanghui_mat -1 - i;
        yanghuiMatPar.push_back(yanghui_mats[i][j]);
    }
    return yanghuiMatPar;
}

std::vector<robos::Point2d> BezierSmoothing::getBezier(std::vector<robos::Point2d> input_points)
{
    std::vector<robos::Point2d> smoothing_result;
    if(input_points.size() > maxYanghuiMatOrder || input_points.size() < 2)
    {
        return smoothing_result;
    }
    const int points_size = input_points.size();
    const double besselStep = 1.0 / (double)(points_size - 1);
    std::vector<double> yanghui_matpar = getYanghuiMatPar(points_size);
    double t = 0;
    for(int i = 0;i < points_size;++i)
    {
        robos::Point2d pt = robos::Point2d(0, 0);
        for(int j = 0;j < points_size;++j)
        {
            pt.x += yanghui_matpar[j] * pow(1 - t, points_size - 1 - j)  *  pow(t,j) * input_points[j].x;
            pt.y += yanghui_matpar[j] * pow(1 - t, points_size - 1 - j)  *  pow(t,j) * input_points[j].y;
        }
        t += besselStep;
        smoothing_result.push_back(pt);
    }
    return smoothing_result;
}

std::vector<robos::Point2d> BezierSmoothing::getLocalBezier(std::vector<robos::Point2d> input_points, int cross_length)
{
    if(input_points.size() <= maxYanghuiMatOrder && input_points.size() >= 2)
    {
        return getBezier(input_points);
    }
    else
    {
        int points_size =  input_points.size();
        std::vector<robos::Point2d> smoothing_result;
        if((points_size < 2)  || (cross_length % 2 != 0) || (points_size <= cross_length))
        {
            return smoothing_result;
        }
        else  //waitForSmothingPointCount > 800  && crossLength为偶数 crossLength < waitForSmothingPointCount
        {

            std::vector<robos::Point2d> inputPathPoints_T = input_points;
            std::vector<robos::Point2d> smoothingResult_T;
            std::vector<robos::Point2d> smoothingResult_P;
            std::vector<robos::Point2d>::iterator sp = inputPathPoints_T.begin();
            std::vector<robos::Point2d>::iterator ep = inputPathPoints_T.begin() + maxYanghuiMatOrder;
            int firstProcessFlag = 0;
            do
            {
                smoothingResult_P = std::vector<robos::Point2d>(sp,ep);
                smoothingResult_T = getBezier(smoothingResult_P);
                if(firstProcessFlag == 0)
                {
                    smoothingResult_T.erase(smoothingResult_T.begin() + (maxYanghuiMatOrder - cross_length / 2),\
                                            smoothingResult_T.end());
                    smoothing_result.insert(smoothing_result.end(),smoothingResult_T.begin(),smoothingResult_T.end());
                    firstProcessFlag = 1;
                }
                else if(firstProcessFlag == 1)
                {
                    smoothingResult_T.erase(smoothingResult_T.begin(),smoothingResult_T.begin() + cross_length / 2);
                    smoothingResult_T.erase(smoothingResult_T.end() - +cross_length / 2, smoothingResult_T.end());
                    smoothing_result.insert(smoothing_result.end(),smoothingResult_T.begin(),smoothingResult_T.end());
                }
                else if(firstProcessFlag == 2)
                {
                    smoothingResult_T.erase(smoothingResult_T.begin(),smoothingResult_T.begin() + cross_length / 2);
                    smoothing_result.insert(smoothing_result.end(),smoothingResult_T.begin(),smoothingResult_T.end());
                }
                if(smoothingResult_P.size() == maxYanghuiMatOrder)
                {
                    sp = inputPathPoints_T.begin() + (maxYanghuiMatOrder - cross_length);
                    ep = inputPathPoints_T.end();
                }
                else
                {
                    sp = inputPathPoints_T.begin() + smoothingResult_P.size();
                    ep = inputPathPoints_T.end();
                }

                inputPathPoints_T = std::vector<robos::Point2d>(sp,ep);
                if(inputPathPoints_T.size() / maxYanghuiMatOrder == 0) // < maxYanghuiMatOrder 个
                {
                    sp = inputPathPoints_T.begin();
                    ep = inputPathPoints_T.end();
                    firstProcessFlag = 2;
                }
                else // = maxYanghuiMatOrder 个
                {
                    sp = inputPathPoints_T.begin();
                    ep = inputPathPoints_T.begin() + maxYanghuiMatOrder;
                }

            }while(!inputPathPoints_T.empty());
            return smoothing_result;
        }
    }
}

std::vector<robos::Point2d> BezierSmoothing::getGlobalBezier(std::vector<robos::Point2d> input_points,\
    const size_t& smothing_length,const int& cross_length)
{
    if(input_points.size() <= smothing_length && input_points.size() >= 2)
    {
        return getBezier(input_points);
    }
    else
    {
        int waitForSmothingPointCount =  input_points.size();
        std::vector<robos::Point2d> smoothingResult;
        if(waitForSmothingPointCount < 2  || (cross_length % 2 != 0) ||
            waitForSmothingPointCount <= cross_length)
        {
            return smoothingResult;
        }
        else  //waitForSmothingPointCount > 800  && crossLength为偶数 crossLength < waitForSmothingPointCount
        {
            std::vector<robos::Point2d> inputPathPoints_T = input_points;
            std::vector<robos::Point2d> smoothingResult_T;
            std::vector<robos::Point2d> smoothingResult_P;
            std::vector<robos::Point2d>::iterator sp = inputPathPoints_T.begin();
            std::vector<robos::Point2d>::iterator ep = inputPathPoints_T.begin() + smothing_length;
            int firstProcessFlag = 0;
            do
            {
                smoothingResult_P = std::vector<robos::Point2d>(sp,ep);
                smoothingResult_T = getBezier(smoothingResult_P);
                if(firstProcessFlag == 0)
                {
                    smoothingResult_T.erase(smoothingResult_T.begin() + (smothing_length - cross_length / 2),\
                                            smoothingResult_T.end());
                    smoothingResult.insert(smoothingResult.end(),smoothingResult_T.begin(),smoothingResult_T.end());
                    firstProcessFlag = 1;
                }
                else if(firstProcessFlag == 1)
                {
                    smoothingResult_T.erase(smoothingResult_T.begin(),smoothingResult_T.begin() + cross_length / 2);
                    smoothingResult_T.erase(smoothingResult_T.end() - + cross_length / 2,smoothingResult_T.end());
                    smoothingResult.insert(smoothingResult.end(),smoothingResult_T.begin(),smoothingResult_T.end());
                }
                else if(firstProcessFlag == 2)
                {
                    smoothingResult_T.erase(smoothingResult_T.begin(),smoothingResult_T.begin() + cross_length / 2);
                    smoothingResult.insert(smoothingResult.end(),smoothingResult_T.begin(),smoothingResult_T.end());
                }
                if(smoothingResult_P.size() == smothing_length)
                {
                    sp = inputPathPoints_T.begin() + (smothing_length - cross_length);
                    ep = inputPathPoints_T.end();
                }
                else
                {
                    sp = inputPathPoints_T.begin() + smoothingResult_P.size();
                    ep = inputPathPoints_T.end();
                }

                inputPathPoints_T = std::vector<robos::Point2d>(sp,ep);
                if(inputPathPoints_T.size() / smothing_length == 0) // < maxYanghuiMatOrder 个
                {
                    sp = inputPathPoints_T.begin();
                    ep = inputPathPoints_T.end();
                    firstProcessFlag = 2;
                }
                else // = maxYanghuiMatOrder 个
                {
                    sp = inputPathPoints_T.begin();
                    ep = inputPathPoints_T.begin() + smothing_length;
                }

            }while(!inputPathPoints_T.empty());
            return smoothingResult;
        }
    }
}

std::vector<robos::Point2d> BezierSmoothing::getGlobalBezier(std::vector<robos::Point2d> input_points, 
    double map_resolution)
{
    //smooth 按照分辨率为5cm，平滑长度为400个点，交叉个数为200个点（偶数）等比例设置//
    size_t smothing_length = (int)(map_resolution * 400 / 5);
    size_t cross_length = (int)(smothing_length * 200 /400);
    if(cross_length % 2 == 1)
    {
        cross_length += 1;
    }
    if(input_points.size() <= smothing_length && input_points.size() >= 2)
    {
        return getBezier(input_points);
    }
    else
    {
        size_t waitForSmothingPointCount =  input_points.size();
        std::vector<robos::Point2d> smoothing_result;
        if((waitForSmothingPointCount < 2) || (cross_length % 2 != 0)
            || (waitForSmothingPointCount <= cross_length))
        {
            return smoothing_result;
        }
        else  //waitForSmothingPointCount > 800  && crossLength为偶数 crossLength < waitForSmothingPointCount
        {
            std::vector<robos::Point2d> inputPathPoints_T = input_points;
            std::vector<robos::Point2d> smoothingResult_T;
            std::vector<robos::Point2d> smoothingResult_P;
            std::vector<robos::Point2d>::iterator sp = inputPathPoints_T.begin();
            std::vector<robos::Point2d>::iterator ep = inputPathPoints_T.begin() + smothing_length;
            int firstProcessFlag = 0;
            do
            {
                smoothingResult_P = std::vector<robos::Point2d>(sp,ep);
                smoothingResult_T = getBezier(smoothingResult_P);
                if(firstProcessFlag == 0)
                {
                    smoothingResult_T.erase(smoothingResult_T.begin() + (smothing_length - cross_length / 2),\
                                            smoothingResult_T.end());
                    smoothing_result.insert(smoothing_result.end(),smoothingResult_T.begin(),smoothingResult_T.end());
                    firstProcessFlag = 1;
                }
                else if(firstProcessFlag == 1)
                {
                    smoothingResult_T.erase(smoothingResult_T.begin(),smoothingResult_T.begin() + cross_length / 2);
                    smoothingResult_T.erase(smoothingResult_T.end() - + cross_length / 2,smoothingResult_T.end());
                    smoothing_result.insert(smoothing_result.end(),smoothingResult_T.begin(),smoothingResult_T.end());
                }
                else if(firstProcessFlag == 2)
                {
                    smoothingResult_T.erase(smoothingResult_T.begin(),smoothingResult_T.begin() + cross_length / 2);
                    smoothing_result.insert(smoothing_result.end(),smoothingResult_T.begin(),smoothingResult_T.end());
                }
                if(smoothingResult_P.size() == smothing_length)
                {
                    sp = inputPathPoints_T.begin() + (smothing_length - cross_length);
                    ep = inputPathPoints_T.end();
                }
                else
                {
                    sp = inputPathPoints_T.begin() + smoothingResult_P.size();
                    ep = inputPathPoints_T.end();
                }

                inputPathPoints_T = std::vector<robos::Point2d>(sp,ep);
                if(inputPathPoints_T.size() / smothing_length == 0) // < maxYanghuiMatOrder 个
                {
                    sp = inputPathPoints_T.begin();
                    ep = inputPathPoints_T.end();
                    firstProcessFlag = 2;
                }
                else // = maxYanghuiMatOrder 个
                {
                    sp = inputPathPoints_T.begin();
                    ep = inputPathPoints_T.begin() + smothing_length;
                }

            }while(!inputPathPoints_T.empty());
            return smoothing_result;
        }
    }

}
