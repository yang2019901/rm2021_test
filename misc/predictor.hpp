/*
 * 这是 TJU Robomasters 上位机源码，未经管理层允许严禁传播给其他人（包括队内以及队外）
 *
 * 该文件包含各种预测模型，并封装到了以抽象类Predictor类为基类的类中
 */

#pragma once

#include <opencv2/opencv.hpp>
#include "util.hpp"

using namespace cv;

typedef struct _prd_path_pt{

    Point3f worldPosition;
    Point2f targetPTZAngle;
    Point2f selfWorldPosition;
    double duration;

    _prd_path_pt(Point3d _worldp,Point2f _tgtAngle,Point2f _sfworldp,float _dur)
    {
        worldPosition = _worldp;
        targetPTZAngle = _tgtAngle;
        selfWorldPosition = _sfworldp;
        duration = _dur;
    }

    _prd_path_pt()
    { }

}PredictionPathPoint;


class Predictor
{
public:
    // 添加一个轨迹点，该函数应该能够自主甄别轨迹点是否与之前追踪的目标一样，如果不是
    // 预测器自动清空历史数据重新开始预测
    virtual void AddPredictPoint(PredictionPathPoint ppp) = 0;

    // 预测一段时间之后目标的位置
    virtual Point3f Predict(double prdTime) = 0;
    
    // 主动清除历史记录
    virtual void ClearHistory() = 0;
};

// 线性预测器，由历史数据给出线性的预测（假设目标匀速直线运动）
class LinearPredictor : public Predictor
{
public:
    static const int HistorySize = 6;

    LinearPredictor(){}

    void AddPredictPoint(PredictionPathPoint ppp)
    {
        if (hisCount && Length(ppp.targetPTZAngle - history[0].targetPTZAngle) > 10)
            ClearHistory();
        if (hisCount < HistorySize)
            hisCount++;
        ShiftData();
        history[0] = ppp;
    }

    Point3f Predict(double prdTime)
    {
        if (!hisCount) return Point3f();
        if (hisCount < 2) return history[0].worldPosition;
        int t = hisCount / 2;
        Point3f v , v_t;
        FOREACH(i,t)
        {
            v_t = history[i].worldPosition - history[i+t].worldPosition;
            float time = 0;
            for (int j=i;j<i+t;j++) time += history[j].duration;
            v += v_t * (1.0 / time);
        }
        v *= (1.0 / t); // 逐差法计算各方向移动速度
        return history[0].worldPosition + v * prdTime;
    }

    void ClearHistory()
    {
        hisCount = 0;
    }

protected:
    PredictionPathPoint history[HistorySize];
    int hisCount = 0;

    void ShiftData()
    {
        for (int i=hisCount -1;i>0;i--)
        {
            history[i] = history[i-1];
        }
    }
};
