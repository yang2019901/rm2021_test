/* 
*  namespace "millhiter" mainly contains MillHiter class and designed to fulfill mill-hitting ability as its name goes.
*  It is deprived from Mill.hpp, thus independent, and focused on mill-hitting, the ability part.
*  Due to its independency, it can be relatively easy to add functions to this class and namespace.
*/

#ifndef MILLHITER_HPP
#define MILLHITER_HPP
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <chrono> // <- this head file is just to test time cost. removable when released 

using namespace std;
using namespace cv;
using namespace chrono;

/* +++++++++++++ */
#include "SinePredictor.hpp"
/* +++++++++++++ */

namespace millhiter {

/* #ifndef SINE_SPIN_PARAMS
#define SINE_SPIN_PARAMS
    const double a = 0.785;
    const double b = 1.884;
    const double c = 1.305;
    const double A = a / b;
#endif */

const bool BLUE = false;
const bool RED = true;

const bool DEG = false;
const bool RAD = true;

const int CONSTMODE = 0;   // 小能量机关模式
const int SINMODE = 1;     // 大能量机关模式
const int CONSTSPEED = 60; // 小能量机关模式下，机关旋转的角速度。单位: deg/s

const int UNKNOWN = 0;
const int CLOCKWISE = 1;   // 注意方向，由于图片坐标系的x轴不变而y轴反向，故在极坐标系下参考方向也反向，变为顺时针。
const int COUNTERCLOCKWISE = -1;

#ifndef PI
#define PI 3.1415927
#endif

#ifndef RAD2DEG
#define RAD2DEG(rad) ((rad)*180 / PI)
#endif

#ifndef DEG2RAD
#define DEG2RAD(deg) ((deg)*PI / 180)
#endif

#define DISTANCE(p1, p2) sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2))

/* typedef struct
{
    float _phase;
    float _angle;
} Angle_SpinParams; */


// NOTE: ROI算法的前提是相机参考系与大符参考系完全相对静止，这就要求不能使用安装在枪管上的相机，不然图片会随之而动，导致ROI设置失败
// TODO: 增加斜侧向打符的功能，主要用于干扰对方（想法：仿射变换 or 透视变换）

class MillHiter
{
protected:
    // 由核心数据成员和辅助数据成员组成，其中核心数据成员一般会在定义其他的成员函数时用到(比如拟合圆的中心_centerR)
    Rect _roi;                    // 核心数据成员：矩形区域 ROI （如果已经找到的话）
    Point2f _centerR;             // 核心数据成员：中心点 R 的坐标 （如果已经找到的话）
    vector<Point2f> _sampleR;     // 辅助数据成员：大风车中心点的样本集，取其中的“聚点”作为大风车中心点的坐标
    vector<float> _angle;         // 辅助数据成员：记录前几帧扇叶的角度，以便计算扇叶的旋转方向
    bool _roiAvail;               // 辅助数据成员：ROI是否有效（找到）
    bool _centerRAvail;           // 辅助数据成员：中心点R坐标是否有效（找到）
    bool _colorFlag;              // 核心数据成员：要击打的能量机关的颜色
    int _spinDir;                 // 核心数据成员：能量机关旋转的方向
    // Angle_SpinParams _spinParams; // 核心数据成员：大能量机关中，扇叶角度与时间的函数关系中的两个待定参数 theta(t) = -0.785/1.884*cos(1.884*t+_phase) + 1.305*t + _angle，确定这两个参数即可确定大能量机关特定扇叶的运动方程，扇叶切换时，只需更改_spinParams._angle
    
    /* ++++++++++++ */
    Sine::SinePredictor preditor; // 预测器，不写成类的话很麻烦
    size_t frame_counter;         // 记录参数修正次数
    /* ++++++++++++ */

public:
    /* 取消默认构造函数，如果没有colorFlag（也即不输入打哪种颜色的装甲），就报错，终止运行 */
    MillHiter(bool colorFlag)
    : _colorFlag(colorFlag), _roiAvail(false), _centerRAvail(false), _spinDir(UNKNOWN)
    {
        /* ++++++++++++++++++++ */
        this->preditor.init_filter_variance(0.1, 0.01);
        /* ++++++++++++++++++++ */
    }
    MillHiter(Rect roi, Point2f centerR, bool colorFlag)
    : _roi(roi), _centerR(centerR), _roiAvail(true), _centerRAvail(true), _colorFlag(colorFlag), _spinDir(UNKNOWN)
    {
        /* ++++++++++++++++++++ */
        this->preditor.init_filter_variance(0.1, 0.01);
        /* ++++++++++++++++++++ */
    };

    // 实现初始化功能的函数，初始化完成后，大风车的中心_centerR，大风车的旋转方向_spinDir和ROI区域_roi确定，而大能量机关的旋转方向不能确定，需要另行初始化！
    bool init(Mat src, int mode = 1, uint dotSampleSize = 10, double DistanceErr = 7.0, double nearbyPercentage = 0.8, uint angleSampleSize = 5);
    
    // 实现预测功能的两个函数
    bool predictConstSpeed(const Point2f &nowPos, Point2f &predPos, double dt);
    /* 传入现在的点坐标，以及预测间隔gap(s)，结果存入predPos */
    bool predictSineSpeed(const Point2f &nowPos, Point2f &predPos, double gap);
    
    // 用于更改能量机关的颜色信息
    void setColor(bool colorFlag);

    // 获取待击打扇叶的中心。成功则返回true，并把中心坐标存到aim中
    bool targetLock(Mat src, Point2f &aim, int mode = 0, int SampleSize = 10, double DistanceErr = 7.0, double nearbyPercentage = 0.8);

/* 这里的函数不对外开放，只是类中其它成员函数实现的工具函数 */
protected:
    bool targetDetect(Mat roi, RotatedRect &target, int mode);
    double getAngle(const Point2f &center, const Point2f &pos, bool unit = RAD) const;
    double getAngularSpeed(const Point2f &center, const Point2f &nowPos, const Point2f &lastPos, time_t dT, bool unit = RAD) const;
    // 与实现roi设置有关的函数
    bool findMillCenter(Mat src, Point2f &center) const;
    RotatedRect armorDetect(Mat src) const;
    Rect centerRoi(Mat src, const Point2f &center);
    void trimRegion(Mat src, Rect &region) const;
};

}

#endif
