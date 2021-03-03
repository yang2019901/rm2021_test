/* 
*  namespace "runehiter" mainly contains RuneHiter class and designed to fulfill rune-hitting ability as its name goes.
*  It is deprived from Rune.hpp, thus independent, and focused on rune-hitting, the ability part.
*  Due to its independency, it can be relatively easy to add functions to this class and namespace.
*/

#ifndef RUNEHITER_HPP
#define RUNEHITER_HPP
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

using namespace Sine;
/* +++++++++++++ */

namespace runehiter {

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

/** @brief: orthogonal right-hand coordinate system (unit: meter):
 *              z axis: direction of gravity;
 *              x axis: projection of the heading of the camera (vertical to z axis)\
 *              y axis: direction defined by x&z axses and right-hand principle
 *          polar system (unit: rad):
 *              polar axis of Yaw: x axis;  plane of Yaw: xOy
 *              polar axis of Pitch: x axis;    plane of Pitch: xOz 
 */
class GimbalController
{
protected:
    float _v;   // v0, the initial velocity of the projectile
    float _k;   // as in `f = kv^2`
    float _m;   // mass of projectile 
    static constexpr float g = 9.80; // gravity accelaration

public:
    GimbalController(void):_v(0), _k(0), _m(0) {};
    GimbalController(float v, float k, float m):_v(v), _k(k), _m(m) {};
    void init(float v, float k, float m);
    float bulletModel(float horizontal_distance, float angle) const;
    float getPitch(float horizontal_distance, float vertical_distance, double err = 1E-4) const;
    void transform(Point3f targetPos, float& yaw_dst, float& pitch_dst) const;
};

// NOTE: ROI算法的前提是相机参考系与大符参考系完全相对静止，这就要求不能使用安装在枪管上的相机，不然图片会随之而动，导致ROI设置失败
// TODO: 增加斜侧向打符的功能，主要用于干扰对方（想法：仿射变换 or 透视变换）
class RuneHiter
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
    Sine::SinePredictor predictor; // 预测器，不写成类的话很麻烦
    size_t frame_counter;         // 记录参数修正次数
    /* ++++++++++++ */

/* ++++++++++++ */
public:
    GimbalController gimbal;
/* ++++++++++++ */

public:
    /* 取消默认构造函数，如果没有colorFlag（也即不输入打哪种颜色的装甲），就报错，终止运行 */
    RuneHiter(bool colorFlag)
    : _colorFlag(colorFlag), _roiAvail(false), _centerRAvail(false), _spinDir(UNKNOWN), frame_counter(0)
    {
        /* ++++++++++++++++++++ */
        this->predictor.init_filter_variance(0.1, 0.01);
        /* ++++++++++++++++++++ */
    }
    RuneHiter(Rect roi, Point2f centerR, bool colorFlag)
    : _roi(roi), _centerR(centerR), _roiAvail(true), _centerRAvail(true), _colorFlag(colorFlag), _spinDir(UNKNOWN), frame_counter(0)
    {
        /* ++++++++++++++++++++ */
        this->predictor.init_filter_variance(0.1, 0.01);
        /* ++++++++++++++++++++ */
    };
    RuneHiter(bool colorFlag, float v, float k, float m)
    : _colorFlag(colorFlag), _roiAvail(false), _centerRAvail(false), _spinDir(UNKNOWN), frame_counter(0), gimbal(v, k, m)
    {
        this->predictor.init_filter_variance(0.1, 0.01);
    };
    // 实现初始化功能的函数，初始化完成后，大风车的中心_centerR，大风车的旋转方向_spinDir和ROI区域_roi确定，而大能量机关的旋转方向不能确定，需要另行初始化！
    bool init(Mat src, int mode = 1, uint dotSampleSize = 5, double DistanceErr = 7.0, double nearbyPercentage = 0.8, uint angleSampleSize = 5);

    // 实现预测功能的两个函数
    bool predictConstSpeed(const Point2f &nowPos, Point2f &predPos, double dt);
    /* 传入现在的点坐标，以及预测间隔gap(s)，结果存入predPos */
    bool predictSineSpeed(const Point2f &nowPos, Point2f &predPos, double gap);
    
    // 用于更改能量机关的颜色信息
    void setColor(bool colorFlag);

    // 获取待击打扇叶的中心。成功则返回true，并把中心坐标存到aim中
    bool targetLock(Mat src, Point2f &aim, int mode = 0, int SampleSize = 10, double DistanceErr = 7.0, double nearbyPercentage = 0.8);


    /** tragically, if the camera is dynamic relative to the rune, the following functions may be of help */

    // 初始化函数，专用于能量机关在图像中平移的情况（aka，相机与能量机关有相对运动），此时，只初始化旋转方向
    bool init2(Mat src, uint angleSampleSize = 5, int mode = 0);

    // 获取待击打扇叶的中心。专用于能量机关在图像中平移的情况。成功则返回true，并把中心坐标存到aim中
    bool targetLock2(Mat src, Point2f &aim, int mode = 0);

/* 这里的函数不对外开放，只是类中其它成员函数实现的工具函数 */
protected:
    bool targetDetect(Mat roi, RotatedRect &target, int mode);
    
    double getAngle(const Point2f &center, const Point2f &pos, bool unit = RAD) const;
    
    double getAngularSpeed(const Point2f &center, const Point2f &nowPos, const Point2f &lastPos, time_t dT, bool unit = RAD) const;

    /* 与实现roi设置有关的函数 */
    bool findRuneCenter(Mat src, Point2f &center) const;
    RotatedRect armorDetect(Mat src) const;
    Rect getRoi(Mat src, const Point2f &center);  // get ROI by center position of the rune
    void trimRegion(Mat src, Rect &region) const;
};

}

#endif
