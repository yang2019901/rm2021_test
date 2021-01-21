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
#include <chrono>

using namespace std;
using namespace cv;
using namespace chrono;

namespace millhiter{

const bool BLUE = false;
const bool RED = true;

const bool DEG = false;
const bool RAD = true;

const int CONSTMODE = 0;   // 小能量机关模式
const int SINMODE = 1;     // 大能量机关模式
const int CONSTSPEED = 60; // 小能量机关模式下，机关旋转的角速度。单位: deg/s

const int UNKNOWN = 0;
const int CLOCKWISE = 1; // 注意方向，由于图片坐标系的x轴不变而y轴反向，故在极坐标系下参考方向也反向，变为顺时针。
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

#define TIMING true
#define DISTANCE(p1, p2) sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2))

// NOTE: ROI算法的前提是相机参考系与大符参考系完全相对静止，这就要求不能使用安装在枪管上的相机，不然图片会随之而动，导致ROI设置失败
// TODO: 增加斜侧向打符的功能，主要用于干扰对方（想法：仿射变换 or 透视变换）

class MillHiter
{
protected:
    Rect _roi;                // 矩形区域 ROI （如果已经找到的话）
    Point2f _centerR;         // 中心点 R 的坐标 （如果已经找到的话）
    vector<Point2f> _sampleR; // 取“聚点”作为大风车中心点的坐标
    vector<float> _angle;     // 记录前几帧扇叶的角度，以便计算扇叶的旋转方向
    bool _roiAvail;           // ROI是否有效（找到）
    bool _centerRAvail;       // 中心点R坐标是否有效（找到）
    bool _colorFlag;          // 要击打的能量机关的颜色
    int _spinDir;             // 能量机关旋转的方向

public:
    /* 取消默认构造函数，如果没有colorFlag（也即不输入打哪种颜色的装甲），就报错，终止运行 */
    // MillHiter() : _roiAvail(false), _centerRAvail(false), _colorFlag(BLUE), _spinDir(UNKNOWN){};
    MillHiter(bool colorFlag) : _colorFlag(colorFlag), _roiAvail(false), _centerRAvail(false), _spinDir(UNKNOWN){};
    MillHiter(Rect roi, Point2f centerR, bool colorFlag) : _roi(roi), _centerR(centerR), _roiAvail(true), _centerRAvail(true), _colorFlag(colorFlag), _spinDir(UNKNOWN){};

    void setColor(bool colorFlag);
    bool init(Mat src, int mode = 1, uint dotSampleSize = 10, double DistanceErr = 7.0, double nearbyPercentage = 0.8, uint angleSampleSize = 5);
    bool targetDetect(Mat roi, RotatedRect &target, int mode);
    bool targetLock(Mat src, Point2f &aim, int mode = 0, int SampleSize = 10, double DistanceErr = 7.0, double nearbyPercentage = 0.8);
    bool findMillCenter(Mat src, Point2f &center) const;
    double getAngle(const Point2f &center, const Point2f &pos, bool unit = RAD) const;
    double getAngularSpeed(const Point2f &center, const Point2f &nowPos, const Point2f &lastPos, time_t dT, bool unit = RAD) const;
    void trimRegion(Mat src, Rect &region) const;
    bool predictIn(const Point2f &prePos, Point2f &postPos, double dt, int mode = CONSTMODE); // to predict prePos's position in dt ms

    // 与实现roi设置有关的函数
    RotatedRect armorDetect(Mat src) const;
    Rect centerRoi(Mat src, const Point2f &center);
};

}

#endif