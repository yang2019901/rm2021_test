#include "tool.hpp"
#include "kalman.hpp"
#include "sys_time.hpp"

#ifndef SINE_PREDICTOR_H
#define SINE_PREDICTOR_H

namespace Sine {

class SinePredictor
{
private:
    /* 估计方程的各个参数 
     * t_hat: 根据初始omega估计的时间，单位(s)
     * init_angle: 估计t_hat完毕后对应的初始角度，单位(rad)
     * init_sys_time: 估计t_hat完毕后对应的系统时间，单位(us)
     */
    double t_hat;
    double init_angle;
    Sys_time init_sys_time;

    /* 梯度下降参数 
     * alpha: 修正初始时间t_hat
     * beta: 修正初始角度
     * step: 学习率
     */
    double alpha;
    double beta;
    double step;
    
    /* 预测器的参数
     * last_frame_sys_time: 记录上一帧的系统时间，用于计算帧间隔，单位(us)
     * is_receving: 判断预测器是否还在收集图像
     * angles: 存储初始的4个角度的数组
     * index: angles数组的下标
     * times: 存储初始4个角度对应的系统时间
     */
    bool is_receving;
    double angles[4];
    Sys_time times[4];
    size_t index;
    Sys_time last_frame_sys_time;

    /* 这玩意儿专用的超简易滤波器，但应该比opencv自带的快 */
    Kalman filter;
    double variance_angle;
    double variance_frame_gap;

    /* 初始化滤波器 */
    void init_filter(const double angle, const double omega, const double frame_gap, const double variance_angle, const double variance_frame_gap)
    {
        this->filter.init_filter(angle, omega, frame_gap, variance_angle, variance_frame_gap);
    }

public:
    /* constructor */
    SinePredictor()
    : is_receving(true), index(0)
    {}

    /* 初始化滤波器的观测值方差 */
    void init_filter_variance(const double variance_angle, const double variance_frame_gap)
    {
        this->variance_angle = variance_angle;
        this->variance_frame_gap = variance_frame_gap;
    }

    /* 输入识别的装甲板中心角度，以及对应的系统时间(us)，会自动更新参数 */
    void feed_angle(const double angle, const Sys_time sys_time);

    /* 输入系统时间now(us)，以及需要往后预测的时长gap(s)，注意单位是秒!!!!。输出预测的角度。 */
    const double predict(const Sys_time now, const double gap) const;

    /* 判断预测器能否用于预测。is ok ? */
    const bool is_ok() const { return !this->is_receving; }

    /* 用于切换装甲板，具体做法看cpp */
    void change_target(const Sys_time sys_time, const double angle);

    /* 查看修正的参数，测试的时候用的 */
    const double get_alpha() const { return this->alpha; }
    const double get_beta() const { return this->beta; }
};

}

#endif