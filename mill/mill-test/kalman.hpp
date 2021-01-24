/* 线性代数求解库 */
#include <Eigen/Dense>
using namespace Eigen;

#ifndef KALMAN_H
#define KALMAN_H

class Kalman
{
private:
    /* 状态向量(theta, delta_t)
     * theta   -> 上一状态的角度
     * delta_t -> 预计下一帧到达所需时间
     */
    Vector2d state;
    
    /* 上一个状态的omega */
    double omega;
    
    /* 矩阵H，将状态映射为观测值 */
    Matrix<double, 2, 2> H;

    /* 矩阵P，状态向量的协方差，体现状态转移的不确定性 */
    Matrix<double, 2, 2> P;

    /* 矩阵Q，体现“考虑外部控制u”所带来的不确定性 */
    Matrix<double, 2, 2> Q;

    /* 矩阵R，观测向量Z的协方差，体现观测的不确定性 */
    Matrix<double, 2, 2> R;

    /* 根据上一次的最佳状态对当前状态进行初步估计 */
    void estimate();

    /* 融合传感器的值，对初步估计进行修正 */
    const double revise(const double theta_observation, const double delta_t_observation);

public:
    /* 没什么用的构造函数 */
    Kalman() {};

    /* 初始化滤波器 */
    void init_filter(const double theta, const double omega, const double delta_t, const double variance_theta, const double variance_delta_t);

    /* 获取最佳估计 */
    const double get_best_estimate(const double theta_observation, const double delta_t_observation);

    /* 更新转移矩阵中的omega */
    void update_omega(const double omega) { this->omega = omega; }
};

#endif