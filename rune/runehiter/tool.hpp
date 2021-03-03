#include <cmath>

/* 这个头文件主要包含了一些SinePredictor会用到的函数 */

#ifndef TOOL_H
#define TOOL_H


/* 运动参数 */
static const double A = 0.785;
static const double B = 1.884;
static const double C = 1.305;

static const double t_1 = M_PIf64 / (2 * B);
static const double t_2 = -t_1;


/* 工具函数 */

/* 根据已知公式由t(s)计算omega(rad / s) */
const double
t2omega(const double t);


/* 根据收集的两个omega(rad / s)，计算初始t_hat(s) */
/* TODO: 极特殊点会出现bug. 例如端点 */
const double
omega2t(const double omega_0, const double omega_1);


/* 判断差分计算出来的omega是否在合法范围内 */
const bool
is_omega_valid(const double omega);


/* 基于由t_hat确定的omega规律，给定初始角度，积分计算出delta_t时间后的角度
 * init_angle: 初始的角度，对应delta_t = 0时刻
 * beta:       对初始角度的修正
 * t_hat:      用于确定omega规律
 * delta_t:    间隔时间，单位(s)
 * alpha:      时间修正
 */
const double
predict_angle(const double init_angle, const double beta, const double t_hat, const double delta_t, const double alpha);


/* 根据观测值与预测值，计算alpha的梯度 
 * angle_observe: 滤波后的观测值
 * angle_predict: 用上面那个方程估计出来的值
 * t_hat:         确定omega规律
 * delta_t:       系统时间差值，时间(s)，与angle_predict对应的delta_t相同
 * alpha:         时间修正
 */
const double
get_gradient_alpha(const double angle_observe, const double angle_predict, const double t_hat, const double delta_t, const double alpha);


/* 根据观测值与预测值，计算beta的梯度 
 * angle_observe: 滤波后的观测值
 * angle_predict: 估计值
 */
const double
get_gradient_beta(const double angle_observe, const double angle_predict);



/* ********************************* 以下函数用于模拟测试环境 ********************************* */

/* 根据已知公式由t(s)，以及设定的初始角度theta_0(rad)，计算theta(rad) */
const double
t2theta(const double theta_0, const double t);


#endif