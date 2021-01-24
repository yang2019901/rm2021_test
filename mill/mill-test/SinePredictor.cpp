#include "SinePredictor.hpp"

// #define DEBUG

#ifdef DEBUG
    #include <iostream>
    using namespace std;
#endif

namespace Sine {

void
SinePredictor::feed_angle(const double angle, const Sys_time sys_time)
{
    /* 判断当前是否在收集角度 */
    if (this->is_receving)
    {
        /* 存储 */
        angles[index] = angle;
        times[index] = sys_time;
        /* 如果收集了4个，就开始估计方程 */
        if (index == 3)
        {
            /* 估计omega */
            double delta_t_0 = get_delta_t(times[1], times[0]);
            double delta_t_1 = get_delta_t(times[3], times[2]);
            double omega_0 = (angles[1] - angles[0]) / delta_t_0;
            double omega_1 = (angles[3] - angles[2]) / delta_t_1;

            /* 保证omega合法 */
            if (!is_omega_valid(omega_1))
                /* 重新获取角度 */
                index = 0;
            
            /* 计算t_hat */
            this->t_hat = omega2t(omega_0, omega_1);
            
            /* 记录当前装甲板角度 */
            this->init_angle = angles[3];
            
            /* 记录init_angle对应的系统时间 */
            this->init_sys_time = times[3];
            
            /* 初始化滤波器 */
            init_filter(init_angle, 
                        omega_1, 
                        get_delta_t(times[3], times[2]),
                        this->variance_angle,
                        this->variance_frame_gap
                        );
        
        #ifdef DEBUG
            cout << "Predictor Initialization: " << endl;
            cout << "   omega_0 : " << omega_0 << endl;
            cout << "   omega_1 : " << omega_1 << endl;
            cout << "   t_hat: " << this->t_hat << endl;
            cout << "   delta t: " << get_delta_t(times[3], times[2]) << endl;
            cout << "   init_angle: " << this->init_angle << endl;
            cout << "   init_sys_time: " << this->init_sys_time << endl;
            getchar();
        #endif

            /* 初始化修正方程用的参数alpha, beta */
            this->alpha = 0;
            this->beta = 0;
            this->step = 1.0;

            /* 记录一下最后一帧对应的系统时间 */
            this->last_frame_sys_time = times[3];

            /* 修改模式 */
            this->is_receving = false;
        }
        else
            index++;
    }
    /* 开始修正方程 */
    else
    {
        /* 计算帧的时间差(s) */
        const double fram_gap = get_delta_t(sys_time, this->last_frame_sys_time);

        /* 使用滤波器进行数据融合 */
        const double angle_observe = this->filter.get_best_estimate(angle, fram_gap);

        /* 初始化完毕到现在通过的时间(s) */
        const double delta_t = get_delta_t(sys_time, this->init_sys_time);

        /* 估计angle的值 */
        const double angle_predict = predict_angle(
            this->init_angle, this->beta, this->t_hat, delta_t, this->alpha
        );
        /* TODO: format angle_predict... 这边的估计值不一定在[-pi, pi]里头 */
        /* TODO: 还有卡尔曼滤波预测的时候要格式化一下角度 kalman.cpp: 50 */

    #ifdef DEBUG
        cout << endl;
        cout << "Prediction: " << endl;
        cout << "   frame gap is: " << fram_gap << endl;
        cout << "   angle_observe: " << angle_observe << endl;
        cout << "   delta_t from begin: " << delta_t << endl;
        cout << "   angle_predict: " << angle_predict << endl;
    #endif

        /* 求解梯度 */
        const double grad_alpha = get_gradient_alpha(angle_observe, angle_predict, this->t_hat, delta_t, this->alpha);
        const double grad_beta = get_gradient_beta(angle_observe, angle_predict);

        /* 更新参数 */
        this->alpha += ( -this->step ) * grad_alpha;
        this->beta += ( -this->step ) * grad_beta;

        /* 更新上一帧的系统时间 */
        this->last_frame_sys_time = sys_time;

        /* 更新滤波器的状态转移矩阵 */
        this->filter.update_omega(t2omega(this->t_hat + delta_t + this->alpha));
    }
}


const double
SinePredictor::predict(const Sys_time now, const double gap) const
{
    const double delta_t = get_delta_t(now, this->init_sys_time) + gap;
#ifdef DEBUG
    cout << "delta_t: " << delta_t << endl;
#endif
    return predict_angle(
        this->init_angle, this->beta, this->t_hat, delta_t, this->alpha
    );
}


void
SinePredictor::change_target(const Sys_time sys_time, const double angle)
{
    /* 目标切换可以看做是一次重新估计t_hat，只是不进行修正。
     * 所以只需要重新确定t_hat, init_angle, init_sys_time
     */
#ifdef DEBUG
    cout << "t_hat before: " << this->t_hat << endl;
#endif
    this->t_hat += ((sys_time - this->init_sys_time) / 1e+6);

#ifdef DEBUG
    cout << "t_hat after: " << this->t_hat << endl;
#endif
    /* TODO: this->beta 初始角度的修正要不要加呢，效果差的话再说吧= = */
    this->init_angle = angle;
    this->beta = 0;

#ifdef DEBUG
    cout << "new angle: " << angle << endl;
#endif
    this->init_sys_time = sys_time;
}

}