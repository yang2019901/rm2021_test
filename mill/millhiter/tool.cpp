#include "tool.hpp"

const double
t2omega(const double t)
{
    return A * sin(B * t) + C;
}


const double
omega2t(const double omega_0, const double omega_1)
{
    /* 反解当前的时间估计，这边要注意，因为omega_1有误差，所以可能会溢出变成nan */
    double t = asin( (omega_1 - C) / A ) / B;
    double tendency = omega_1 - omega_0;
    /* 位于正半轴 */
    if (omega_1 > C)
    {
        if (tendency > 0)
            return t;
        else
            return 2 * t_1 - t;
    }
    /* 位于负半轴 */
    else
    {
        if (tendency < 0)
            return 2 * t_1 - t;
        else
            return t + 4 * t_1;
    }
}


const bool
is_omega_valid(const double omega)
{
    return !(abs((omega - C) / A) >= 1);
}


const double
predict_angle(const double init_angle, const double beta, const double t_hat, const double delta_t, const double alpha)
{
    return init_angle + beta + (A / B) * (cos(B * (t_hat + alpha)) - cos(B * (t_hat + delta_t + alpha))) + C * delta_t;
}


const double
get_gradient_alpha(const double angle_observe, const double angle_predict, const double t_hat, const double delta_t, const double alpha)
{
    return (angle_predict - angle_observe) *
           A * (sin(B * (t_hat + delta_t + alpha)) - sin(B * (t_hat + alpha)));
}


const double
get_gradient_beta(const double angle_observe, const double angle_predict)
{
    return angle_predict - angle_observe;
}


const double
t2theta(const double theta_0, const double t)
{
    return theta_0 + (A / B) * (1 - cos(B * t)) + C * t;
}



