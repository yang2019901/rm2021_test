#include "kalman.hpp"


void
Kalman::init_filter(const double theta, const double omega, const double frame_gap, const double variance_theta, const double variance_frame_gap)
{
    /* 初始化状态 */
    this->state << theta, frame_gap;
    /* 初始化转移矩阵中的omega */
    this->omega = omega;
    /* 初始化P矩阵，由于P矩阵会自动修正，所以随便设一个常数 */
    this->P << 0.2, 0,
                 0, 0.1;
    /* 初始化Q矩阵，无外部控制，Q矩阵为0 */
    this->Q << 0, 0,
               0, 0;
    /* 初始化R矩阵 */
    this->R << variance_theta,   0,
               0, variance_frame_gap;
    /* 初始化H矩阵 */
    this->H << 1, 0,
               0, 1;
}

void
Kalman::estimate()
{
    /* 转移矩阵, 不是常数 */
    Matrix<double, 2, 2> A;
    A << 1, this->omega,
         0, 1;
    /* 无外部控制 */
    this->state = A * this->state;
}


const double
Kalman::revise(const double theta_observation, const double frame_gap_observation)
{
    /* 观测向量Z */
    Vector2d Z;
    Z << theta_observation, frame_gap_observation;
    /* 卡尔曼增益 
     * K = P * H^T * (H * P * H^T + R)^(-1)
     */
    Matrix<double, 2, 2> K = this->P * this->H.transpose() *
                                (this->H * this->P * this->H.transpose() + this->R).inverse();

    /* 根据卡尔曼增益修正状态初步估计 */
    /* TODO: format angle. */
    this->state = this->state + K * (Z - this->H * this->state);
    this->P = (MatrixXd::Identity(2, 2) - K * this->H) * this->P;

    /* 返回theta的估计值 */
    return this->state(0);
}


const double
Kalman::get_best_estimate(const double theta, const double frame_gap)
{
    /* 进行初始估计 */
    this->estimate();
    /* 对初步估计进行修正 */
    return this->revise(theta, frame_gap);
}