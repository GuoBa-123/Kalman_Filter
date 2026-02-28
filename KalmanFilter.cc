#include "matrix.h"
#include "KalmanFilter.h"
//TODO:先实现一个二维kalman->扩展维数kalman->在ESP32上跑，加装传感器
// template<int M, int N>
// void Kalman_Filter<M,N>::Kalman_init(math::matrix<M,1> X0, math::matrix<M,M> A, math::matrix<M,M> P0, math::matrix<M,M> Q, math::matrix<N,N> R, math::matrix<N,M> H){
//     this->X = X0;
//     this->A = A;
//     this->P = P0;
//     this->Q = Q;
//     this->R = R;
//     this->H = H;
// }

// template<int M, int N>
// void Kalman_Filter<M,N>::Kalman_update(math::matrix<N,1> z){
//     // 预测
//     X = A * X;
//     P = A * P * A.transpose() + Q;

//     //更新
//     K = P * H.transpose() * (H * P * H.transpose() + R).inv();
//     X = X + K * (z - H * X);
//     P = (math::matrix<M,M>(1) - K * H) * P;
//     kf_out = X;

// }