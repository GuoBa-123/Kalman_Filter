#pragma once
#include "matrix.h"
//M:状态量维度，N:观测量维度
template<int M, int N>
class Kalman_Filter{
public:
math::matrix<N,1>z;//观测值
math::matrix<M,1>kf_out;//输出值
void Kalman_init(math::matrix<M,1>X0, math::matrix<M,M> A_, math::matrix<M,M> P0, math::matrix<M,M> Q_, math::matrix<N,N> R_, math::matrix<N,M> H_){
    this->X = X0;
    this->A = A_;
    this->P = P0;
    this->Q = Q_;
    this->R = R_;
    this->H = H_;
}

void Kalman_update(math::matrix<N,1> z){
    // 预测
    X = A * X;
    P = A * P * A.transpose() + Q;

    //更新
    K = P * H.transpose() * (H * P * H.transpose() + R).inv();
    X = X + K * (z - H * X);
    P = (math::matrix<M,M>(1) - K * H) * P;
    kf_out = X;
}

private:
math::matrix<M,M>A;//状态转移矩阵
math::matrix<M,1>K;//卡尔曼增益矩阵
math::matrix<M,M>P;//误差协方差矩阵
math::matrix<M,1>X;//初始矩阵
math::matrix<M,M>Q;//过程噪声协方差矩阵
math::matrix<N,N>R;//测量噪声协方差矩阵
math::matrix<N,M>H;//状态观测矩阵
};