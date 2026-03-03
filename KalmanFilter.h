#pragma once
#include "matrix.h"
//M:状态量维度，N:观测量维度
template<int M, int N>
class Kalman_Filter{
public:
math::matrix<N,1>z;//观测值
math::matrix<M,1>kf_out;//输出值

bool skip_fun[5]= {false};//是否跳过KF的五式中的某一式，默认不跳过

void Kalman_init(math::matrix<M,1>X0, math::matrix<M,M> F_, math::matrix<M,M> P0, math::matrix<M,M> Q_, math::matrix<N,N> R_, math::matrix<N,M> H_){
    this->X = X0;
    this->F = F_;
    this->P = P0;
    this->Q = Q_;
    this->R = R_;
    this->H = H_;
}

void Kalman_update(math::matrix<N,1> z){
    // 预测
    state_predict();
    P_predict();

    //更新
    K_caculate();
    state_update(z);
    P_update();

    //输出
    kf_out = X;
}

private:
math::matrix<M,M>F;//状态转移矩阵
void(*F_func)(math::matrix<M,1>X);//状态转移函数
math::matrix<M,1>K;//卡尔曼增益矩阵
math::matrix<M,M>P;//误差协方差矩阵
math::matrix<M,1>X;//初始矩阵
math::matrix<M,M>Q;//过程噪声协方差矩阵
math::matrix<N,N>R;//测量噪声协方差矩阵
math::matrix<N,M>H;//状态观测矩阵

//KF的黄金五式
/*1.状态预测*/
void state_predict(){
    if(!skip_fun[0])
    X = F * X;
}
/*2.协方差预测*/
void P_predict(){
    if(!skip_fun[1])
     P = F * P * F.transpose() + Q;
}
/*3.卡尔曼增益计算*/
void K_caculate(){
    if(!skip_fun[2])
    K = P * H.transpose() * (H * P * H.transpose() + R).inv();
}
/*4.数据融合*/
void state_update(math::matrix<N,1> z){
    if(!skip_fun[3])
    X = X + K * (z - H * X);
}
/*5.更新误差协方差矩阵*/
void P_update(){
    if(!skip_fun[4])
    P = (math::matrix<M,M>(1) - K * H) * P;
}

};