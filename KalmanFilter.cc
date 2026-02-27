#include "matrix.h"
#include "KalmanFilter.h"
//TODO:先实现一个二维kalman->扩展维数kalman->在ESP32上跑，加装传感器
void Kalman_Filter::Kalman_init(Kalman_Filter*kf){
}
void Kalman_Filter::Kalman_update(math::matrix<1,1> z){
    // 预测
    X = A * X;
    P = A * P * A.transpose() + Q;

    //更新
    K = P * H.transpose() * (H * P * H.transpose() + R).inv();
    X = X + K * (z - H * X);
    P = (math::matrix<2,2>(1) - K * H) * P;
    kf_out = X(0,0);

}