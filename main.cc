#include "KalmanFilter.h"
#include "ExtendedKalmanFilter.h"
#include <iostream>
#include <cmath>
using namespace std;

int main() {
    // ==================== 原有 KF 测试 ====================
    cout << "========== Kalman Filter Test ==========" << endl;
    Kalman_Filter<2,1> kf;
    //kalman init
    float dt = 2.0f;
    math::matrix<2,1> X0 = {0,0};
    math::matrix<2,2> P0({100,0,0,100});
    math::matrix<2,2> Q({0.01f,0.f,0.f,0.01f});
    math::matrix<1,1> R({0.1f});
    math::matrix<1,2> H = {1,0};
    math::matrix<2,2> F({1.0f,dt,0.0f,1.0f});
    
    kf.Kalman_init(X0, F, P0, Q, R, H);
    float obversation[10] = {0.0, 0.9, 2.1, 3.05, 3.95, 5.02, 5.98, 7.03, 7.97, 9.01};
    for(int i=0;i<10;i++){
        kf.Kalman_update(math::matrix<1,1>(obversation[i]));
        cout<<"observation: "<<obversation[i]<<"  kalman output_1: "<<kf.kf_out(0,0)<<"   kalman output_2: "<<kf.kf_out(1,0)<<endl;
    }

    // ==================== EKF 测试（圆周运动跟踪）====================
    cout << "\n========== Extended Kalman Filter Test ==========" << endl;
    cout << "Tracking a target in circular motion (nonlinear state transition)" << endl;
    
    Extended_Kalman_Filter<4, 2> ekf;
    
    // 初始化参数
    // 状态: [x, y, vx, vy]^T (位置x, 位置y, 速度x, 速度y)
    math::matrix<4,1> X0_ekf = {1.0f, 0.0f, 0.0f, 1.0f};  // 从(1,0)开始，初速度(0,1)
    math::matrix<4,4> P0_ekf({1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1});
    math::matrix<4,4> Q_ekf({0.01f,0.0f,0.0f,0.0f, 0.0f,0.01f,0.0f,0.0f, 0.0f,0.0f,0.001f,0.0f, 0.0f,0.0f,0.0f,0.001f});
    math::matrix<2,2> R_ekf({0.1f,0.0f, 0.0f,0.1f});
    
    ekf.EKF_init(X0_ekf, P0_ekf, Q_ekf, R_ekf);
    
    // 非线性状态转移函数: 匀速圆周运动
    float dt_ekf = 0.1f;
    float omega = 1.0f;  // 角速度
    ekf.f_func = [dt_ekf, omega](const math::matrix<4,1>& X) -> math::matrix<4,1> {
        math::matrix<4,1> X_next;
        float x = X(0,0), y = X(1,0), vx = X(2,0), vy = X(3,0);
        // 位置更新
        X_next(0,0) = x + vx * dt_ekf;
        X_next(1,0) = y + vy * dt_ekf;
        // 速度更新（旋转）
        float cos_wt = cos(omega * dt_ekf);
        float sin_wt = sin(omega * dt_ekf);
        X_next(2,0) = vx * cos_wt - vy * sin_wt;
        X_next(3,0) = vx * sin_wt + vy * cos_wt;
        return X_next;
    };
    
    // 观测函数: 直接观测位置
    ekf.h_func = [](const math::matrix<4,1>& X) -> math::matrix<2,1> {
        math::matrix<2,1> Z;
        Z(0,0) = X(0,0);  // x位置
        Z(1,0) = X(1,0);  // y位置
        return Z;
    };
    
    // 手动提供雅可比矩阵（可选，如果不提供则自动数值计算）
    ekf.Fj_func = [dt_ekf, omega](const math::matrix<4,1>& X) -> math::matrix<4,4> {
        math::matrix<4,4> Fj;
        float cos_wt = cos(omega * dt_ekf);
        float sin_wt = sin(omega * dt_ekf);
        // Fj = [I, dt*I; 0, R(w*dt)]
        Fj(0,0) = 1; Fj(0,1) = 0; Fj(0,2) = dt_ekf; Fj(0,3) = 0;
        Fj(1,0) = 0; Fj(1,1) = 1; Fj(1,2) = 0;      Fj(1,3) = dt_ekf;
        Fj(2,0) = 0; Fj(2,1) = 0; Fj(2,2) = cos_wt; Fj(2,3) = -sin_wt;
        Fj(3,0) = 0; Fj(3,1) = 0; Fj(3,2) = sin_wt; Fj(3,3) = cos_wt;
        return Fj;
    };
    
    ekf.Hj_func = [](const math::matrix<4,1>& X) -> math::matrix<2,4> {
        math::matrix<2,4> Hj;
        // Hj = [I, 0] (只观测位置)
        Hj(0,0) = 1; Hj(0,1) = 0; Hj(0,2) = 0; Hj(0,3) = 0;
        Hj(1,0) = 0; Hj(1,1) = 1; Hj(1,2) = 0; Hj(1,3) = 0;
        return Hj;
    };
    
    // 生成仿真数据（真实圆周轨迹 + 噪声）
    cout << "Step | True X | True Y | Obs X | Obs Y | EKF X | EKF Y | EKF Vx | EKF Vy" << endl;
    cout << "--------------------------------------------------------------------------" << endl;
    
    float true_x = 1.0f, true_y = 0.0f;
    float true_vx = 0.0f, true_vy = 1.0f;
    
    for(int i = 0; i < 20; i++) {
        // 真实状态更新（圆周运动）
        float new_x = true_x + true_vx * dt_ekf;
        float new_y = true_y + true_vy * dt_ekf;
        float cos_wt = cos(omega * dt_ekf);
        float sin_wt = sin(omega * dt_ekf);
        float new_vx = true_vx * cos_wt - true_vy * sin_wt;
        float new_vy = true_vx * sin_wt + true_vy * cos_wt;
        true_x = new_x; true_y = new_y;
        true_vx = new_vx; true_vy = new_vy;
        
        // 添加观测噪声
        float noise_x = 0.05f * ((float)rand() / RAND_MAX - 0.5f);
        float noise_y = 0.05f * ((float)rand() / RAND_MAX - 0.5f);
        math::matrix<2,1> z = {true_x + noise_x, true_y + noise_y};
        
        // EKF更新
        ekf.EKF_update(z);
        
        cout << i << " | " 
             << true_x << ", " << true_y << " | "
             << z(0,0) << ", " << z(1,0) << " | "
             << ekf.ekf_out(0,0) << ", " << ekf.ekf_out(1,0) << " | "
             << ekf.ekf_out(2,0) << ", " << ekf.ekf_out(3,0) << endl;
    }
    
    cout << "\nEKF test completed successfully!" << endl;
    
    return 0;
}
