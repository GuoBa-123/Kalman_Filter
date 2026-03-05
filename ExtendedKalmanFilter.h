#pragma once
#include "matrix.h"
#include <functional>

// M: 状态量维度, N: 观测量维度
template<int M, int N>
class Extended_Kalman_Filter {
public:
    math::matrix<M,1> ekf_out;

    // 非线性状态转移函数 f(X): X_{k} -> X_{k+1}
    std::function<math::matrix<M,1>(const math::matrix<M,1>&)> f_func;
    // 非线性观测函数 h(X): X -> Z
    std::function<math::matrix<N,1>(const math::matrix<M,1>&)> h_func;

    // Jacobian函数（可选）：不设置则自动用数值微分计算
    std::function<math::matrix<M,M>(const math::matrix<M,1>&)> Fj_func;
    std::function<math::matrix<N,M>(const math::matrix<M,1>&)> Hj_func;

    void EKF_init(math::matrix<M,1> X0, math::matrix<M,M> P0,
                  math::matrix<M,M> Q_, math::matrix<N,N> R_) {
        X = X0;
        P = P0;
        Q = Q_;
        R = R_;
    }

    void EKF_update(math::matrix<N,1> z) {
        // 在当前状态处线性化
        math::matrix<M,M> Fj = Fj_func ? Fj_func(X) : numerical_Fj(X);

        // 1. 状态预测
        X = f_func(X);
        // 2. 协方差预测
        P = Fj * P * Fj.transpose() + Q;

        // 在预测状态处线性化观测
        math::matrix<N,M> Hj = Hj_func ? Hj_func(X) : numerical_Hj(X);

        // 3. 卡尔曼增益
        math::matrix<M,N> K = P * Hj.transpose() * (Hj * P * Hj.transpose() + R).inv();
        // 4. 状态更新
        X = X + K * (z - h_func(X));
        // 5. 协方差更新
        P = (math::matrix<M,M>(1) - K * Hj) * P;

        ekf_out = X;
    }

private:
    math::matrix<M,1> X;
    math::matrix<M,M> P;
    math::matrix<M,M> Q;
    math::matrix<N,N> R;

    static constexpr float eps_jac = 1e-5f;

    // 数值Jacobian：对 f 求偏导，列 j 对应对 x_j 的偏导
    math::matrix<M,M> numerical_Fj(const math::matrix<M,1>& x) {
        math::matrix<M,M> Fj;
        for (int j = 0; j < M; j++) {
            math::matrix<M,1> xp = x, xm = x;
            xp(j, 0) += eps_jac;
            xm(j, 0) -= eps_jac;
            auto df = f_func(xp) - f_func(xm);
            for (int i = 0; i < M; i++)
                Fj(i, j) = df(i, 0) / (2.0f * eps_jac);
        }
        return Fj;
    }

    math::matrix<N,M> numerical_Hj(const math::matrix<M,1>& x) {
        math::matrix<N,M> Hj;
        for (int j = 0; j < M; j++) {
            math::matrix<M,1> xp = x, xm = x;
            xp(j, 0) += eps_jac;
            xm(j, 0) -= eps_jac;
            auto dh = h_func(xp) - h_func(xm);
            for (int i = 0; i < N; i++)
                Hj(i, j) = dh(i, 0) / (2.0f * eps_jac);
        }
        return Hj;
    }
};
