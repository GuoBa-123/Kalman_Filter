#include "matrix.h"

class Kalman_Filter{
public:
int n; // kalman维数n，待使用
math::matrix<2,2>A;//状态转移矩阵
math::matrix<2,1>K;//卡尔曼增益矩阵
math::matrix<2,2>P;//误差协方差矩阵
math::matrix<2,1>X = {0,0};//初始矩阵
math::matrix<2,2>Q;//过程噪声协方差矩阵
math::matrix<1,1>R;//测量噪声协方差矩阵
math::matrix<1,2>H = {1,0};//状态观测矩阵
math::matrix<1,1>z;//观测值
math::matrix<2,1>kf_out;//输出值
void Kalman_init(Kalman_Filter*kf);
void Kalman_update(math::matrix<1,1> z);
};     