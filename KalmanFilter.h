#include "matrix.h"

class Kalman_Filter{
public:
int n; // kalman维数n，待使用
math::matrix<2,2>A;//状态转移矩阵
math::matrix<2,1>X = {0,0};//初始矩阵
math::matrix<2,2>Q = {0,0,
                      0,0};
math::matrix<2,2>R;
math::matrix<1,2>H = {0,1};//状态观测矩阵
void Kalman_init();
void Kalman_update();
};