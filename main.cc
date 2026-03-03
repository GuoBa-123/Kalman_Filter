#include "KalmanFilter.h"
#include <iostream> 
using namespace std; 

int main() {
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
    return 0;
}