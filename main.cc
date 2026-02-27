#include "KalmanFilter.h"
#include <iostream> 
using namespace std; 

int main() {
    Kalman_Filter kf;
    //kalman init
    kf.A(0,0) = 1;
    kf.A(0,1) = 1;
    kf.A(1,0) = 0;
    kf.A(1,1) = 1;
    kf.P(0,0) = 1;
    kf.P(0,1) = 0;
    kf.P(1,0) = 0;
    kf.P(1,1) = 1;
       kf.Q(0,0) = 0.0001;
       kf.Q(0,1) = 0;
       kf.Q(1,0) = 0;
       kf.Q(1,1) = 0.0001;
       kf.R(0,0) = 0.01;
       float obversation[10] = {0.0, 0.9, 2.1, 3.05, 3.95, 5.02, 5.98, 7.03, 7.97, 9.01};
    for(int i=0;i<10;i++){
        kf.Kalman_update(math::matrix<1,1>(obversation[i]));
        cout<<"observation: "<<obversation[i]<<"  kalman output_1: "<<kf.kf_out<<"   kalman output_2: "<<kf.X(1,0)<<endl;
    }
    return 0;
}