#include <iostream>
#include <cmath>
#include <random>
#include "motionModel.hh"
using namespace std;


//construcor initializing the tuning paramters, starting from small uniform values
MotionModel::MotionModel()
{
            alpha1=0.1;
            alpha2=0.1;
            alpha3=0.1;
            alpha4=0.1;
}

vector<double> MotionModel::update(vector<double> u_t0, vector<double> u_t1, vector<double> x_t0)
{
    double del_rot1, del_trans, del_rot2, ddash_rot1, ddash_trans, ddash_rot2, std1, std2, std3;


    //Simple trignometric operations to calculate the robots displacement <s,s'>
    del_rot1 = atan2(u_t1[1] - u_t0[1], u_t1[0] - u_t0[0]) - u_t0[2];
    del_trans = sqrt(pow((u_t0[0]-u_t1[0]),2)+ pow((u_t0[1]-u_t1[1]),2));
    del_rot2= u_t1[2]- u_t0[2]- del_rot1;

    //standard deviation for each parmeter used in the next segment

    std1 = alpha1*del_rot1 + alpha2*del_trans;
    std2 = alpha3*del_trans + alpha4*(del_rot1 + del_rot2);
    std3 = alpha1*del_rot2 + alpha2*del_trans;

    // Gaussian noise distribution with zero mean and std modelling the noise of each parameter
    normal_distribution<double> noise_rot1{0, std1};
    normal_distribution<double> noise_trans{0, std2};
    normal_distribution<double> noise_rot2{0, std3};
    default_random_engine generator;

    ddash_rot1 = del_rot1 - noise_rot1(generator);
    ddash_trans = del_trans - noise_trans(generator);
    ddash_rot2 = del_trans - noise_rot2(generator);

    // converting parameters from the robot's coordinate frame to the world coordinate frame, again by simple trignometric calculations. 
    vector<double> xt(3, 0.0);
    xt[0] = x_t0[0] + ddash_trans* cos(x_t0[2] + ddash_rot1);
    xt[1] = x_t0[1] + ddash_trans* sin(x_t0[2]+ ddash_rot1);
    xt[2] = x_t0[2] + ddash_rot1 + ddash_rot2;
    
    return xt;
    
}
