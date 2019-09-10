#include <iostream>
#include <math>
#include "motionModel.hh"
using namespace std;


//construcor initializing the tuning paramters, starting from small uniform values
void MotionModel::MotionModel()
{
            alpha1=0.1;
            alpha2=0.1;
            alpha3=0.1;
            alpha4=0.1;
}

vector<double> MotionModel::update(vector<double> u_t0, vector<double> u_t1, vector<double> x_t0)
{
    double del_rot1, del_trans, del_rot2, ddash_rot1, ddash_trans, ddash_rot2, term1, term2, term3;


    //Simple trignometric operations to calculate the robots displacement <s,s'>
    del_rot1 = arctan2(u_t1[1] - u_t0[1], u_t1[0] - u_t0[0]) - u_t0[2];
    del_trans = sqrt(pow((u_t0[0]-u_t1[0]),2)+ pow((u_t0[1]-u_t1[1]),2));
    del_rot2= u_t1[2]- u_t0[2]- del_rot1;

    //standard deviation for each parmeter used in the next segment 
    term1 = alpha1*del_rot1 + alpha*del_trans;
    term2= alpha3*del_trans + alpha4*(del_rot1 + del_rot2);
    term3= alpha1*del_rot2 + alpha2*del_trans;

    //Gaussian noise distribution with zero mean and sd for each paramter modelling the noise of each parameter
    random_device rd{};
    mt19937 gen{rd()};
    ddash_rot1= del_rot1 - normal_distribution<double> d1{0, term1};
    ddash_trans= del_trans - normal_distribution<double> d2{0, term2};
    ddash_rot2= del_trans - normal_distribution<double> d3{0, term3};

    // converting parameters from the robot's coordinate frame to the world coordinate frame, again by simple trignometric calculations. 
    vector<double> xt(3, 0.0);
    xt[0]= x_t0[0] + ddash_trans* cos(x_t0[2] + ddash_rot1);
    xt[1]= x_t0[1] + ddash_trans* sin(x_t0[2]+ ddash_rot1);
    xt[2]= x_t0[2] + ddash_rot1 + ddash_rot2;
    
    return xt;
    
}