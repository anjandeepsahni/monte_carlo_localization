#include <iostream>
#include <cmath>
#include <random>
#include <vector>

#include "particleFilter.hh"
#include "../include/motionModel.hh"

using namespace std;


// Initialize the tuning paramters, starting from small uniform values
MotionModel::MotionModel(double a1, double a2, double a3, double a4)
{
    alpha1 = a1;
    alpha2 = a2;
    alpha3 = a3;
    alpha4 = a4;
}


state_t MotionModel::update(vector<double> u_t0, vector<double> u_t1, state_t x_t0)
{
    double del_rot1, del_trans, del_rot2;
    double del_rot1_hat, del_trans_hat, del_rot2_hat;
    double var1, var2, var3;

    // Simple trignometric operations to calculate the robots displacement <s,s'>
    del_rot1 = atan2(u_t1[1] - u_t0[1], u_t1[0] - u_t0[0]) - u_t0[2];
    del_trans = sqrt(pow((u_t0[0] - u_t1[0]), 2) + pow((u_t0[1] - u_t1[1]), 2));
    del_rot2 = u_t1[2] - u_t0[2] - del_rot1;

    // Standard deviation for each parmeter used in the next segment
    var1 = alpha1 * abs(del_rot1) + alpha2 * abs(del_trans);
    var2 = alpha3 * abs(del_trans) + alpha4 * abs(del_rot1 + del_rot2);
    var3 = alpha1 * abs(del_rot2) + alpha2 * abs(del_trans);

    normal_distribution<double> d1{0, sqrt(var1)};
    normal_distribution<double> d2{0, sqrt(var2)};
    normal_distribution<double> d3{0, sqrt(var3)};

//    var1 = alpha1 * del_rot1 + alpha2 * del_trans;
//    var2 = alpha3 * del_trans + alpha4 * del_rot1 + del_rot2;
//    var3 = alpha1 * del_rot2 + alpha2 * del_trans;
//
//    normal_distribution<double> d1{0, sqrt(abs(var1))};
//    normal_distribution<double> d2{0, sqrt(abs(var2))};
//    normal_distribution<double> d3{0, sqrt(abs(var3))};

    // Gaussian noise distribution with zero mean and sd for each paramter,
    // thereby modelling the noise of each parameter
    default_random_engine gen;
//    random_device rd{};
//    mt19937 gen{rd()};
    del_rot1_hat = del_rot1 - d1(gen);
    del_trans_hat = del_trans - d2(gen);
    del_rot2_hat = del_rot2 - d3(gen);
    
    // Converting parameters from the robot's coordinate frame to the world
    // coordinate frame, again by simpletrignometric calculations.
    state_t x_t1;
    x_t1.x = x_t0.x + del_trans_hat * cos(x_t0.theta + del_rot1_hat);
    x_t1.y = x_t0.y + del_trans_hat * sin(x_t0.theta + del_rot1_hat);
    x_t1.theta = x_t0.theta + del_rot1_hat + del_rot2_hat;
    x_t1.weight = x_t0.weight;    // Not necessary.

    return x_t1;
}
