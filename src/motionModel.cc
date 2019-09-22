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
    double ddash_rot1, ddash_trans, ddash_rot2;
    double term1, term2, term3;

    // Simple trignometric operations to calculate the robots displacement <s,s'>
    del_rot1 = atan2(u_t1[1] - u_t0[1], u_t1[0] - u_t0[0]) - u_t0[2];
    del_trans = sqrt(pow((u_t0[0] - u_t1[0]), 2) + pow((u_t0[1] - u_t1[1]), 2));
    del_rot2 = u_t1[2] - u_t0[2] - del_rot1;

    // Standard deviation for each parmeter used in the next segment
    term1 = alpha1 * abs(del_rot1) + alpha2 * abs(del_trans);
    term2 = alpha3 * abs(del_trans) + alpha4 * abs(del_rot1 + del_rot2);
    term3 = alpha1 * abs(del_rot2) + alpha2 * abs(del_trans);

    normal_distribution<double> d1{0, sqrt(term1)};
    normal_distribution<double> d2{0, sqrt(term2)};
    normal_distribution<double> d3{0, sqrt(term3)};

    // Gaussian noise distribution with zero mean and sd for each paramter,
    // thereby modelling the noise of each parameter
    random_device rd{};
    mt19937 gen{rd()};
    ddash_rot1 = del_rot1 - d1(gen);
    ddash_trans = del_trans - d2(gen);
    ddash_rot2 = del_rot2 - d3(gen);
    
    // Converting parameters from the robot's coordinate frame to the world
    // coordinate frame, again by simpletrignometric calculations.
    state_t st;
    st.x = x_t0.x + ddash_trans * cos(x_t0.theta + ddash_rot1);
    st.y = x_t0.y + ddash_trans * sin(x_t0.theta + ddash_rot1);
    st.theta = x_t0.theta + ddash_rot1 + ddash_rot2;
    st.weight = 0.0;

    return st;
}
