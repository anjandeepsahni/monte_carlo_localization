#ifndef _MOTIONMODEL_H
#define _MOTIONMODEL_H

#include <iostream>
#include <cmath>
#include <random>
#include <vector>
using namespace std;

class MotionModel
{
    private:
    double alpha1, alpha2, alpha3, alpha4;

    public:
        MotionModel(double a1, double a2, double a3, double a4);
        state_t update(vector<double>, vector<double>, state_t);
};

#endif /* _MOTIONMODEL_H */
