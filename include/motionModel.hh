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
        MotionModel(double alpha1, double aplha2, double alpha3, double alpha4);

        ~MotionModel();

        state_t update(vector<double>, vector<double>, state_t);
};

#endif /* _MOTIONMODEL_H */
