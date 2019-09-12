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

        vector<double> update(vector<double> , vector<double> , vector<double>);
};
