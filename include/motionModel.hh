#include <iostream>
using namespace std;

class MotionModel
{
private:
    float alpha1, alpha2, alpha3, alpha4;

public:
    MotionModel();
    vector<double> update(vector<double>, vector<double>, vector<double>);
};
