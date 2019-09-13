#ifndef _PARTICLEFILTER_H
#define _PARTICLEFILTER_H

#include <iostream>
using namespace std;


typedef struct state {
    double x;
    double y;
    double theta;
    double weight;
} state_t;

#endif  /* _PARTICLEFILTER_H */
