#include <iostream>
#include <random>
#include <stdlib.h>
#include "resampler.hh"
#include "particleFilter.hh"

using namespace std;


vector<state_t> Resampler::multinomial_sampler(vector<state_t> x_bar)
{
    // number of particles
    long M = x_bar.size();

    // Extract the particle weights.
    vector<double> weights;
    for(int i=0; i<M; ++i) {
        weights.push_back(x_bar[i].weight);
    }

    // Create the distribution with those weights.
    discrete_distribution<> dist(weights.begin(), weights.end());

    // Use the distribution to resample particles.
    vector<state_t> x_bar_resampled(M);
    random_device rd{};
    mt19937 gen{rd()};

    for(int i=0; i<M; ++i) {
        x_bar_resampled[i] = x_bar[dist(gen)];
    }

    return x_bar_resampled;
}


vector<state_t> Resampler::low_variance_sampler(vector<state_t> x_bar)
{
    // number of particles
    long M = x_bar.size();

    // sampler parameters
    uniform_real_distribution<> dist(0.0, (double)(1.0/(double)M));
    random_device rd{};
    mt19937 gen{rd()};

    double r = dist(gen);
    double c = x_bar[0].weight;
    int i = 0;
    vector<state_t> x_bar_resampled(M);

    for (int m=1; m<=M; ++m)
    {
        double u = r + ((double) (m-1) / (double) M);
        while (u > c)
        {
            ++i;
            c += x_bar[i].weight;
        }
        x_bar_resampled[m-1] = x_bar[i];
    }

    return x_bar_resampled;
}

