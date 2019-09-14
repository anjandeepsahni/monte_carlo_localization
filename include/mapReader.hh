#ifndef _MAPREADER_H
#define _MAPREADER_H

#define MAP_VISUALIZE

#include <iostream>
#include <vector>
#include <stdio.h>
#include "particleFilter.hh"

#ifdef MAP_VISUALIZE
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif /* MAP_VISUALIZE */

using namespace std;
using namespace cv;

class SensorModel;  // Forward declaration

typedef struct {
    int resolution, size_x, size_y;
    double offset_x, offset_y;
    int min_x, max_x, min_y, max_y;
    vector<vector<double>> prob;   // prob of (x,y) being occupied
} map_type;


class MapReader
{
public:
    map_type map;
    string mapName;
    vector<Mat> videoFrames;

    MapReader(string mapName);
    int read_map();
    int visualize_map(vector<state_t> x_bar={}, bool storeForVideo=false, bool visRays=false, SensorModel* sensor_model=NULL);
    int save_video(string videoPath);
};

#endif  /* _MAPREADER_H */
