#ifndef _MAPREADER_H
#define _MAPREADER_H

#include <iostream>
#include <stdio.h>

using namespace std;

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
    const char *mapName;

    MapReader(const char *mapName);
    int read_map();
    int visulize_map();
};

#endif  /* _MAPREADER_H */
