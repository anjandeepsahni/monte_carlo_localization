#ifndef _MAPREADER_H
#define _MAPREADER_H

#include <stdio.h>

typedef struct {
    int resolution, size_x, size_y;
    float offset_x, offset_y;
    int min_x, max_x, min_y, max_y;
    float **prob;   // prob of (x,y) being occupied
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
