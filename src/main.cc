#include <iostream>
#include "mapReader.hh"

using namespace std;

int main(int argc, char *argv[]) {
    string map_name = "../data/map/wean.dat";
    MapReader map_reader(map_name.c_str());

    int ret_val = map_reader.read_map();
    if (ret_val >= 0)
    {
        map_type map = map_reader.map;
        map_reader.visulize_map();
    }

    return 0;
}
