#include <iostream>
#include "mapReader.hh"

using namespace std;

int main(int argc, char *argv[]) {
    string map_name = "../data/map/wean.dat";
    MapReader map_reader(map_name.c_str());

    map_reader.read_map();
    map_type map = map_reader.map;
    map_reader.visulize_map();

    return 0;
}
