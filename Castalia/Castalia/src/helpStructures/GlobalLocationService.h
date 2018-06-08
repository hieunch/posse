#ifndef GLOBALLOCATIONSERVICE_H_
#define GLOBALLOCATIONSERVICE_H_

#include <map>
#include <vector>
#include <omnetpp.h>
#include <iostream>
#include <queue>
#include <iomanip>
#include "GeoMathHelper.h"

using namespace std;


class GlobalLocationService {
 private:
  static cModule *networkModule;
  static bool initialized;
  static std::vector<Point> locations;
  static std::vector<std::vector<NeighborRecord>> neighborTables;

 public:
	GlobalLocationService();
  static void initialize(cModule*);
  static std::vector<NeighborRecord> getNeighborTable(int);
  static Point getLocation(int);
  static map<tuple<int, int>, int> spCache;
  static int numHopShortestPath(int source, int destination);
};

#endif
