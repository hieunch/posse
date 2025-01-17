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
  static double cellWidth;
  static double cellHeight;
  static std::vector<int> numReceiveds;
  static int numEnds;
  static std::vector<double> sumRatios;
  static std::vector<std::map<int, int>> colors;

 public:
  static int numNodes;
	GlobalLocationService();
  static void initialize(cModule*);
  static std::vector<NeighborRecord> getNeighborTable(int);
  static Point getLocation(int);
  static int getId(Point point);
  static int getNumReceived(int);
  static int getTotalNumReceived();
  static void increaseNumReceived(int);
  static void decreaseNumReceived(int);
  static double getSumRatio(int);
  static double getTotalSumRatio();
  static double getBalancingIndex();
  static double getStretch();
  static void increaseSumRatio(int, double);
  static map<tuple<int, int>, int> spCache;
  static int numHopShortestPath(int source, int destination);
  static bool isInSameCell(Point A, Point B);
  static vector<Point> getCell(Point p);
  static int getColor(int current, int destination);
  static int getColor(int node);
  static void increaseColor(int current, int destination);
  static void removeNode(int nodeId);
};

#endif
