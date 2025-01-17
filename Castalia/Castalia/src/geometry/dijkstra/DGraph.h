#ifndef GEOMETRY_DIJKSTRA_DGRAPH_H_
#define GEOMETRY_DIJKSTRA_DGRAPH_H_

#include <GeoMathHelper.h>
#include <list>
#include <utility>
#include <vector>
#include <queue>

using namespace std;

typedef pair<double, int> pair_;
class DGraph {
    static map<pair<Point, Point>, vector<int>> spCache;
    static map<pair<Point, Point>, vector<Point>> spPointCache;
    list<pair_> *adj;
    vector<Point> polygon;
    vector<int> STP;
    void addVertexToPath(vector<int> parent, int j);
    bool isVertexOfPolygon(Point s, vector<Point> p);
    int getIndexInPolygon(Point s, vector<Point> p);
public:
    DGraph(vector<Point> hole);
    vector<Point> shortestPath(Point s, Point t);
    vector<int> shortestPath2(Point s, Point t);
    static void clearCache();
};

#endif /* GEOMETRY_DIJKSTRA_DGRAPH_H_ */
