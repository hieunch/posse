#ifndef __WSN_ICH2_H_
#define __WSN_ICH2_H_

#include <boundhole/BoundHole.h>
#include <vicinityhole/VHR.h>
#include <holevicinityrouting2/ICH2Message_m.h>
#include <DGraph.h>
#include <algorithm>

#define MIN_CAVE_VERTICES 10

using namespace std;

class ICH2: public BoundHole {
protected:
    void startup();
    void fromApplicationLayer(cPacket *, const char *);
    void fromMacLayer(cPacket *, int, double, double);

private:
    double broadcastFactor;
    double scaleFactorCave;
    double scaleFactorConvex;
    vector<BoundaryNode> hole; // hole's node is clock wise order
    // broadcast phase - disseminate hole's information
    void broadcastHCI() override;
    void recvHCI(BoundHolePacket *p);
    bool canBroadcast();

    // routing - handling data packet
    void sendData(ICH2DataPacket*);
    void recvData(ICH2DataPacket*);
    int findNextHop(ICH2DataPacket*, Point, Point, vector<Point>);

    int getVertexIndexInPolygon(Point, vector<Point>);
    double calculatePathLength(vector<Point>);

    // geometry function - to find shortest path between source - destination
    void createPolygonHole(PolygonHole &) override;
    vector<BoundaryNode> determineConvexHull();
    double distanceToConvexHull();
//    vector<Point> findShortestPath(Point, Point );
    bool sdHoleIntersect(Point, Point);
    void findViewLimitVertices(Point, vector<BoundaryNode>, int &i1, int &i2);
    vector<Point> determineCaveContainingNode(Point s);
//    vector<Point> escapeHole(Point, Point);
    vector<Point> bypassHole(double, double, double, double, int, int, int, int, vector<Point>);

    Point findHomotheticCenter(vector<Point>, double);
    tuple<vector<Point>, vector<double>> findHomotheticArray(vector<Point>, Point, Point);

    // dump
    void dumpBroadcastRegion();
    void dumpScale(Point);
};

#endif
