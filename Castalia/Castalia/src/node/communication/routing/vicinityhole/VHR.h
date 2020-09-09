#ifndef __WSN_VHR_H_
#define __WSN_VHR_H_

#include <boundhole/BoundHole.h>
#include <DGraph.h>
#include <VHRMessage_m.h>
#include <algorithm>

#define MIN_CAVE_VERTICES 10

using namespace std;

struct BoundaryNode : Point {
    int id_;
    bool is_convex_hull_boundary_;

    // comparison is done first on y coordinate and then on x coordinate
    bool operator<(BoundaryNode n2) {
        return y_ == n2.y_ ? x_ < n2.x_ : y_ < n2.y_;
    }
};

struct COORDINATE_ORDER {
    bool operator()(const BoundaryNode *a, const BoundaryNode *b) const {
        return a->y_ == b->y_ ? a->x_ < b->x_ : a->y_ < b->y_;
    }
};

// used for sorting points according to polar order w.r.t the pivot
struct POLAR_ORDER {
    struct BoundaryNode pivot;
    POLAR_ORDER(struct BoundaryNode p) { this->pivot = p; }
    bool operator()(const BoundaryNode *a, const BoundaryNode *b) const {
        int order = G::orientation(pivot, *a, *b);
        if (order == 0)
            return G::distance(pivot, *a) < G::distance(pivot, *b);
        return (order == 2);
    }
};

class VHR : public BoundHole
{
protected:
    void startup();
    void fromApplicationLayer(cPacket *, const char *);
    void fromMacLayer(cPacket *, int, double, double);

private:
    double broadcastFactor;
    double scaleFactor;
    vector<BoundaryNode> hole;

    // broadcast phase - disseminate hole's information
    void broadcastHCI() override;
    void recvHCI(BoundHolePacket *p);
    bool canBroadcast();

    // routing - handling data packet
    void sendData(VHRDataPacket*);
    void recvData(VHRDataPacket*);

    int getVertexIndexInPolygon(Point, vector<Point>);
    double calculatePathLength(vector<Point>);

    // geometry function - to find shortest path between source - destination
    void createPolygonHole(PolygonHole &) override;
    vector<BoundaryNode> determineConvexHull();
    double distanceToConvexHull();
    void findViewLimitVertices(Point, vector<BoundaryNode>, int &, int &);
    vector<Point> determineCaveContainingNode(Point s);
    vector<Point> bypassHole(double, double, double, double, int, int, int, int, vector<Point>);
    vector<Point> findPath(Point, Point);
    Point findHomotheticCenter(vector<Point>, double, double &);
    // dump
    void dumpBroadcastRegion();
    void dumpScale(Point);    // dump
};

#endif
