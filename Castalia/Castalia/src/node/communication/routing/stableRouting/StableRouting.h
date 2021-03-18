

#ifndef _STABLEROUTING_H_
#define _STABLEROUTING_H_

#include <map>
#include "VirtualRouting.h"
#include "StableRoutingPacket_m.h"
#include "GeoMathHelper.h"


#define DEFAULT_STABLE_TIMEOUT   200.0
/* the default time out period is 200.0 sec
   If the hello messge was not received during this period
   the entry in the neighbor list may be deleted 
   */

// if something is wrong, will retry sending HELLO message STABLE_RETRY_HELLO_DELAY second later
#define STABLE_RETRY_HELLO_DELAY 1

#define CavernPairRadius tuple<tuple<Point, Point>, Point, Point, double>

using namespace std;



enum StableRoutingTimers {
  DISCOVER_HOLE_START = 1,
};

class StableRouting: public VirtualRouting {
  private:
    static int cnt;
    static int nextId;
    const int NUM_PATH = 50;
    double holeDiameter;
    // Parameters
    int StableSetupFrameOverhead;	// in bytes
    double secondBallRadius;
    bool collectTraceInfo;
    map<int, int> smallestOriginatorId; // smallest originator id come from direction of a neighbor node (given by id)
    bool receivedHole;
    // StableRouting-related member variables
    vector<Point> hole;
    vector<Point> holeConvexHull;
    vector<vector<Point>> caverns;
    static map<CavernPairRadius, vector<Point> > outCavernCache;
    static map<tuple<Point, Point, double>, vector<Point> > aroundHoleCache;
    static map<tuple<Point, Point, double, double, double>, vector<Point> > findPathCache;
  protected:

    void startup();
    void finishSpecific();
    void fromApplicationLayer(cPacket *, const char *);
    void fromMacLayer(cPacket *, int, double, double);
    void handleNetworkControlCommand(cMessage *);
    void sendTopologySetupPacket();
    void timerFiredCallback(int);
    void processBufferedPacket();

    void sendHelloMessage();
    void processDataPacket(StablePacket*);
    void processDiscoverHolePacket(DiscoverHolePacket*);
    void processHole(DiscoverHolePacket*);
    void propagateHole(DiscoverHolePacket*);

    Point getNeighborLocation(int);
    Point nearestCenter(Point pivot, Point next, Point center);

    vector<Point> findPathOutCavern(Point from, Point to, vector<Point> &hole,
                                vector<Point> &cavern, double ballRadius);

    vector<Point> findPathAroundHole(Point from, Point to, vector<Point> &hole, double ballRadius);
    tuple<vector<Point>, double, double, double> findPath(Point from, Point to, vector<Point> &hole, vector<vector<Point>> &cavern);
};

#endif				//STABLEROUTINGMODULE
