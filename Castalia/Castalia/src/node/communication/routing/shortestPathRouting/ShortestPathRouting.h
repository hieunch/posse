

#ifndef _SHORTESTPATHROUTING_H_
#define _SHORTESTPATHROUTING_H_

#include <map>
#include "VirtualRouting.h"
#include "ShortestPathRoutingPacket_m.h"
#include "GeoMathHelper.h"


#define DEFAULT_SHORTESTPATH_TIMEOUT   200.0
/* the default time out period is 200.0 sec
   If the hello messge was not received during this period
   the entry in the neighbor list may be deleted 
   */

// if something is wrong, will retry sending HELLO message SHORTESTPATH_RETRY_HELLO_DELAY second later
#define SHORTESTPATH_RETRY_HELLO_DELAY 1

#define CavernPairRadius tuple<tuple<Point, Point>, Point, Point, double>

using namespace std;



enum ShortestPathRoutingTimers {
  DISCOVER_HOLE_START = 1,
};

class ShortestPathRouting: public VirtualRouting {
  private:
    static int cnt;
    static int nextId;
    const int NUM_PATH_CAVERN = 3;
    const int NUM_PATH_HOLE = 3;
    double holeDiameter;
    // Parameters
    int ShortestPathSetupFrameOverhead;	// in bytes
    double secondBallRadius;
    bool collectTraceInfo;
    map<int, int> smallestOriginatorId; // smallest originator id come from direction of a neighbor node (given by id)
    bool receivedHole;
    // ShortestPathRouting-related member variables
    vector<Point> hole;
    vector<Point> holeConvexHull;
    vector<vector<Point>> caverns;
    static map<tuple<Point, Point>, vector<Point> > shortestPathCache;
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
    void processDataPacket(ShortestPathRoutingPacket*);
    void processDiscoverHolePacket(ShortestPathDiscoverHolePacket*);
    void processHole(ShortestPathDiscoverHolePacket*);
    void propagateHole(ShortestPathDiscoverHolePacket*);

    void processPacketFirstTimeHaveHoleInfo(ShortestPathRoutingPacket*);
    void forwardToNextStoppingPlace(ShortestPathRoutingPacket*);

    Point getNeighborLocation(int);
    Point nearestCenter(Point pivot, Point next, Point center);

    vector<Point> findPath(Point from, Point to, vector<Point> &hole, vector<vector<Point>> &cavern);
};

#endif				//SHORTESTPATHROUTINGMODULE
