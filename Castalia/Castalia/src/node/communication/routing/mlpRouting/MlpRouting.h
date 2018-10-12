

#ifndef _MLPROUTING_H_
#define _MLPROUTING_H_

#include <map>
#include "VirtualRouting.h"
#include "MlpRoutingPacket_m.h"
#include "StableRoutingPacket_m.h"
#include "GeoMathHelper.h"

#define NULL_VAL 99999

#define DEFAULT_MLP_TIMEOUT   200.0
/* the default time out period is 200.0 sec
   If the hello messge was not received during this period
   the entry in the neighbor list may be deleted 
   */

// if something is wrong, will retry sending HELLO message MLP_RETRY_HELLO_DELAY second later
#define MLP_RETRY_HELLO_DELAY 1
using namespace std;



enum MlpRoutingTimers {
  DISCOVER_HOLE_START = 1,
};

class MlpRouting: public VirtualRouting {
  private:
    static int cnt;
    static int nextId;
    const int RANGE = 2;
    double holeDiameter;
    // Parameters
    int MlpSetupFrameOverhead;	// in bytes
    double secondBallRadius;
    bool collectTraceInfo;
    map<int, int> smallestOriginatorId; // smallest originator id come from direction of a neighbor node (given by id)
    bool receivedHole;
    // MlpRouting-related member variables
    vector<Point> hole;
    vector<Point> holeConvexHull;
    vector<vector<Point>> caverns;

    static map<tuple<tuple<Point, Point>, Point, Point, int>, vector<Point> > outCavernCache;
    static map<tuple<Point, Point, double>, vector<Point> > aroundHoleCache;
    static map<tuple<Point, Point, int, double, int>, vector<Point> > findPathCache;
  protected:

    void startup();
    void finishSpecific();
    void fromApplicationLayer(cPacket *, const char *);
    void fromMacLayer(cPacket *, int, double, double);
    void handleNetworkControlCommand(cMessage *);
    void sendTopologySetupPacket();
    void timerFiredCallback(int);
    void processBufferedPacket();

    void processDataPacket(MlpPacket*);
    void processDiscoverHolePacket(DiscoverHolePacket*);
    void processHole(DiscoverHolePacket*);
    void propagateHole(DiscoverHolePacket*);
    tuple<vector<Point>, int, double, int> findPath(Point from, Point to,
        vector<Point> &hole, vector<vector<Point>> &caverns);
    vector<Point> findPathOutCavern(Point from, Point to, vector<Point> &hole, vector<Point> &cavern, int delta);
    vector<Point> findPathAroundHole(Point from, Point to, vector<Point> &hole, double ballRadius);
};

#endif				//MLPROUTINGMODULE
