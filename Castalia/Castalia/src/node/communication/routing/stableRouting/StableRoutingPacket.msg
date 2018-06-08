
cplusplus {{
#include "RoutingPacket_m.h"
#include "GeoMathHelper.h"
}}

class noncobject Point;

class RoutingPacket;

enum StablePacketDef {
    STABLE_DATA_PACKET             =   0;       // data packet
};

enum RollingBallForwardingMode {
    GREEDY_ROUTING = 0;
    ROLLINGBALL_ROUTING = 1;
};

packet StablePacket extends RoutingPacket {
  int packetId;
  int previousId;
  int StablePacketKind enum (StablePacketDef);

  // normal field
  Point destLocation;
  Point sourceLocation;

  // for routing point to point
  Point ballCenter;
  Point stuckLocation;
  int routingMode enum (RollingBallForwardingMode);


  Point nextStoppingPlace;

  Point startStableLocation;
  double outCavernRadius = -1;
  double aroundHoleRadius = -1;
  double inCavernRadius = -1;
}

packet DiscoverHolePacket extends RoutingPacket {
  int originatorId;
  Point ballCenter;
  int previousId;
  string path;
}


