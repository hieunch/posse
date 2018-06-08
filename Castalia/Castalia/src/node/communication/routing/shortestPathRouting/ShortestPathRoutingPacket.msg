
cplusplus {{
#include "RoutingPacket_m.h"
#include "GeoMathHelper.h"
}}

class noncobject Point;

class RoutingPacket;

enum ShortestPathPacketDef {
    SHORTEST_PATH_DATA_PACKET             =   0;       // data packet
};

enum RollingBallForwardingMode {
    GREEDY_ROUTING = 0;
    ROLLINGBALL_ROUTING = 1;
};

packet ShortestPathRoutingPacket extends RoutingPacket {
  int packetId;
  int previousId;
  int ShortestPathPacketKind enum (ShortestPathPacketDef);

  // normal field
  Point destLocation;
  Point sourceLocation;

  // for routing point to point
  Point ballCenter;
  Point stuckLocation;
  int routingMode enum (RollingBallForwardingMode);


  Point nextStoppingPlace;

  Point startShortestPathLocation;
}

packet ShortestPathDiscoverHolePacket extends RoutingPacket {
  int originatorId;
  Point ballCenter;
  int previousId;
  string path;
}


