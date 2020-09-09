
cplusplus {{
#include "RoutingPacket_m.h"
#include "GeoMathHelper.h"
#include "StableRoutingPacket_m.h"
typedef std::vector<Point> PointVector;
}}

class noncobject Point;
class noncobject PointVector;

class RoutingPacket;

enum MlpPacketDef {
    MLP_DATA_PACKET             =   0;       // data packet
};

enum MLPForwardingMode {
    MLP_GREEDY_ROUTING = 0;
    MLP_ROLLINGBALL_ROUTING = 1;
};

packet MlpPacket extends RoutingPacket {
  int packetId;
  int previousId;
  int MlpPacketKind enum (MlpPacketDef);

  // normal field
  Point destLocation;
  Point sourceLocation;

  // for routing point to point
  Point ballCenter;
  Point stuckLocation;
  int routingMode enum (MLPForwardingMode);

  Point previousStoppingPlace;
  Point nextStoppingPlace;
  int nextStoppingPlaceId;
  PointVector path;

  Point startStableLocation;
  int outDelta = 99999;
  double aroundHoleRadius = 99999;
  int inDelta = 99999;
}



