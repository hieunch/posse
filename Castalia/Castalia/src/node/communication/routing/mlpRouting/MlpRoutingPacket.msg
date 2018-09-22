
cplusplus {{
#include "RoutingPacket_m.h"
#include "GeoMathHelper.h"
}}

class noncobject Point;

class RoutingPacket;

enum MlpPacketDef {
    MLP_DATA_PACKET             =   0;       // data packet
};

packet MlpPacket extends RoutingPacket {
  int packetId;
  int previousId;
  int MlpPacketKind enum (MlpPacketDef);

  // normal field
  Point destLocation;
  Point sourceLocation;
}



