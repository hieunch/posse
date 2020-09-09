
cplusplus {{
#include "RoutingPacket_m.h"
#include "GeoMathHelper.h"
}}

class noncobject Point;

class RoutingPacket;


enum RainbowPacketDef {
    RAINBOW_DATA_PACKET             =   0;       // data packet
};

packet RainbowPacket extends RoutingPacket {
    int packetId;
    int RainbowPacketKind enum (RainbowPacketDef);
    Point destLocation;
}



