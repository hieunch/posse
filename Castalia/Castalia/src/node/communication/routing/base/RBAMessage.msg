cplusplus {{
    #include "GeoMathHelper.h"
    #include "Common.h"
    #include "RoutingPacket_m.h"
}}

class noncobject Point;

class RoutingPacket;

packet RBAData extends RoutingPacket {
    Point destLocation;
    int packetId;
}