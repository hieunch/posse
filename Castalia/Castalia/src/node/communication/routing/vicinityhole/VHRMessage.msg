cplusplus {{
    #include "GeoMathHelper.h"
    #include "RoutingPacket_m.h"
    #include <vector>
    typedef std::vector<Point> PointVector;
}}

class noncobject Point;
class noncobject PointVector;
class RoutingPacket;

packet VHRDataPacket  extends RoutingPacket {
    int packetId;
    PointVector path;
    int apIndex;
    Point destLocation;
}