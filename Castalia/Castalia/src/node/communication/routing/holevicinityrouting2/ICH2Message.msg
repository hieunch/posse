cplusplus {{
    #include "GeoMathHelper.h"
    #include "RoutingPacket_m.h"
    typedef std::vector<Point> PointVector;
    typedef std::vector<double> doubleVector;
}}

class noncobject Point;
class RoutingPacket;
class noncobject PointVector;
class noncobject doubleVector;

enum RollingBallForwardingMode {
    GREEDY_ROUTING = 0;
    ROLLINGBALL_ROUTING = 1;
};

enum RollingBallPacketDef {
    ROLLINGBALL_DATA_PACKET             =   0;       // data packet
};

packet ICH2DataPacket extends RoutingPacket {
    int packetId;
    Point sourceLocation;
    Point destLocation;
    Point anchor;
   	unsigned int apIndex; //index of anchor point in shortest path from source to destination
    Point homotheticCenter;
    PointVector homotheticArray;
    double scaleFactor;
    doubleVector scaleFactorArray;
    PointVector shortestPath;

    int RollingBallPacketKind enum (RollingBallPacketDef);
    int routingMode enum (RollingBallForwardingMode);
    Point stuckLocation;
    Point ballCenter;
    Point previousLocation;
}