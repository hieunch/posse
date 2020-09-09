#ifndef __WSN_BOUNDHOLE_H_
#define __WSN_BOUNDHOLE_H_

#include "VirtualRouting.h"
#include <Common.h>
#include <RoutingBaseAlgorithm.h>
#include <BoundHoleMessage_m.h>
#include <vector>

enum BoundHoleTimer {
  START_BOUND_HOLE = 1,
};

class BoundHole: public RoutingBaseAlgorithm {

private:
    enum BoundHoleStorageMode {
        BoundHole_Storage_All, BoundHole_Storage_One,
    };

    void sortNeighbor();
    void findStuckAngle();

    void sendBoundHole();
    void recvBoundHole(BoundHolePacket *bhh);

    void sendRefresh(PolygonHole hole);
    void recvRefresh(BoundHolePacket*);

    NeighborRecord* getNeighborByBoundHole(Point*p, Point*prev);
    NeighborRecord* getNeighbor(int id);

    void dumpBoundHole(PolygonHole &hole);

protected:
    int countBoundholePkt;
    double storageOpt;
    int limitMaxHop;
    int limitMinHop;

    std::vector<StuckAngle> stuckAngle_;
    std::vector<PolygonHole> holeList;
    std::vector<NeighborRecord> nbList;

    PolygonHole* createHole(BoundHolePacket *packet);
    virtual void createPolygonHole(PolygonHole &hole);
    virtual void broadcastHCI() {
    };

    void startup() override;
    // void finishSpecific();
    // void fromApplicationLayer(cPacket *, const char *);
    void fromMacLayer(cPacket *, int, double, double);
    // void handleNetworkControlCommand(cMessage *);
    void timerFiredCallback(int);
    void recvMessage(BoundHolePacket* packet);
    void processBoundHolePacketFromMacLayer(BoundHolePacket* pkt);
};

#endif

