#ifndef __WSN_ROUTINGBASEALGORITHM_H_
#define __WSN_ROUTINGBASEALGORITHM_H_

#include "VirtualRouting.h"
#include <vector>
#include <RBAMessage_m.h>
#include <Common.h>

class RoutingBaseAlgorithm : public VirtualRouting
{
protected:
    // double range;
    // bool isSource;
    // int maxTTL;

    // used for log packets's ID
    int countDataPkt; // number of data packets that this node forward

    // int *mySelf, *dest;

protected:
    void startup();
    void finishSpecific();
    void fromApplicationLayer(cPacket *, const char *);
    void fromMacLayer(cPacket *, int, double, double);
    void processDataPacketFromMacLayer(RBAData* pkt);
    
    double range;

    // routing
    int getNextHopGreedy(Point &dest);

    // handling data
    void sendRBA(RBAData *msg);
};

#endif
