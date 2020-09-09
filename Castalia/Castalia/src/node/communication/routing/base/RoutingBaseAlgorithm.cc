#include <Common.h>
#include <RoutingBaseAlgorithm.h>
#include <GeoMathHelper.h>
#include <cmath>
#include <iostream>
#include <iterator>
#include <utility>

Define_Module(RoutingBaseAlgorithm);

void RoutingBaseAlgorithm::startup() {
    trace() << "OK";
    countDataPkt = 0;
}

void RoutingBaseAlgorithm::fromApplicationLayer(cPacket * pkt, const char *destination){

    RBAData *dataPacket = new RBAData("RBA routing data packet", NETWORK_LAYER_PACKET);

    encapsulatePacket(dataPacket, pkt);
    dataPacket->setSource(SELF_NETWORK_ADDRESS);
    dataPacket->setDestination(destination);

    if (string(destination).compare(BROADCAST_NETWORK_ADDRESS)==0) {
        toMacLayer(dataPacket, BROADCAST_MAC_ADDRESS);
        return;
    }

    dataPacket->setDestLocation(GlobalLocationService::getLocation(atoi(destination)));
    dataPacket->setPacketId(countDataPkt++);

    sendRBA(dataPacket);
}

void RoutingBaseAlgorithm::fromMacLayer(cPacket * pkt, int macAddress, double rssi, double lqi){
  RBAData *netPacket = dynamic_cast <RBAData*>(pkt);
  if (netPacket){
        string dst(netPacket->getDestination());
        string src(netPacket->getSource());
        if ((dst.compare(BROADCAST_NETWORK_ADDRESS) == 0))
            trace() << "Received data from node " << src << " by broadcast";
        processDataPacketFromMacLayer(netPacket);
  }
}

void RoutingBaseAlgorithm::processDataPacketFromMacLayer(RBAData* pkt){

    string dst(pkt->getDestination());
    string src(pkt->getSource());

    // if the node is the destination
    if ((dst.compare(SELF_NETWORK_ADDRESS) == 0)) {
        trace() << "WSN_EVENT RECEIVE packetId:" << pkt->getPacketId() << " source:" << pkt->getSource()
        << " destination:" << pkt->getDestination() << " current:" << self;
        trace() << "WSN_EVENT ENERGY id:" << self << " energy:" << resMgrModule->getRemainingEnergy();
        toApplicationLayer(pkt->decapsulate());
        return;
    } 

    // if the node is the destination by broadcast, we do not forward it
    if ((dst.compare(BROADCAST_NETWORK_ADDRESS) == 0)) {
        trace() << "Received data (routing broadcast) from MAC, send data to application layer. Source node: " << src;
        toApplicationLayer(pkt->decapsulate());
        return;
    }


    // duplicate the packet because we are going to forward it
    RBAData *netPacket = pkt->dup();
    sendRBA(netPacket);
}

void RoutingBaseAlgorithm::finishSpecific() {
  trace() << "WSN_EVENT FINAL" << " id:" << self << " x:" << selfLocation.x() << " y:" << selfLocation.y() << " deathTime:-1";
}

void RoutingBaseAlgorithm::sendRBA(RBAData *dataPacket) {
    int nextHop = getNextHopGreedy(dataPacket->getDestLocation());
    if (nextHop != -1) {
        trace() << "WSN_EVENT SEND packetId:" << dataPacket->getPacketId() << " source:" << dataPacket->getSource()
        << " destination:" << dataPacket->getDestination() << " current:" << self;
        trace() << "WSN_EVENT ENERGY id:" << self << " energy:" << resMgrModule->getRemainingEnergy();

        debugLine(selfLocation, GlobalLocationService::getLocation(nextHop), "black");
        toMacLayer(dataPacket, nextHop);
        return;
    }
    else {
        trace() << "WSN_EVENT DROP AT STUCK_NODE packetId:" << dataPacket->getPacketId() << " source:" 
            << dataPacket->getSource() << " destination:" << dataPacket->getDestination() << " current:" << self;
        // delete dataPacket;
        return;
    }
}

int RoutingBaseAlgorithm::getNextHopGreedy(Point &dest){
    int nextHop = -1; double dist = 0;
    int tblSize = (int)neighborTable.size();
    double minDist = G::distance(selfLocation, dest);

    for (auto &neighbor: neighborTable) {
        dist = G::distance(dest, neighbor.location);

        if (dist < minDist) {
        minDist = dist;
        nextHop = neighbor.id;
        }
    }

    return nextHop;
}

