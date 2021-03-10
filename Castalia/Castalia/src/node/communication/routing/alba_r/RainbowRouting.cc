#include "RainbowRouting.h"

Define_Module(RainbowRouting);

int RainbowRouting::nextId;

void RainbowRouting::startup(){
  seqHello = par("seqHello");
  nextId = 0; // static member

}

void RainbowRouting::timerFiredCallback(int index){

  switch(index){
    default: break;
  }
}

void RainbowRouting::processBufferedPacket(){
  while (!TXBuffer.empty()) {
    toMacLayer(TXBuffer.front(), BROADCAST_MAC_ADDRESS);
    TXBuffer.pop();
  }
}

void RainbowRouting::fromApplicationLayer(cPacket * pkt, const char *destination){

  RainbowPacket *dataPacket = new RainbowPacket("Rainbow routing data packet", NETWORK_LAYER_PACKET);

  encapsulatePacket(dataPacket, pkt);
  dataPacket->setRainbowPacketKind(RAINBOW_DATA_PACKET);
  dataPacket->setSource(SELF_NETWORK_ADDRESS);
  dataPacket->setDestination(destination);

  if (string(destination).compare(BROADCAST_NETWORK_ADDRESS)==0) {
    toMacLayer(dataPacket, BROADCAST_MAC_ADDRESS);
    return;
  }


  dataPacket->setDestLocation(GlobalLocationService::getLocation(atoi(destination)));
  dataPacket->setPacketId(nextId++);
  dataPacket->setIsDataPacket(true);

  int nextHop = getNextHopGreedy(dataPacket);
  if (nextHop != -1) {
    trace() << "WSN_EVENT SEND packetId:" << dataPacket->getPacketId() << " source:" << dataPacket->getSource()
      << " destination:" << dataPacket->getDestination() << " current:" << self;
    trace() << "WSN_EVENT ENERGY id:" << self << " energy:" << resMgrModule->getRemainingEnergy();

    if ((nextId >= 99) && (nextId <= 120))
    debugLine(selfLocation, GlobalLocationService::getLocation(nextHop), "black");
    toMacLayer(dataPacket, nextHop);
    return;
  }
  else {
//    delete dataPacket;;
  }

}



void RainbowRouting::fromMacLayer(cPacket * pkt, int macAddress, double rssi, double lqi){
  RainbowPacket *netPacket = dynamic_cast <RainbowPacket*>(pkt);
  if (!netPacket)
    return;

  switch (netPacket->getRainbowPacketKind()) {
    case RAINBOW_DATA_PACKET:
      { 
        string dst(netPacket->getDestination());
        string src(netPacket->getSource());
        if ((dst.compare(BROADCAST_NETWORK_ADDRESS) == 0))
          trace() << "Received data from node " << src << " by broadcast";
        processDataPacketFromMacLayer(netPacket);
        break;
      }

    default: return;
  }
}

void RainbowRouting::finishSpecific() {
  int color = GlobalLocationService::getColor(self);
  // if (color == 0) debugPoint(selfLocation, "blue");
  // else if (color == 1) debugPoint(selfLocation, "green");
  // else if (color == 2) debugPoint(selfLocation, "yellow");
  // else if (color == 3) debugPoint(selfLocation, "red");
  if (GlobalLocationService::getNumReceived(self) > 0) debugPoint(selfLocation, "red");
  // trace() << "final color " << color;
  trace() << "WSN_EVENT FINAL" << " id:" << self << " x:" << selfLocation.x() << " y:" << selfLocation.y() << " deathTime:-1";
}

void RainbowRouting::processDataPacketFromMacLayer(RainbowPacket* pkt){

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
  RainbowPacket *netPacket = pkt->dup();
  int nextHop = getNextHopGreedy(netPacket);
  if (nextHop != -1) {
    trace() << "WSN_EVENT FORWARD packetId:" << pkt->getPacketId() << " source:" << pkt->getSource()
      << " destination:" << pkt->getDestination() << " current:" << self;
    trace() << "WSN_EVENT ENERGY id:" << self << " energy:" << resMgrModule->getRemainingEnergy();
    if ((nextId >= 99) && (nextId <= 120))
    debugLine(selfLocation, GlobalLocationService::getLocation(nextHop), "black");
    toMacLayer(netPacket, nextHop);
    return;
  }
  else {
    trace() << "WSN_EVENT DROP packetId:" << pkt->getPacketId() << " source:" << pkt->getSource()
      << " destination:" << pkt->getDestination() << " current:" << self;
//    delete netPacket;;
  };
}

int RainbowRouting::getNextHopGreedy(RainbowPacket* dataPacket){
  int nextHop = -1; double dist = 0;
  int tblSize = (int)neighborTable.size();
  int destination = atoi(dataPacket->getDestination());
  Point destLocation = dataPacket->getDestLocation();
  double dist0 = G::distance(selfLocation, destLocation);
  double minDist = DBL_MAX;
  int minNumReceived = INT32_MAX;

  int count = 0;
  while ((nextHop == -1) && (count++ < 100)) {
    int selfColor = GlobalLocationService::getColor(self, destination);
    int minColor = selfColor;
    trace() << "color: " << selfColor;
    for (auto &neighbor: neighborTable) {
      dist = G::distance(destLocation, neighbor.location);
      if ((selfColor%2 == 0) && (dist > dist0)) continue;
      if ((selfColor%2 == 1) && (dist <= dist0)) continue;
      int neighborColor = GlobalLocationService::getColor(neighbor.id, destination);    
      if ((neighborColor != selfColor) && (neighborColor != selfColor-1)) continue;
      int numReceived = GlobalLocationService::getNumReceived(neighbor.id);
      trace() << neighbor.id;

      if (neighborColor < minColor) {
        minColor = neighborColor;
        minNumReceived = numReceived;
        minDist = dist;
        nextHop = neighbor.id;
      }
      else if (neighborColor == minColor) {
        if (numReceived < minNumReceived) {
          minNumReceived = numReceived;
          minDist = dist;
          nextHop = neighbor.id;
        }
        else if ((numReceived == minNumReceived) && (dist < minDist)) {
          minDist = dist;
          nextHop = neighbor.id;
        }
      }
    }
    if (nextHop == -1) {
      GlobalLocationService::increaseColor(self, destination);
    }
  }

  return nextHop;
}


Point RainbowRouting::getNeighborLocation(int id) {
  for (auto &n: neighborTable) {
    if (n.id == id) {
      return n.location;
    }
  }

  return Point(); // default
}
// will handle interaction between the application layer and the GPRS module in order to pass parameters such as
// the node's position
void RainbowRouting::handleNetworkControlCommand(cMessage *msg) {
}