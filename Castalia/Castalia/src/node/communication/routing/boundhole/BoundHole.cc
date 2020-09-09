#include <BoundHole.h>
#include <GeoMathHelper.h>
#include <cmath>
#include <cstdio>
#include <log.h>

Define_Module(BoundHole);

void BoundHole::startup() {
    // FILE *fp;
    // fp = fopen("BoundHole.tr", "w");
    // fclose(fp);
    // fp = fopen("TraceBoundHole.tr", "w");
    // fclose(fp);

    countDataPkt = 0;
    countBoundholePkt = 0;
    range = 40;
    storageOpt = 1;//(BoundHoleStorageMode) (int) par("storageOpt");
    limitMaxHop = 1000;//(int) par("limitMaxHop");
    limitMinHop = (int) par("limitMinHop");

    sortNeighbor();
    findStuckAngle();
    setTimer(START_BOUND_HOLE, 1);
}

void BoundHole::timerFiredCallback(int index){
    switch(index){
        case START_BOUND_HOLE: {
            sendBoundHole();
        }
        default: break;
    }
}

// void BoundHole::fromApplicationLayer(cPacket * pkt, const char *destination){

// }

void BoundHole::fromMacLayer(cPacket * pkt, int macAddress, double rssi, double lqi){
  BoundHolePacket *netPacket = dynamic_cast <BoundHolePacket*>(pkt);
  if (netPacket){
        string dst(netPacket->getDestination());
        string src(netPacket->getSource());
        if ((dst.compare(BROADCAST_NETWORK_ADDRESS) == 0))
            trace() << "Received data from node " << src << " by broadcast";
        processBoundHolePacketFromMacLayer(netPacket);
  }
}

void BoundHole::processBoundHolePacketFromMacLayer(BoundHolePacket* pkt){

    string dst(pkt->getDestination());
    string src(pkt->getSource());
    int srcId(pkt->getSourceId());

    // if the node is the destination
    if (src.compare(SELF_NETWORK_ADDRESS) == 0) {
        trace() << "WSN_EVENT RECEIVE packetId:" << pkt->getPacketId() << " source:" << pkt->getSource()
        << " destination:" << pkt->getDestination() << " current:" << self;
        trace() << "WSN_EVENT ENERGY id:" << self << " energy:" << resMgrModule->getRemainingEnergy();
        toApplicationLayer(pkt->decapsulate());
        return;
    } 

    // if the node is the destination by broadcast, we do not forward it
    if ((dst.compare(BROADCAST_NETWORK_ADDRESS) == 0)) {
        // trace() << "Received data (routing broadcast) from MAC, send data to application layer. Source node: " << src;
        toApplicationLayer(pkt->decapsulate());
        return;
    }

    std::vector<int> data;
    for (auto nb : pkt->getBoundholeNodes()) {
        data.push_back(nb);
    }
    Point prev = GlobalLocationService::getLocation(data[data.size()-1]);
    Point prev2 = GlobalLocationService::getLocation(data[data.size()-2]);
    Angle a = G::angle(prev, prev2, prev, selfLocation);
    // for (int i=0; i<5; i++) trace() << "WSN_EVENT RECEIVE BOUNDHOLE packetId: " << pkt->getPacketId() << " source: " << pkt->getSourceId() << " current: " << self << " angle: " << a;
    recvMessage(pkt);
}

void BoundHole::recvMessage(BoundHolePacket* packet) {
    if (packet->getType() == BOUNDHOLE_BOUNDHOLE) {
        recvBoundHole(packet);
    } else if (packet->getType() == BOUNDHOLE_REFRESH) {
        recvRefresh(packet);
    } else {
//            throw cRuntimeError("Unknown BOUNDHOLE type");
        delete(packet);
    }
}

void BoundHole::sortNeighbor() {
    if (neighborTable.empty()) {
        return;
    }
    nbList.push_back(neighborTable[0]);

    unsigned int i = 1;
    for (i = 1; i < neighborTable.size(); i++) {
        Angle angle = G::angle(selfLocation, nbList[0].location, selfLocation, neighborTable[i].location);

        unsigned int j = 0;
        for (j = 0; j + 1 < nbList.size(); j++) {
            if (G::angle(selfLocation, nbList[0].location, selfLocation, nbList[j + 1].location) > angle) {
                nbList.insert(nbList.begin() + j + 1, neighborTable[i]);
                break;
            }
        }

        if (j == nbList.size() - 1) {
            nbList.push_back(neighborTable[i]);
        }
    }
}

//
// TENT rule
//

void BoundHole::findStuckAngle() {
    if (nbList.size() <= 1) {
        return;
    }

    unsigned int i = 0;
    NeighborRecord nb1 = nbList[i]; //u
    NeighborRecord nb2 = nbList[i + 1]; //v
    while (i < nbList.size()) {
        Circle2 circle = G::circumcenter(selfLocation, nb1.location, nb2.location);
        Angle a = G::angle(selfLocation, nb1.location, selfLocation, circle); // upO
        Angle b = G::angle(selfLocation, nb1.location, selfLocation, nb2.location); // upv
        Angle c = G::angle(selfLocation, circle, selfLocation, nb2.location); //Opv

        // if O is outside range of node, nb1 and nb2 create a stuck angle with node
        if (b >= M_PI || ((std::abs(fabs(a) + fabs(c) - fabs(b)) < EPSILON) && G::distance(selfLocation, circle) > range)) {
            stuckAngle_.push_back(StuckAngle(nb1.id, nb2.id));;
        }
        nb1 = nbList[++i];
        nb2 = nbList[(i + 1) % nbList.size()];
    }
    if (!stuckAngle_.empty()){
        trace() << "Stuck node: " << self;
    }
}

//
// BOUNDHOLE
//

void BoundHole::sendBoundHole() {
    if (self > 0) for (auto &sa : stuckAngle_) {
        BoundHolePacket *msg = new BoundHolePacket("BoundHole", NETWORK_LAYER_PACKET);
        msg->setType(BOUNDHOLE_BOUNDHOLE);
        msg->setSourceId(self);
        msg->setPreviousNode(selfLocation);
        msg->getBoundholeNodes().push_back(sa.b_);
        msg->getBoundholeNodes().push_back(self);
        msg->setTTL(limitMaxHop);
        msg->setPacketId(countBoundholePkt++);
        // msg->setSource(SELF_NETWORK_ADDRESS);
        // msg->setDestination(std::to_string(sa.a_).c_str());
        toMacLayer(msg, sa.a_);
    }
    if (!stuckAngle_.empty()){
        trace() << "Stuck node: " << self;
    }
}

void BoundHole::recvBoundHole(BoundHolePacket *bhh) {
    // for (int i=0; i<5; i++) trace() << "recvBoundHole";
    std::vector<int> data;
    for (auto nb : bhh->getBoundholeNodes()) {
        data.push_back(nb);
    }

    int pid(bhh->getSourceId());
    // if the boundhole packet has came back to the initial node
    if (pid == self) {
        if (bhh->getBoundholeNodes().size() < limitMinHop) {
            trace() << self << "SmallHole " << bhh->getBoundholeNodes().size();
            // logBoundHoleDrop(pid, "SmallHole");
            return; // SmallHole
        } else {
            trace() << self << " HOLE FOUND " << bhh->getHopCount();
            auto hole = createHole(bhh);
            createPolygonHole(*hole);
            dumpBoundHole(*hole);
            // debugPoint(GlobalLocationService::getLocation(pid), "blue");
            if (storageOpt == 1){//BoundHole_Storage_One) {
                broadcastHCI();
            } else {
                sendRefresh(*hole);
            }
            return;
        }
    }

    int n = data[0];
    if (n == bhh->getSourceId()) {
        data.erase(data.begin());
    } else {
        n = data[data.size() - 2];
    }
    Point n_location = GlobalLocationService::getLocation(n);
    NeighborRecord *nb = getNeighborByBoundHole(&bhh->getPreviousNode(), &n_location);
    // trace() << "getNeighborByBoundHole " << selfLocation << " " << data[data.size() - 1] << " " << n;
    if (nb == NULL) {
        // logBoundHoleDrop(pid, "DROP_RTR_NO_ROUTE");
        trace() << self << "DROP_RTR_NO_ROUTE";
        return; // DROP_RTR_NO_ROUTE
    }

    Point temp, next;
    for (unsigned int i = 1; i < data.size() - 1; i++) {
        temp = GlobalLocationService::getLocation(data[i]);
        next = GlobalLocationService::getLocation(data[i + 1]);

        if (G::is_intersect2(selfLocation, nb->location, temp, next)) {
            if (G::distance(temp, nb->location) < range) {
                while (data.size() >= (i + 2)) {
                    data.erase(data.begin() + i + 1);
                }
                data.push_back(nb->id);
                nb = getNeighborByBoundHole(&nb->location, &temp);
                if (nb == NULL) {
                    trace() << self << "DROP_RTR_NO_ROUTE";
                    // logBoundHoleDrop(pid, "DROP_RTR_NO_ROUTE");
                    return; // DROP_RTR_NO_ROUTE
                }
                continue;
            } else {
                nb = getNeighbor(data[i+1]);
                if (nb == NULL) {
                    trace() << self << "DROP_RTR_NO_ROUTE";
                    // logBoundHoleDrop(pid, "DROP_RTR_NO_ROUTE");
                    return; // DROP_RTR_NO_ROUTE
                }
                continue;
            }
        }
    }

    // if neighbor already send boundhole message to that node
    if (bhh->getSourceId() > self) {
        for (auto sa : stuckAngle_) {
            if (sa.a_ == nb->id) {
                trace() << self << "REPEAT";
                // logBoundHoleDrop(pid, "REPEAT");
                return; // REPEAT
            }
        }
    }

    data.push_back(self);

    BoundHolePacket *msg = new BoundHolePacket("BoundHole", NETWORK_LAYER_PACKET);//bhh->dup();
    msg->setPreviousNode(selfLocation);
    // msg->setBoundholeNodes(&data);
    msg->setType(BOUNDHOLE_BOUNDHOLE);
    msg->setSource(bhh->getSource());
    msg->setSourceId(bhh->getSourceId());
    msg->setTTL(bhh->getTTL());
    msg->setHopCount(bhh->getHopCount());
    for (auto &nb : data) {
        msg->getBoundholeNodes().push_back(nb);
    }
    toMacLayer(msg, nb->id);
}

PolygonHole* BoundHole::createHole(BoundHolePacket *packet) {
    PolygonHole *hole = new PolygonHole(self);

    auto data = packet->getBoundholeNodes();
    for (auto &nb : data) {
        hole->addNode(nb);
    }

    return hole;
}

NeighborRecord* BoundHole::getNeighborByBoundHole(Point *p, Point *prev) {
    Angle max_angle = -1;
    NeighborRecord* nb = NULL;

    for (auto &temp : nbList) {
        Angle a = G::angle(selfLocation, *p, selfLocation, temp.location);

        if (a > max_angle && (!G::is_intersect(selfLocation, temp.location, *p, *prev) || 
            (temp.location.x() == p->x_ && temp.location.y() == p->y_) || 
            (selfLocation.x() == prev->x_ && selfLocation.y() == prev->y_))) {
            max_angle = a;
            nb = &temp;
            // trace() << "getNeighborByBoundHole " << nb->id << " " << max_angle;
        }
    }
    double re1 = atan2(p->y_ - selfLocation.y_, p->x_ - selfLocation.x_);
    double re2 = atan2(nb->location.y_ - selfLocation.y_, nb->location.x_ - selfLocation.x_);
    double re = re1 - re2;
    // trace() << "getNeighborByBoundHole " << max_angle << " " << nb->id << " " << (p->y_ - selfLocation.y_) << " " << (p->x_ - selfLocation.x_) << " " << re1
        // << " " << (nb->location.y_ - selfLocation.y_) << " " << (nb->location.x_ - selfLocation.x_) << " " << re2 << " " << re;
    return nb;
}

void BoundHole::createPolygonHole(PolygonHole &hole) {
    holeList.push_back(hole);
}

//
// REFRESH
//

void BoundHole::sendRefresh(PolygonHole hole) {
    BoundHolePacket *p = new BoundHolePacket("BoundHoleRefresh", NETWORK_LAYER_PACKET);
    std::vector<int> h;
    for (auto it : hole.nodeList()) {
        h.push_back(it);
    }
    p->setType(BOUNDHOLE_REFRESH);
    p->setIndex(2);
    p->setPreviousNode(selfLocation);
    p->setSourceId(self);
    p->setTTL(h.size()-1);
    p->setPacketId(countBoundholePkt++);
    // p->setSource(SELF_NETWORK_ADDRESS);
    // p->setDestination(std::to_string(h[1]).c_str());
    for (auto &nb : h) {
            p->getBoundholeNodes().push_back(nb);
        }

    toMacLayer(p, h[1]);
}

void BoundHole::recvRefresh(BoundHolePacket *p) {
    unsigned int i = p->getIndex();
    std::vector<int> h;
    for (auto it : p->getBoundholeNodes()) {
        h.push_back(it);
    }
    auto hole = createHole(p);
    createPolygonHole(*hole);
    if ( i < h.size()) {
        p->setIndex(i+1);
        p->setPreviousNode(selfLocation);
        int nextHop = h[i];
        p->setTTL(h.size() - i + 1);
        toMacLayer(p, nextHop);
    } else {
        delete(p);
    }
}

NeighborRecord* BoundHole::getNeighbor(int id) {
    for (unsigned int i = 0; i < nbList.size(); i++) {
        if (nbList[i].id == id) {
            return &nbList[i];
        }
    }
    return NULL;
}

//
// DUMP
//

void BoundHole::dumpBoundHole(PolygonHole &hole) {
    // FILE *fp = fopen("BoundHole.tr", "a");
    // for (auto nb : hole.nodeList()) {
    //     fprintf(fp, "%d\t%f\t%f\n", nb.id(), nb.x(), nb.y());
    // }
    // fprintf(fp, "\n");
    // fclose(fp);

    for (auto id : hole.nodeList()) {
        trace() << "HOLE " << id;
        //debugPoint(GlobalLocationService::getLocation(id), "red");
    }
    vector<int> nodeList = hole.nodeList();
    for (int i=0; i<nodeList.size()-1; i++){
        debugLine(GlobalLocationService::getLocation(nodeList[i]), 
            GlobalLocationService::getLocation(nodeList[(i+1)%nodeList.size()]), "red");
    }

    // FILE *fp = fopen("BoundHole.tr", "a");
    // for (auto id : hole.nodeList()) {
    //     Point nb = GlobalLocationService::getLocation(id);
    //     fprintf(fp, "%f\t%f\n", nb.x(), nb.y());
    // }
    // fprintf(fp, "\n");
    // fclose(fp);
}
