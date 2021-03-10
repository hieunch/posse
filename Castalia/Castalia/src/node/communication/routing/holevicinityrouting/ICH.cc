#include <ICH.h>
#include <stack>
#include <vector>
#include <algorithm>
#include <ctime>

using namespace std;

Define_Module(ICH);

vector<BoundaryNode> ICH::hole1;

void ICH::startup() {
    broadcastFactor = 10;//par("broadcastFactor");
    scaleFactorCave = par("scaleFactorCave");
    scaleFactorConvex = par("scaleFactorConvex");
    srand (time(NULL));
    BoundHole::startup();
}

void ICH::handleRemoveNodeMessage(cMessage *msg) {
//   hole.clear();
//   hole1.clear();
    // DGraph::clearCache();
    // sortNeighbor();
    // findStuckAngle();
    // setTimer(START_BOUND_HOLE, 0);
//   BoundHole::startup();
}

void ICH::fromApplicationLayer(cPacket * pkt, const char *destination){

    ICHDataPacket *dataPacket = new ICHDataPacket("RBA routing data packet", NETWORK_LAYER_PACKET);

    encapsulatePacket(dataPacket, pkt);
    dataPacket->setSource(SELF_NETWORK_ADDRESS);
    dataPacket->setDestination(destination);

    if (string(destination).compare(BROADCAST_NETWORK_ADDRESS)==0) {
        toMacLayer(dataPacket, BROADCAST_MAC_ADDRESS);
        return;
    }

    Point destLocation = GlobalLocationService::getLocation(atoi(destination));
    dataPacket->setDestLocation(destLocation);
    dataPacket->setSourceLocation(selfLocation);
    dataPacket->setPacketId(countDataPkt++);
    dataPacket->setScaleFactor(1);
    dataPacket->setApIndex(0);
    dataPacket->setIsDataPacket(true);

    sendData(dataPacket);
}

void ICH::fromMacLayer(cPacket * pkt, int macAddress, double rssi, double lqi){

    ICHDataPacket *netPacket = dynamic_cast <ICHDataPacket*>(pkt);
    if (netPacket) {
        recvData(netPacket);
    } else {
        // numPacketReceived--;
        if (strcmp(pkt->getName(), "BroadcastHCI") == 0) {
            auto bh_pkt = static_cast<BoundHolePacket*>(pkt);
            recvHCI(bh_pkt);
        } else
            BoundHole::fromMacLayer(pkt, macAddress, rssi, lqi);
    }
}

/*----------- hole's information broadcast phase ------------*/
void ICH::broadcastHCI() {
    BoundHolePacket *p = new BoundHolePacket("BroadcastHCI", NETWORK_LAYER_PACKET);

    if (hole.empty())
        return;

    for (vector<BoundaryNode>::iterator it = hole.begin(); it != hole.end(); it++) {
        int tmp((*it).id_);
        p->getBoundholeNodes().push_back(tmp);
    }
    p->setSourceId(self);
    p->setDestination(BROADCAST_NETWORK_ADDRESS);

    toMacLayer(p, BROADCAST_MAC_ADDRESS);
}

void ICH::recvHCI(BoundHolePacket *p) {
    //if the hci packet has came back to the initial node
    if (p->getSourceId() == self) {
        trace() << "Loop_HCI " << self;
        delete (p);
        return;
    }

    // check if is really receive this hole's information
    if (!hole.empty())  // already receive
    {
        trace() << "HCI_received " << self;
        delete (p);
        return;
    }
    // store hci - just store hole's boundary node
    NodeVector data;
    for (auto it : p->getBoundholeNodes())
        data.push_back(it);
    for (auto it : data) {
        BoundaryNode *n = new BoundaryNode();
        n->id_ = it;
        Point location = GlobalLocationService::getLocation(it);
        n->x_ = location.x_;
        n->y_ = location.y_;
        hole.push_back(*n);
    }
    hole.pop_back();
    // if (hole1.empty()) hole1 = hole;
    bool b = canBroadcast();
    if (!b) {
        trace() << "OutsideBroadcastRegion " << self;
        delete(p);
    } else {
        BoundHolePacket *dup = p->dup();
        dup->setDestination(BROADCAST_NETWORK_ADDRESS);
        toMacLayer(dup, BROADCAST_MAC_ADDRESS);
    }

    dumpBroadcastRegion();
}

// distance broadcast = broadcastRatio * range
bool ICH::canBroadcast() {
    double distance = distanceToConvexHull();
    if (distance <= (broadcastFactor * range))
        return true;
    else
        return false;
}

double ICH::distanceToConvexHull() {
    vector<BoundaryNode> convex = determineConvexHull();

    Point tmp0;
    Point tmp1;
    Point tmp2;
    Point tmp3;
    Line l, l1, l2; // l: line P(i+1)(i+2), l1: parallel with line P(i)P(i+1), l2: parallel with line P(i+2)P(i+3)

    // check if point inside polygon return 0
    std::vector<Point> p;
    for (auto it : convex)
        p.push_back(Point(it.x_, it.y_));
    if (G::isPointInsidePolygon(selfLocation, p))
        return 0;

    double distance;
    double d = DBL_MAX;

    for (unsigned int i = 0; i < convex.size(); i++) {
        tmp0 = p[i];
        tmp1 = p[(i+1) % (p.size())];
        tmp2 = p[(i+2) % (p.size())];
        l = G::line(tmp1, tmp2);

        if (G::position(&selfLocation, &tmp0, &l) > 0) continue;

        distance = G::distance(&selfLocation, l);

        if(distance < d) {
            // detect if node stays on covering polygon's boundary
            tmp3 = p[(i+3) % (p.size())];
            l1 = G::line(tmp0, tmp1);
            l2 = G::line(tmp2, tmp3);

            int pos1 = G::position(&selfLocation, &tmp2, &l1);
            int pos2 = G::position(&selfLocation, &tmp1, &l2);

            if (pos1 * pos2 >= 0 && pos1 >= 0) {
                d = distance;
            } else if (pos1 * pos2 >= 0 && pos1 < 0) {
                if (G::distance(&selfLocation, l1) <= distance && G::distance(&selfLocation, l2) <= distance) {
                    d = distance;
                }
            } else {
                if ((pos1 < 0 && G::distance(&selfLocation, l1) <= distance) || (pos2 < 0 && G::distance(&selfLocation, l2) <= distance)) {
                    d = distance;
                }
            }
        }
    }
    return d;
}

void ICH::createPolygonHole(PolygonHole &h) {
    for (unsigned i = 0; i < h.nodeList().size(); i++) {
        BoundaryNode *bn = new BoundaryNode();
        bn->id_ = h.nodeList()[i];
        bn->is_convex_hull_boundary_ = false;
        Point location = GlobalLocationService::getLocation(h.nodeList()[i]);
        bn->x_ = location.x_;
        bn->y_ = location.y_;
        hole.push_back(*bn);
    }
}

vector<BoundaryNode> ICH::determineConvexHull() {
    stack<BoundaryNode*> hull;
    vector<BoundaryNode> convex;
    vector<BoundaryNode*> clone_hole;

    for (std::vector<BoundaryNode>::iterator it = hole.begin(); it != hole.end(); it++) {
        clone_hole.push_back(&(*it));
    }

    std::sort(clone_hole.begin(), clone_hole.end(), COORDINATE_ORDER());
    BoundaryNode *pivot = clone_hole.front();

    std::sort(clone_hole.begin() + 1, clone_hole.end(), POLAR_ORDER(*pivot));
    hull.push(clone_hole[0]);
    hull.push(clone_hole[1]);
    hull.push(clone_hole[2]);

    for (unsigned int i = 3; i < clone_hole.size(); i++) {
        BoundaryNode *top = hull.top();
        hull.pop();
        while (G::orientation(*hull.top(), *top, *clone_hole[i]) != 2) {
            top = hull.top();
            hull.pop();
        }
        hull.push(top);
        hull.push(clone_hole[i]);
    }

    while (!hull.empty()) {
        BoundaryNode *top = hull.top();
        top->is_convex_hull_boundary_ = true;
        convex.push_back(*top);
        hull.pop();
    }
    return convex;
}

/*----------- handling data packet ------------*/
void ICH::sendData(ICHDataPacket *pkt) {
    int nextHop;
    Point s = pkt->getSourceLocation();
    Point d = pkt->getDestLocation();
    vector<Point> sp;       // shortest path
    vector<Point> path;     // after scale
    Point I; // scale center

//    nextHop = getNextHopRollingBall(pkt->getAnchor());
//    if (nextHop == -1) {
    if (hole.empty() || !sdHoleIntersect(s, d)) {
        // trace() << "695 726 " << G::is_intersect(s, d, GlobalLocationService::getLocation(695), GlobalLocationService::getLocation(726));
        trace() << "case 0";
        if (hole.empty()) trace() << "hole empty";
        else trace() << "not intersect";
        nextHop = getNextHopRollingBall(pkt, d);
    } else {
        int s1, s2, d1, d2;
// 1. Determine convex hull
        vector<BoundaryNode> convex = determineConvexHull();
        std::vector<Point> p;
        for (auto it : convex)
            p.push_back(Point(it.x_, it.y_));

// Case 1: s, t are both outside convex hull
        if (!G::isPointInsidePolygon(s, p) && !G::isPointInsidePolygon(d, p)) {
            trace() << "case 1";
            findViewLimitVertices(s, convex, s1, s2);
            findViewLimitVertices(d, convex, d1, d2);
            vector<Point> sp = bypassHole(G::distance(s, p[s1]), G::distance(s, p[s2]),
                    G::distance(d, p[d1]), G::distance(d, p[d2]), s1, s2, d1,
                    d2, p);
            
            nextHop = findNextHop(pkt, s, d, sp);
        }
// Case 2: s is outside convex hull, d is inside convex hull
        else if (!G::isPointInsidePolygon(s, p) && G::isPointInsidePolygon(d, p)) {
            trace() << "case 2";
            if (pkt->getShortestPath().empty()){
                findViewLimitVertices(s, convex, s1, s2);
                for (int i=0; i<3; i++) trace() << "case f";

                vector<Point> d_cave = determineCaveContainingNode(d);
                DGraph *graph1 = new DGraph(d_cave);
                vector<Point> sp1 = graph1->shortestPath(d_cave[0], d);
                for (int i=0; i<3; i++) trace() << "case g";
                DGraph *graph2 = new DGraph(d_cave);
                vector<Point> sp2 = graph2->shortestPath(d_cave[d_cave.size() - 1], d);
                for (int i=0; i<3; i++) trace() << "case h";

                delete(graph1);
                delete(graph2);
                for (int i=0; i<3; i++) trace() << "case r";

                vector<Point> sp_ = bypassHole(G::distance(s, p[s1]),
                        G::distance(s, p[s2]), calculatePathLength(sp1),
                        calculatePathLength(sp2), s1, s2,
                        getVertexIndexInPolygon(d_cave[0], p),
                        getVertexIndexInPolygon(d_cave[d_cave.size() - 1], p), p);
                double k = pkt->getScaleFactor();
                for (int i=0; i<3; i++) trace() << "case t";

                sp = sp_;
                vector<Point> sp_cave;
                Point nA;
                if (sp_.back() == sp1.front()) {
                    sp.insert(sp.end(), sp1.begin() + 1, sp1.end()-1);
                    sp_cave = sp1;
                    nA = sp1[0];
                } else if (sp_.back() == sp2.front()) {
                    sp.insert(sp.end(), sp2.begin() + 1, sp2.end()-1);
                    sp_cave = sp2;
                    nA = sp2[0];
                }
                for (int i=0; i<3; i++) trace() << "case y";
                pkt->setShortestPath(sp);
            } else {
                sp = pkt->getShortestPath();
            }

            nextHop = findNextHop(pkt, s, d, sp);

        }
// Case 3: s is inside convex hull, d is outside convex hull
        else if (G::isPointInsidePolygon(s, p) && !G::isPointInsidePolygon(d, p)) {
            trace() << "case 3";
            findViewLimitVertices(d, convex, d1, d2);
            vector<Point> s_cave = determineCaveContainingNode(s);

            DGraph *graph1 = new DGraph(s_cave);
            vector<Point> sp1 = graph1->shortestPath(s, s_cave[0]);
            DGraph *graph2 = new DGraph(s_cave);
            vector<Point> sp2 = graph2->shortestPath(s, s_cave[s_cave.size() - 1]);

            delete(graph1);
            delete(graph2);

            vector<Point> sp_ = bypassHole(calculatePathLength(sp1),
                    calculatePathLength(sp2), G::distance(d, p[d1]),
                    G::distance(d, p[d2]),
                    getVertexIndexInPolygon(s_cave[0], p),
                    getVertexIndexInPolygon(s_cave[s_cave.size() - 1], p), d1,
                    d2, p);

            double k = pkt->getScaleFactor();
            vector<Point> sp_cave;
            if (sp1.back() == sp_.front()) {
                sp = sp1;
                sp_cave = sp1;
            } else if (sp2.back() == sp_.front()) {
                sp = sp2;
                sp_cave = sp2;
            }

            sp.insert(sp.end(), sp_.begin() + 1, sp_.end());
            sp.push_back(d);

            if (k != 1) {
                nextHop = getNextHopRollingBall(pkt, pkt->getAnchor());
                if (nextHop == -1) {
                    Point I = pkt->getHomotheticCenter();
                    if (pkt->getApIndex() != sp_cave.size() - 1) {
                        if (pkt->getApIndex() == sp.size() - 1) {
                            nextHop = getNextHopRollingBall(pkt, d);
                            debugLine(pkt->getAnchor(), d, "red");
                            pkt->setAnchor(d);
                            // pkt->setScaleFactor(1);
                        } else {
                            // k = 1;
                            Point nA = sp[pkt->getApIndex() + 1];
                            Point nAScale = Point(k * nA.x_ + (1 - k) * I.x_,
                                    k * nA.y_ + (1 - k) * I.y_);
                            nextHop = getNextHopRollingBall(pkt, nAScale);
                            pkt->setApIndex(pkt->getApIndex() + 1);
                            debugLine(pkt->getAnchor(), nAScale, "red");
                            pkt->setAnchor(nAScale);
                        }
                    } else {    // chuyen sang out of cave routing

                        Point nA = sp_[1];

                        // tinh ty so vi tu cho phan out of cave
                        I.x_ = 0;
                        I.y_ = 0;
                        double fr = 0;
                        for (auto it : sp_) {
                            int ra = rand();
                            I.x_ += it.x_ * ra;
                            I.y_ += it.y_ * ra;
                            fr += ra;
                        }
                        I.x_ = I.x_ / fr;
                        I.y_ = I.y_ / fr;

                        double dis = G::distance(I, sp_[0]);
                        dis += calculatePathLength(sp_);
                        dis += G::distance(sp_[sp_.size() - 1], I);
                        k = 1 + scaleFactorConvex * G::distance(s, d) / dis;

                        Point nAScale = Point(k * nA.x_ + (1 - k) * I.x_,
                                k * nA.y_ + (1 - k) * I.y_);
                        nextHop = getNextHopRollingBall(pkt, nAScale);
                        pkt->setHomotheticCenter(I);
                        pkt->setScaleFactor(k);
                        debugLine(pkt->getAnchor(), nAScale, "red");
                        pkt->setAnchor(nAScale);
                        pkt->setApIndex(pkt->getApIndex() + 1);
                    }
                }
            } else {
                if (sp_cave.size() == 2) {
                    nextHop = getNextHopRollingBall(pkt, sp_cave[1]);
                    debugLine(s, sp_cave[1], "red");
                    pkt->setAnchor(sp_cave[1]);
                } else {
                    I.x_ = 0;
                    I.y_ = 0;
                    double fr = 0;
                    for (auto it : sp_cave) {
                        int ra = rand();
                        I.x_ += it.x_ * ra;
                        I.y_ += it.y_ * ra;
                        fr += ra;
                    }
                    I.x_ = I.x_ / fr;
                    I.y_ = I.y_ / fr;

                    double dis = G::distance(I, sp_cave[0]);
                    dis += calculatePathLength(sp_cave);
                    dis += G::distance(sp_cave[sp_cave.size() - 1], I);
                    k = 1 + scaleFactorCave * G::distance(s, d) / dis;
                    Point nA = sp_cave[1];
                    Point nAScale = Point(k * nA.x_ + (1 - k) * I.x_,
                            k * nA.y_ + (1 - k) * I.y_);
                    nextHop = getNextHopRollingBall(pkt, nAScale);
                    pkt->setHomotheticCenter(I);
                    pkt->setScaleFactor(k);
                    debugLine(s, nAScale, "red");
                    pkt->setAnchor(nAScale);
                    pkt->setApIndex(pkt->getApIndex() + 1);
                }

                // for (int i=0; i<sp.size()-1; i++){
                //     debugLine(sp[i], sp[i+1], "red");
                //     debugPoint(sp[i], "red");
                // }
            }
        }
// Case 4: s, t are both inside convex hull
        else if (G::isPointInsidePolygon(s, p)
                && G::isPointInsidePolygon(d, p)) {
            trace() << "case 4";
            vector<Point> s_cave = determineCaveContainingNode(s);
            vector<Point> d_cave = determineCaveContainingNode(d);
            if (pkt->getShortestPath().empty()){
                if (s_cave[0] == d_cave[0] && s_cave[s_cave.size() - 1] == d_cave[d_cave.size() - 1]) {
                    DGraph *graph = new DGraph(d_cave);
                    sp = graph->shortestPath(s, d);
                } else {
                    DGraph *dgraph1 = new DGraph(d_cave);
                    DGraph *dgraph2 = new DGraph(d_cave);
                    DGraph *sgraph1 = new DGraph(s_cave);
                    DGraph *sgraph2 = new DGraph(s_cave);

                    vector<Point> s_sp1 = sgraph1->shortestPath(s, s_cave[0]);
                    vector<Point> s_sp2 = sgraph2->shortestPath(s, s_cave[s_cave.size() - 1]);

                    vector<Point> d_sp1 = dgraph1->shortestPath(d_cave[d_cave.size() - 1], d);
                    vector<Point> d_sp2 = dgraph2->shortestPath(d_cave[0], d);

                    delete (dgraph1);
                    delete (dgraph2);
                    delete (sgraph1);
                    delete (sgraph2);

                    vector<Point> sp_ = bypassHole(calculatePathLength(s_sp1),
                            calculatePathLength(s_sp2), calculatePathLength(d_sp1),
                            calculatePathLength(d_sp2),
                            getVertexIndexInPolygon(s_cave[0], p),
                            getVertexIndexInPolygon(s_cave[s_cave.size() - 1], p),
                            getVertexIndexInPolygon(d_cave[d_cave.size() - 1], p),
                            getVertexIndexInPolygon(d_cave[0], p), p);

                    if (s_sp1.back() == sp_.front()) {
                        sp.insert(sp.end(), s_sp1.begin() + 1, s_sp1.end());
                        sp.insert(sp.end(), sp_.begin() + 1, sp_.end());
                        if (sp_.back() == d_sp1.front()) {
                            sp.insert(sp.end(), d_sp1.begin() + 1, d_sp1.end() - 1);
                        } else if (sp_.back() == d_sp2.front()) {
                            sp.insert(sp.end(), d_sp2.begin() + 1, d_sp2.end() - 1);
                        }
                    } else if (s_sp2.back() == sp_.front()) {
                        sp.insert(sp.end(), s_sp2.begin() + 1, s_sp2.end());
                        sp.insert(sp.end(), sp_.begin() + 1, sp_.end());
                        if (sp_.back() == d_sp1.front()) {
                            sp.insert(sp.end(), d_sp1.begin() + 1, d_sp1.end() - 1);
                        } else if (sp_.back() == d_sp2.front()) {
                            sp.insert(sp.end(), d_sp2.begin() + 1, d_sp2.end() - 1);
                        }
                    }
                }
                pkt->setShortestPath(sp);
            } else {
                sp = pkt->getShortestPath();
            }

            nextHop = findNextHop(pkt, s, d, sp);
        }

    }
//    }
    if (nextHop != -1) {
        trace() << "WSN_EVENT SEND packetId:" << pkt->getPacketId() << " source:" << pkt->getSource()
        << " destination:" << pkt->getDestination() << " current:" << self << " next:" << nextHop
        << " anchor:" << pkt->getAnchor();
        trace() << "WSN_EVENT ENERGY id:" << self << " energy:" << resMgrModule->getRemainingEnergy();

        // debugPoint(pkt->getAnchor(), "black");
        // debugPoint(pkt->getHomotheticCenter(), "green");
        debugLine(selfLocation, GlobalLocationService::getLocation(nextHop), "black");
        toMacLayer(pkt, nextHop);
        return;
    }
    else {
        trace1() << "WSN_EVENT DROP AT STUCK_NODE packetId:" << pkt->getPacketId() << " source:" 
            << pkt->getSource() << " destination:" << pkt->getDestination() << " current:" << self;
        // delete dataPacket;
        // debugPoint(pkt->getAnchor(), "black");
        // debugPoint(pkt->getHomotheticCenter(), "blue");
        return;
    }
}

int ICH::findNextHop(ICHDataPacket* pkt, Point s, Point d, vector<Point> sp){
    vector<Point> holePoints;
    for (Point p : hole) holePoints.push_back(p);
    int nextHop = -1;
    Point I;
    double k = pkt->getScaleFactor();
    if (k != 1) {    // already has scale factor
        for (int i=0; i<3; i++) trace() << "case a";
        if (GlobalLocationService::isInSameCell(selfLocation, pkt->getAnchor())) {
        // if (reached(pkt->getAnchor())) {
            if (pkt->getApIndex() == sp.size() - 1) {
                for (int i=0; i<3; i++) trace() << "case a1";
                nextHop = getNextHopRollingBall(pkt, d);
                debugLine(pkt->getAnchor(), d, "red");
                pkt->setAnchor(d);
            //    pkt->setScaleFactor(1);
            } else {
                trace() << "case a2";
                // k = 1;
                // I = pkt->getHomotheticCenter();
                do {
                    I = pkt->getHomotheticArray()[pkt->getApIndex() + 1];
                    k = pkt->getScaleFactorArray()[pkt->getApIndex() + 1];
                    Point nA = sp[pkt->getApIndex() + 1];
                    Point nAScale = Point(k * nA.x_ + (1 - k) * I.x_, k * nA.y_ + (1 - k) * I.y_);
                    nextHop = getNextHopRollingBall(pkt, nAScale);
                    pkt->setApIndex(1 + pkt->getApIndex());
                    debugLine(pkt->getAnchor(), nAScale, "red");
                    pkt->setAnchor(nAScale);
                } while (G::isPointInsidePolygon(pkt->getAnchor(), holePoints));
            }
        }
        else {
            for (int i=0; i<3; i++) trace() << "case a3";
            nextHop = getNextHopRollingBall(pkt, pkt->getAnchor());
        }
    } else { // start scaling node - trong truong hop s khong co thong tin cua ho hoac node hien tai la node nguon
        for (int i=0; i<3; i++) trace() << "case b";
        vector<Point> homotheticArray;
        vector<double> scaleFactorArray;
        tie(homotheticArray, scaleFactorArray) = findHomotheticArray(sp, s, d);

        int startIndex = 0;
        do {
            Point nA = sp[startIndex];
            Point I = homotheticArray[startIndex];
            double k = scaleFactorArray[startIndex];
            Point nAScale = Point(k * nA.x_ + (1 - k) * I.x_,
                    k * nA.y_ + (1 - k) * I.y_);
            // debugLine(I, nAScale, "green");
            nextHop = getNextHopRollingBall(pkt, nAScale);
            pkt->setApIndex(startIndex);
            debugLine(s, nAScale, "red");
            pkt->setAnchor(nAScale);
            pkt->setHomotheticCenter(I);
            pkt->setScaleFactor(k);
            startIndex++;
        } while (G::isPointInsidePolygon(pkt->getAnchor(), holePoints));

        pkt->setHomotheticArray(homotheticArray);
        pkt->setScaleFactorArray(scaleFactorArray);

        // for (int i=0; i<sp.size()-1; i++){
        //     debugLine(sp[i], sp[i+1], "red");
        //     debugPoint(sp[i], "red");
        // }

        for (int i=0; i<sp.size(); i++){
            Point nA2 = sp[i];
            Point I2 = homotheticArray[i];
            double k2 = scaleFactorArray[i];
            Point nAScale2 = Point(k2 * nA2.x_ + (1 - k2) * I2.x_,
                    k2 * nA2.y_ + (1 - k2) * I2.y_);
            // debugLine(nA2, nAScale2, "green");
        }
    }
    return nextHop;
}

tuple<vector<Point>, vector<double>> ICH::findHomotheticArray(vector<Point> v, Point s, Point d) {
    vector<Point> holePoint;
    for (Point p : hole) holePoint.push_back(p);
    vector<Point> homotheticArray;
    vector<double> scaleFactorArray;
    vector<Point> monotoneSeg;

    int n = holePoint.size();
    double sign = 0;
    Point h = s;
    for (int i=0; i<v.size(); i++){
        if (monotoneSeg.empty()) monotoneSeg.push_back(v[i]);
        else {
            Point p0 = monotoneSeg[0];
            Point p1 = monotoneSeg[1];
            Point p_i1 = monotoneSeg[monotoneSeg.size()-1];
            Point p_i2 = v[i];
            if (i == v.size()-1) {
                monotoneSeg.push_back(v[i]);
                if (monotoneSeg.size() == 2) sign = G::isMonotoneLine(monotoneSeg[0], monotoneSeg[1], holePoint);
            }
            if ((G::isMonotoneLine(p_i1, p_i2, holePoint) == 0) || (i == v.size()-1)){
                if (monotoneSeg.size() == 1){
                    vector<Point> newSeg;
                    int id = G::getVertexIndexInPolygon(monotoneSeg[0], holePoint);
                    newSeg.push_back(holePoint[(id-1+n)%n]);
                    newSeg.push_back(holePoint[id]);
                    newSeg.push_back(holePoint[(id+1)%n]);
                    homotheticArray.push_back(findHomotheticCenter(newSeg, 1));
                }
                else {
                    Point I = findHomotheticCenter(monotoneSeg, sign);

                    double dis = G::distance(I, monotoneSeg[0]);
                    double length = calculatePathLength(monotoneSeg);
                    double delta = 0;
                    if (h == s) delta += G::distance(h, monotoneSeg[0]);
                    else delta += G::distance(h, monotoneSeg[0]) / 2;
                    if (i < v.size()-1) delta += G::distance(p_i1, p_i2) / 2;
                    else delta += G::distance(p_i2, d);
                    dis += length;
                    dis += G::distance(monotoneSeg[monotoneSeg.size() - 1], I);
                    double k = 1 + (scaleFactorConvex - 1) * (length + delta) / dis;
                    trace() << "k = " << k << " length + delta = " << length + delta << " dis = " << dis;

                    for (int j=0; j<monotoneSeg.size(); j++){
                        homotheticArray.push_back(I);
                        scaleFactorArray.push_back(k);
                    }
                }
                h = monotoneSeg[monotoneSeg.size()-1];
                trace() << "h = " << h;
                monotoneSeg.clear();
            }
            if (i < v.size()-1) monotoneSeg.push_back(v[i]);
            if (monotoneSeg.size() == 2) sign = G::isMonotoneLine(monotoneSeg[0], monotoneSeg[1], holePoint);
        }
    }
    if (homotheticArray.size() != v.size()) trace() << "ERROR";
    return make_tuple(homotheticArray, scaleFactorArray);
}

Point ICH::findHomotheticCenter(vector<Point> v, double sign) {
    Point I = Point();
    double range = 0.3;
    vector<Point> v_new;
    for (Point p : v) v_new.push_back(p);
    Point v_a, v_b;

    if (v.size() < 2) return v[0];
    else if (v.size() == 2) {
        Point v_a = v[0] + Vector(v[1], v[0]).rotate(M_PI / 2 * sign) * range;
        Point v_b = v[1] + Vector(v[0], v[1]).rotate(-M_PI / 2 * sign) * range;
        v_new.push_back(v_b);
        v_new.push_back(v_a);
    } 
    else {
        int last = v.size()-1;
        Angle A0 = M_PI - G::angle(v[0], v[1], v[last]);
        Angle An = M_PI - G::angle(v[last], v[last-1], v[0]);
        Vector V0(v[1], v[0]);
        Vector Vn(v[last-1], v[last]);
        double length = calculatePathLength(v);
        Vector V0u = V0 * (range*length/V0.length());
        Vector Vnu = Vn * (range*length/Vn.length());
        // if (Vector(v[0],v[1]).dotProduct(Vector(v[0],v[last])) >= 0){
        //     Point Va = v[0] + Vector(v[1], v[0]) * (range / sin(G::angle(v[1], v[0], v[2])));
        //     Point Vb = v[size()-1] + Vector(v[v.size()-2], v[v.size()-1]) * (range / sin(G::angle(v[v.size()-2], v[v.size()-1], v[v.size()-3])));
        //     v_new.push_back(Va);
        //     v_new.push_back(Vb);
        // } else {
            // Point v_a = v[0] + Vector(v[last], v[0]).rotate(M_PI / 2 * sign) * range;
            // Point v_b = v[last] + Vector(v[0], v[last]).rotate(-M_PI / 2 * sign) * range;
            // debugPoint(v_a, "blue");
            // debugPoint(v_b, "blue");
            // debugPoint(v_mid, "blue");
            // v_new.push_back(v_b);
            // v_new.push_back(v_a);
            // while(v_new.size() < v.size()) v_new.push_back(v_mid);
        // }
        if ((A0 < M_PI/2) && (An < M_PI/2)) {
            for (int i=0; i<3; i++) trace() << "case findHomothetic a";
            v_a = v[0] + V0u * (1 / sin(A0));
            v_b = v[last] + Vnu * (1 / sin(An));
            LineSegment l_a(v[0], v_a);
            LineSegment l_b(v[last], v_b);
            if (G::doIntersect(l_a,l_b)) {
                for (int i=0; i<3; i++) trace() << "case findHomothetic a.1";
                I = G::intersection(l_a, l_b);
                v_new.push_back(I);
            }
            else {
                for (int i=0; i<3; i++) trace() << "case findHomothetic a.2";
                v_new.push_back(v_b);
                v_new.push_back(v_a);
            }
        }
        else if ((A0 < M_PI/2) && (An >= M_PI/2)) {
            for (int i=0; i<3; i++) trace() << "case findHomothetic b";
            v_a = v[0] + V0u * (1 / sin(A0));
            if (An > M_PI-A0){
                for (int i=0; i<3; i++) trace() << "case findHomothetic b.1";
                v_b = v[last] + Vnu.rotate((An+A0-M_PI) * (-sign)) * (1 / sin(A0));
            } else {
                for (int i=0; i<3; i++) trace() << "case findHomothetic b.2";
                v_b = v[last] + Vnu * (1 / sin(An));
            }
            LineSegment l_a(v[0], v_a);
            LineSegment l_b(v[last], v_b);
            if (G::doIntersect(l_a,l_b)) {
                I = G::intersection(l_a, l_b);
                v_new.push_back(I);
            }
            else {
                v_new.push_back(v_b);
                v_new.push_back(v_a);
            }
        }
        else if ((A0 >= M_PI/2) && (An < M_PI/2)) {
            for (int i=0; i<3; i++) trace() << "case findHomothetic c";
            v_b = v[last] + Vnu * (1 / sin(An));
            if (A0 > M_PI-An){
                for (int i=0; i<3; i++) trace() << "case findHomothetic c.1";
                v_a = v[0] + V0u.rotate((A0+An-M_PI) * sign) * (1 / sin(An));
            } else {
                for (int i=0; i<3; i++) trace() << "case findHomothetic c.2";
                v_a = v[0] + V0u * (1 / sin(A0));
            }
            LineSegment l_a(v[0], v_a);
            LineSegment l_b(v[last], v_b);
            if (G::doIntersect(l_a,l_b)) {
                I = G::intersection(l_a, l_b);
                v_new.push_back(I);
            }
            else {
                v_new.push_back(v_b);
                v_new.push_back(v_a);
            }
        }
        else {
            for (int i=0; i<3; i++) trace() << "case findHomothetic d";
            Vector V(v[last], v[0]);
            Vector Vu = V.rotate(M_PI / 2 * sign) * (range*length/V.length());
            v_a = v[0] + Vu;
            v_b = v[last] + Vu;
            v_new.push_back(v_b);
            v_new.push_back(v_a);
        }
        trace() << "angle a " << A0/M_PI*180;
        trace() << "angle b " << An/M_PI*180;
    }
    debugPoint(v_a, "green");
    debugPoint(v_b, "blue");
    for (Point p : v) debugPoint(p, "black");
    debugPolygon(v_new, "black");
    
//     double fr = 0;
//     for (auto it : v_new) {
// //        srand(time(NULL));
//         int ra = rand();
//         I.x_ += it.x_ * ra;
//         I.y_ += it.y_ * ra;
//         fr += ra;
//     }
//     I.x_ = I.x_ / fr;
//     I.y_ = I.y_ / fr;
    I = G::randomPointInPolygon(v_new); 
    debugPoint(I, "purple");
    trace() << "homothetic center " << I;
    return I;
}

void ICH::recvData(ICHDataPacket *pkt) {
    // if (isPaused || hole1.empty()) return;
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
    ICHDataPacket *netPacket = pkt->dup();
    sendData(netPacket);
}

bool ICH::sdHoleIntersect(Point s, Point d) {
    int num_intersection = 0;

    for (unsigned int i = 0; i < hole.size(); i++) {
        unsigned int j = (i == hole.size() - 1 ? 0 : (i + 1));
        if (G::is_in_line(s, d, hole[i]) && G::is_in_line(s, d, hole[j])){
            // trace() << "is in line";
            break;
        }
        // trace() << GlobalLocationService::getId(hole[i]) << " " << GlobalLocationService::getId(hole[j]);
        if (G::is_intersect(s, d, hole[i], hole[j]))
            num_intersection++;
        // else trace() << "not intersect";
    }

    // trace() << num_intersection;
    return num_intersection > 0;

}

void ICH::findViewLimitVertices(Point point, std::vector<BoundaryNode> polygon, int &i1, int &i2) {
    std::vector<BoundaryNode*> clone;
    for (unsigned int i = 0; i < polygon.size(); i++) {
        polygon[i].id_ = i;
        clone.push_back(&polygon[i]);
    }

    BoundaryNode pivot;
    pivot.x_ = point.x_;
    pivot.y_ = point.y_;
    std::sort(clone.begin(), clone.end(), POLAR_ORDER(pivot));
    i1 = clone[0]->id_;
    i2 = clone[clone.size()-1]->id_;
}

int ICH::getVertexIndexInPolygon(Point t, vector<Point> p) {
    for (unsigned int i = 0; i < p.size(); i++)
        if (p[i] == t)
            return i;
    return NaN;
}

double ICH::calculatePathLength(vector<Point> p) {
    double length = 0;
    for (unsigned int i = 0; i < p.size() - 1; i++) {
        length += G::distance(p[i], p[i+1]);
    }
    return length;
}

// sDist1 = distance(S, p[s1]), sDist2 = distance(S, p[s2]); dDist1 = distance(D, p[d1]) dDist2 = distance(D, p[d2])
vector<Point> ICH::bypassHole(double sDist1, double sDist2, double dDist1, double dDist2, int s1, int s2, int d1, int d2, vector<Point> p) {
    Point ms = G::midpoint(p[s1], p[s2]);
    Point md = G::midpoint(p[d1], p[d2]);

    if (G::position(p[s1], G::line(ms, md)) > 0) {
        int tmp = s1;
        s1 = s2;
        s2 = tmp;
        double tmp_ = sDist1;
        sDist1 = sDist2;
        sDist2 = tmp_;
    }
    if (G::position(p[d1], G::line(ms, md)) > 0) {
        int tmp = d1;
        d1 = d2;
        d2 = tmp;
        double tmp_ = dDist1;
        dDist1 = dDist2;
        dDist2 = tmp_;
    }
    double t1, t2, t3, t4, min;
    vector<Point> sp1, sp2, sp3, sp4, sp;
    //============================================= S S1 D1 D
    t1 = sDist1;
    if (d1 < s1) d1 += p.size();
    for (int i = s1; i < d1; i++) {
        t1 += G::distance(p[i % p.size()], p[(i+1) % p.size()]);
        sp1.push_back(p[i % p.size()]);
    }
    t1 += dDist1;
    sp1.push_back(p[d1 % p.size()]);
    min = t1; sp = sp1;

    //============================================= S S2 D2 D
    t2 = sDist2;
    if (d2 < s2) d2 += p.size();
    for (int i = s2; i < d2; i++) {
        t2 += G::distance(p[i % p.size()], p[(i+1) % p.size()]);
        sp2.push_back(p[i % p.size()]);
    }
    t2 += dDist2;
    sp2.push_back(p[d2 % p.size()]);
    if (t2 < min) { min = t2; sp = sp2;}

    //============================================= D D1 S1 S
    t3 = dDist1;
    if (s1 < d1) s1 += p.size();
    for (int i = d1; i < s1; i++) {
        t3 += G::distance(p[i % p.size()], p[(i+1) % p.size()]);
        sp3.push_back(p[i % p.size()]);
    }
    t3 += sDist1;
    sp3.push_back(p[s1 % p.size()]);
    reverse(sp3.begin(), sp3.end());
    if (t3 < min) { min = t3; sp = sp3;}

    //============================================= D D2 S2 S
    t4 = dDist2;
    if (s2 < d2) s2 += p.size();
    for (int i = d2; i < s2; i++) {
        t4 += G::distance(p[i % p.size()], p[(i+1) % p.size()]);
        sp4.push_back(p[i % p.size()]);
    }
    t4 += sDist2;
    sp4.push_back(p[s2 % p.size()]);
    reverse(sp4.begin(), sp4.end());
    if (t4 < min) {sp = sp4;}

    return sp;

}
// determine cave of hole that contains node s, cave is CCW order after this phase
vector<Point> ICH::determineCaveContainingNode(Point s) {
    vector<BoundaryNode> convex = determineConvexHull();

    // rotate hole list, i.e. the first element is convex hull boundary
    int j = 0;
    while (!hole[j].is_convex_hull_boundary_)
        j++;
    rotate(hole.begin(), hole.begin() + j, hole.end());

    // determine cave containing s
    vector<Point> cave;
    for (unsigned int i = 0; i < hole.size() - 1; i++) {
        if (hole.at(i).is_convex_hull_boundary_ && s.x() == hole.at(i).x() && s.y() == hole.at(i).y()) {
            break; // s is gate of cave & lies on convex hull's boundary
        }
        if (hole.at(i).is_convex_hull_boundary_ && !hole.at(i+1).is_convex_hull_boundary_) {
            cave.push_back(hole.at(i++));
            while (!hole.at(i).is_convex_hull_boundary_) {
                cave.push_back(hole.at(i++));
                if (i == hole.size() - 1)
                    break;
            }
            cave.push_back(hole.at(i--));
            if (i == hole.size() - 2)
                cave.push_back(hole.at(0));
            if (cave.size() >= MIN_CAVE_VERTICES) {
                if (G::isPointInsidePolygon(s, cave))
                    break;
            }
//            vector<Point>().swap(cave);
            cave.clear();
        }
    }
    return cave;
}

void ICH::dumpBroadcastRegion() {
    // FILE *fp = fopen("BroadcastRegion.tr", "a+");
    // fprintf(fp, "%d\t%f\t%f\n", &selfLocation->id(), &selfLocation->x_, &selfLocation->y_);
    // fclose(fp);
}

void ICH::dumpScale(Point nAScale) {
    // FILE *fp = fopen("Scale.tr", "a+");
    // fprintf(fp, "%f\t%f\n", nAScale.x_, nAScale.y_);
    // fclose(fp);
}

int ICH::getNextHopGreedy(ICHDataPacket* dataPacket, Point destLocation){
  int nextHop = -1; double dist = 0;
  int tblSize = (int)neighborTable.size();
  double minDist = G::distance(selfLocation, destLocation);

  for (auto &neighbor: neighborTable) {
    // if (GlobalLocationService::isInSameCell(neighbor.location, destLocation)) {
    // if (reached(destLocation)) {
    //     nextHop = neighbor.id;
    //     break;
    // }
    dist = G::distance(destLocation, neighbor.location);

    if (dist < minDist) {
      minDist = dist;
      nextHop = neighbor.id;
    }
  }

  return nextHop;

  if (nextHop != -1) {
    return nextHop;
  } else {
    dataPacket->setRoutingMode(ROLLINGBALL_ROUTING);
    trace1() << "set mode ROLLINGBALL_ROUTING packetId:" << dataPacket->getPacketId() << " source:" 
            << dataPacket->getSource() << " destination:" << dataPacket->getDestination() << " current:" << self << " anchor:" << destLocation;
    dataPacket->setStuckLocation(selfLocation);

    // compute first ball with radius = RADIO_RANGE/2
    double x1 = selfLocation.x(), y1 = selfLocation.y();
    double x2 = destLocation.x(), y2 = destLocation.y();
    double d = G::distance(selfLocation, destLocation);
    double centerX = x1 + (x2 - x1) * RADIO_RANGE / 2 / d;
    double centerY = y1 + (y2 - y1) * RADIO_RANGE / 2 / d;
//    debugLine(selfLocation, destLocation, "black");
//    debugCircle(Point(centerX, centerY), RADIO_RANGE / 2, "red");
//    debugPoint(Point(centerX, centerY), "blue");

    // set ball radius
    dataPacket->setBallCenter(Point(centerX, centerY));
    return getNextHopRollingBall(dataPacket, destLocation);
  }

}

int ICH::getNextHopRollingBall(ICHDataPacket* dataPacket, Point destLocation){
    return getNextHopGreedy(dataPacket, destLocation);
    for (int i=0; i<3; i++) trace() << "get next hop";
    Point stuckLocation = dataPacket->getStuckLocation();
    trace() << "stuckLocation " << stuckLocation;
    if (stuckLocation.isUnspecified() || (G::distance(selfLocation, destLocation) - G::distance(stuckLocation, destLocation) < -EPSILON)) {
        for (int i=0; i<3; i++) trace() << "greedy";
        dataPacket->setStuckLocation(Point());
        dataPacket->setRoutingMode(GREEDY_ROUTING);
        return getNextHopGreedy(dataPacket, destLocation);
    } else {
        for (int i=0; i<3; i++) trace() << "Rolling Ball";
        int nextHop = -1;
        Point nextCenter;
        Point ballCenter = dataPacket->getBallCenter();
        nextHop = G::findNextHopRollingBall(selfLocation, ballCenter, RADIO_RANGE / 2, neighborTable, nextCenter);
        if (nextHop != -1) {
    //      debugCircle(nextCenter, RADIO_RANGE / 2, "blue");
            dataPacket->setBallCenter(nextCenter);
        }

        return nextHop;
    }
}