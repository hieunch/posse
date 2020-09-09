#include <VHR.h>
#include <stack>
#include <vector>
#include <algorithm>
#include <time.h>
using namespace std;

Define_Module(VHR);

void VHR::startup() {
    broadcastFactor = par("broadcastFactor");
    scaleFactor = 0.7;//par("scaleFactor");
    BoundHole::startup();
}

void VHR::fromApplicationLayer(cPacket * pkt, const char *destination){

    VHRDataPacket *dataPacket = new VHRDataPacket("RBA routing data packet", NETWORK_LAYER_PACKET);

    encapsulatePacket(dataPacket, pkt);
    dataPacket->setSource(SELF_NETWORK_ADDRESS);
    dataPacket->setDestination(destination);

    if (string(destination).compare(BROADCAST_NETWORK_ADDRESS)==0) {
        toMacLayer(dataPacket, BROADCAST_MAC_ADDRESS);
        return;
    }

    Point destLocation = GlobalLocationService::getLocation(atoi(destination));
    dataPacket->setDestLocation(destLocation);
    dataPacket->setPacketId(countDataPkt++);

    if (!hole.empty()) {
        vector<Point> path = findPath(selfLocation, destLocation);
        if (path.empty())
            return;
        dataPacket->setPath(path);
        dataPacket->setApIndex(0);
    }

    sendData(dataPacket);
}

void VHR::fromMacLayer(cPacket * pkt, int macAddress, double rssi, double lqi){
    VHRDataPacket *netPacket = dynamic_cast <VHRDataPacket*>(pkt);
    if (netPacket) {
        recvData(netPacket);
    } else {
        if (strcmp(pkt->getName(), "BroadcastHCI") == 0) {
            auto bh_pkt = static_cast<BoundHolePacket*>(pkt);
            recvHCI(bh_pkt);
        } else
            BoundHole::fromMacLayer(pkt, macAddress, rssi, lqi);
    }
}

/*----------- hole's information broadcast phase ------------*/
void VHR::broadcastHCI() {
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

void VHR::recvHCI(BoundHolePacket *p) {
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
bool VHR::canBroadcast() {
    double distance = distanceToConvexHull();
    if (distance <= (broadcastFactor * range))
        return true;
    else
        return false;
}

double VHR::distanceToConvexHull() {
    vector<BoundaryNode> convex = determineConvexHull();
    // for (int i=0; i<convex.size()-1; i++){
    //     debugLine(GlobalLocationService::getLocation(convex[i].id_), 
    //         GlobalLocationService::getLocation(convex[(i+1)%convex.size()].id_), "blue");
    // }

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
    trace() << "DISTANCE " << d << " " << self;
    return d;
}

void VHR::createPolygonHole(PolygonHole &h) {
    for (unsigned i = 0; i < h.nodeList().size(); i++) {
        BoundaryNode *bn = new BoundaryNode();
        bn->id_ = h.nodeList()[i];
        bn->is_convex_hull_boundary_ = false;
        Point location = GlobalLocationService::getLocation(h.nodeList()[i]);
        bn->x_ = location.x_;
        bn->y_ = location.y_;
        hole.push_back(*bn);
    }
    // if (!hole.empty())
    //     debugPoint(selfLocation, "red");
}

vector<BoundaryNode> VHR::determineConvexHull() {
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

void VHR::sendData(VHRDataPacket *pkt) {
    int nextHop;
    if(!pkt->getPath().empty()) {
        nextHop = getNextHopGreedy(pkt->getPath()[pkt->getApIndex()]);
        while (nextHop == -1 && pkt->getApIndex() < (pkt->getPath().size() - 1)) {
            pkt->setApIndex(pkt->getApIndex() + 1);
            nextHop = getNextHopGreedy(pkt->getPath()[pkt->getApIndex()]);
        }
    } else {
        if (hole.empty()) {
            nextHop = getNextHopGreedy(pkt->getDestLocation());
        } else {
            vector<Point> path = findPath(selfLocation, pkt->getDestLocation());
            if (path.empty())
                return;
            pkt->setPath(path);
            pkt->setApIndex(0);
            nextHop = getNextHopGreedy(pkt->getPath()[pkt->getApIndex()]);
        }
    }

    if (nextHop == -1) {
        trace() << "WSN_EVENT DROP AT STUCK_NODE packetId:" << pkt->getPacketId() << " source:" 
            << pkt->getSource() << " destination:" << pkt->getDestination() << " current:" << self;
        delete (pkt);
        return;
    } else {
        trace() << "WSN_EVENT SEND packetId:" << pkt->getPacketId() << " source:" << pkt->getSource()
        << " destination:" << pkt->getDestination() << " current:" << self;
        trace() << "WSN_EVENT ENERGY id:" << self << " energy:" << resMgrModule->getRemainingEnergy();

        debugLine(selfLocation, GlobalLocationService::getLocation(nextHop), "black");
        toMacLayer(pkt, nextHop);
    }
}

void VHR::recvData(VHRDataPacket *pkt) {

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
    VHRDataPacket *netPacket = pkt->dup();
    sendData(netPacket);
}

vector<Point> VHR::findPath(Point s, Point d) {
    Point I; // homothetic center
    double k; // scale ratio
    vector<Point> path; // after scaling
    int s1, d1, s2, d2;
// 1. Determine convex hull
    vector<BoundaryNode> convex = determineConvexHull();
    std::vector<Point> p;
    for (auto it : convex)
        p.push_back(Point(it.x_, it.y_));

// Case 1: s, d are both outside convex hull
    if (!G::isPointInsidePolygon(s, p) && !G::isPointInsidePolygon(d, p)) {
        findViewLimitVertices(s, convex, s1, s2);
        findViewLimitVertices(d, convex, d1, d2);
        vector<Point> sp = bypassHole(G::distance(s, p[s1]), G::distance(s, p[s2]),
                G::distance(d, p[d1]), G::distance(d, p[d2]), s1, s2, d1,
                d2, p);

        // scaling
        vector<Point> sp_;
        sp_.push_back(s);
        sp_.insert(sp_.end(), sp.begin(), sp.end());
        sp_.push_back(d);
        I = findHomotheticCenter(sp_, scaleFactor, k);
        for (auto it : sp) {
            Point anchor = Point(k*it.x_ + (1-k)*I.x_, k*it.y_ + (1-k)*I.y_);
            path.push_back(anchor);
        }
        path.push_back(d);
        auto flattenPath = G::flatten(path);
        return flattenPath;
    }

// Case 2: s is outside convex hull, d is inside convex hull
    else if (!G::isPointInsidePolygon(s, p) && G::isPointInsidePolygon(d, p)) {
        findViewLimitVertices(s, convex, s1, s2);

        vector<Point> d_cave = determineCaveContainingNode(d);
        if (d_cave.empty()) {
            auto flattenPath = G::flatten(path);
            return flattenPath;
        }
        DGraph *graph1 = new DGraph(d_cave);
        vector<Point> sp1 = graph1->shortestPath(d_cave[0], d);
        DGraph *graph2 = new DGraph(d_cave);
        vector<Point> sp2 = graph2->shortestPath(d_cave[d_cave.size() - 1],
                d);

        delete (graph1);
        delete (graph2);

        vector<Point> sp = bypassHole(G::distance(s, p[s1]),
                G::distance(s, p[s2]), calculatePathLength(sp1),
                calculatePathLength(sp2), s1, s2,
                getVertexIndexInPolygon(d_cave[0], p),
                getVertexIndexInPolygon(d_cave[d_cave.size() - 1], p), p);

        // scale sp
        vector<Point> sp_;
        sp_.push_back(s);
        sp_.insert(sp_.end(), sp.begin(), sp.end());
        sp_.push_back(d);
        I = findHomotheticCenter(sp_, scaleFactor, k);
        for (unsigned int i = 0; i < sp.size() - 1; i++) {
            Point anchor = Point(k * sp[i].x_ + (1 - k) * I.x_, k * sp[i].y_ + (1 - k) * I.y_);
            path.push_back(anchor);
        }

        // scale concave path
        if (sp.back() == sp1.front()) {
            if (sp1.size() == 2) {
                path.push_back(sp1.front());
            } else {
                I = findHomotheticCenter(sp1, scaleFactor, k);
                for (unsigned int i = 0; i < sp1.size() - 1; i++) {
                    Point anchor = Point(k * sp1[i].x_ + (1 - k) * I.x_, k * sp1[i].y_ + (1 - k) * I.y_);
                    path.push_back(anchor);
                }
            }
        } else if (sp.back() == sp2.front()) {
            if (sp2.size() == 2) {
                path.push_back(sp2.front());
            } else {
                I = findHomotheticCenter(sp2, scaleFactor, k);
                for (unsigned int i = 0; i < sp2.size() - 1; i++) {
                    Point anchor = Point(k * sp2[i].x_ + (1 - k) * I.x_, k * sp2[i].y_ + (1 - k) * I.y_);
                    path.push_back(anchor);
                }
            }
        }
        path.push_back(d);
        auto flattenPath = G::flatten(path);
        return flattenPath;
    }

// Case 3: s is inside convex hull, d is outside convex hull
    else if (G::isPointInsidePolygon(s, p) && !G::isPointInsidePolygon(d, p)) {
        findViewLimitVertices(d, convex, d1, d2);
        vector<Point> s_cave = determineCaveContainingNode(s);
        if (s_cave.empty()) {
            auto flattenPath = G::flatten(path);
            return flattenPath;
        }
        DGraph *graph1 = new DGraph(s_cave);
        vector<Point> sp1 = graph1->shortestPath(s, s_cave[0]);
        DGraph *graph2 = new DGraph(s_cave);
        vector<Point> sp2 = graph2->shortestPath(s, s_cave[s_cave.size() - 1]);

        delete (graph1);
        delete (graph2);

        vector<Point> sp = bypassHole(calculatePathLength(sp1),
                calculatePathLength(sp2), G::distance(d, p[d1]),
                G::distance(d, p[d2]), getVertexIndexInPolygon(s_cave[0], p),
                getVertexIndexInPolygon(s_cave[s_cave.size() - 1], p), d1, d2,
                p);

        // scale cave path
        if (sp1.back() == sp.front()) {
            if (sp1.size() == 2) {
                path.push_back(sp.front());
            } else {
                I = findHomotheticCenter(sp1, scaleFactor, k);
                for (unsigned int i = 1; i < sp1.size(); i++) {
                    Point anchor = Point(k * sp1[i].x_ + (1 - k) * I.x_, k * sp1[i].y_ + (1 - k) * I.y_);
                    path.push_back(anchor);
                }
            }
        } else if (sp2.back() == sp.front()) {
            if (sp2.size() == 2) {
                path.push_back(sp.front());
            } else {
                I = findHomotheticCenter(sp2, scaleFactor, k);
                for (unsigned int i = 1; i < sp2.size(); i++) {
                    Point anchor = Point(k * sp2[i].x_ + (1 - k) * I.x_, k * sp2[i].y_ + (1 - k) * I.y_);
                    path.push_back(anchor);
                }
            }
        }

        // scale convex path
//        if (sp.size() == 2) {
//            path.push_back(sp.back());
//        } else {

        vector<Point> sp_;
        sp_.push_back(s);
        sp_.insert(sp_.end(), sp.begin(), sp.end());
        sp_.push_back(d);
        I = findHomotheticCenter(sp_, scaleFactor, k);
        for (unsigned int i = 1; i < sp.size() - 1; i++) {
            Point anchor = Point(k * sp[i].x_ + (1 - k) * I.x_, k * sp[i].y_ + (1 - k) * I.y_);
            path.push_back(anchor);
//            }
        }
        path.push_back(d);
        for (auto pt : path){
            debugPoint(pt, "black");
        }
        auto flattenPath = G::flatten(path);
        return flattenPath;
    }

// Case 4: both s and d is inside convex hull
    else if (G::isPointInsidePolygon(s, p) && G::isPointInsidePolygon(d, p)) {
        vector<Point> s_cave = determineCaveContainingNode(s);
        vector<Point> d_cave = determineCaveContainingNode(d);
        if (s_cave.empty() || d_cave.empty()) {
            auto flattenPath = G::flatten(path);
            return flattenPath;
        }
        if (s_cave[0] == d_cave[0] && s_cave[s_cave.size() - 1] == d_cave[d_cave.size() - 1]) {
            DGraph *graph = new DGraph(d_cave);
            vector<Point> sp = graph->shortestPath(s, d);

            // scale
            vector<Point> sp_;
            sp_.push_back(s);
            sp_.insert(sp_.end(), sp.begin(), sp.end());
            sp_.push_back(d);
            I = findHomotheticCenter(sp_, scaleFactor, k);
            for (auto it : sp) {
                Point anchor = Point(k * it.x_ + (1-k) * I.x_, k * it.y_ + (1-k) * I.y_);
                path.push_back(anchor);
            }
            path.push_back(d);
            auto flattenPath = G::flatten(path);
        return flattenPath;
        } else {
            DGraph *dgraph1 = new DGraph(d_cave);
            DGraph *dgraph2 = new DGraph(d_cave);
            DGraph *sgraph1 = new DGraph(s_cave);
            DGraph *sgraph2 = new DGraph(s_cave);

            vector<Point> s_sp1 = sgraph1->shortestPath(s, s_cave[0]);
            vector<Point> s_sp2 = sgraph2->shortestPath(s,
                    s_cave[s_cave.size() - 1]);

            vector<Point> d_sp1 = dgraph1->shortestPath(
                    d_cave[d_cave.size() - 1], d);
            vector<Point> d_sp2 = dgraph2->shortestPath(d_cave[0], d);

            delete (dgraph1);
            delete (dgraph2);
            delete (sgraph1);
            delete (sgraph2);

            vector<Point> sp = bypassHole(calculatePathLength(s_sp1),
                    calculatePathLength(s_sp2), calculatePathLength(d_sp1),
                    calculatePathLength(d_sp2),
                    getVertexIndexInPolygon(s_cave[0], p),
                    getVertexIndexInPolygon(s_cave[s_cave.size() - 1], p),
                    getVertexIndexInPolygon(d_cave[d_cave.size() - 1], p),
                    getVertexIndexInPolygon(d_cave[0], p), p);

            // scale s_cave path
            if (s_sp1.back() == sp.front()) {
                if (s_sp1.size() == 2) {
                    path.push_back(s_sp1.back());
                } else {
                    I = findHomotheticCenter(s_sp1, scaleFactor, k);
                    for (unsigned int i = 1; i < s_sp1.size(); i++) {
                        Point anchor = Point(k * s_sp1[i].x_ + (1 - k) * I.x_, k * s_sp1[i].y_ + (1 - k) * I.y_);
                        path.push_back(anchor);
                    }
                }
            } else if (s_sp2.back() == sp.front()) {
                if (s_sp2.size() == 2) {
                    path.push_back(s_sp2.back());
                } else {
                    I = findHomotheticCenter(s_sp2, scaleFactor, k);
                    for (unsigned int i = 1; i < s_sp2.size(); i++) {
                        Point anchor = Point(k * s_sp2[i].x_ + (1 - k) * I.x_, k * s_sp2[i].y_ + (1 - k) * I.y_);
                        path.push_back(anchor);
                    }
                }
            }

            // scale sp
//            if (sp.size() != 2) {
            vector<Point> sp_;
            sp_.push_back(s);
            sp_.insert(sp_.end(), sp.begin(), sp.end());
            sp_.push_back(d);
            I = findHomotheticCenter(sp, scaleFactor, k);
            for (unsigned int i = 1; i < sp.size() - 1; i++) {
                Point anchor = Point(k * sp[i].x_ + (1 - k) * I.x_, k * sp[i].y_ + (1 - k) * I.y_);
                path.push_back(anchor);
//                }
            }

            // scale d_cave path
            if (d_sp1.front() == sp.back()) {
                if (d_sp1.size() == 2) {
                    path.push_back(d_sp1.front());
                } else {
                    I = findHomotheticCenter(d_sp1, scaleFactor, k);
                    for (unsigned int i = 0; i < d_sp1.size() - 1; i++) {
                        Point anchor = Point(k * d_sp1[i].x_ + (1 - k) * I.x_, k * d_sp1[i].y_ + (1 - k) * I.y_);
                        path.push_back(anchor);
                    }
                }
            } else if (d_sp2.front() == sp.back()) {
                if (d_sp2.size() == 2) {
                    path.push_back(d_sp2.front());
                } else {
                    I = findHomotheticCenter(d_sp2, scaleFactor, k);
                    for (unsigned int i = 0; i < d_sp2.size() - 1; i++) {
                        Point anchor = Point(k * d_sp2[i].x_ + (1 - k) * I.x_, k * d_sp2[i].y_ + (1 - k) * I.y_);
                        path.push_back(anchor);
                    }
                }
            }
            path.push_back(d);
            auto flattenPath = G::flatten(path);
        return flattenPath;
        }
    }
}

Point VHR::findHomotheticCenter(vector<Point> v, double lamda, double &k) {
    Point I = Point(0, 0);
    double fr = 0;
    for (auto it : v) {
//        srand(time(NULL));
        int ra = intuniform(1,10^6);
        I.x_ += it.x_ * ra;
        I.y_ += it.y_ * ra;
        fr += ra;
    }
    I.x_ = I.x_ / fr;
    I.y_ = I.y_ / fr;

    // calculate scale ratio
    double dis = G::distance(I, v[0]);
    double l = calculatePathLength(v);
    dis += l;
    dis += G::distance(I, v[v.size() - 1]);
    k = 1 + lamda * l / dis;

    for (int i=0; i<v.size(); i++){
        debugPoint(v[i], "green");
        debugLine(I, v[i], "green");
    }
    debugPoint(I, "blue");
    return I;
}

void VHR::findViewLimitVertices(Point point, std::vector<BoundaryNode> polygon, int &i1, int &i2) {
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

int VHR::getVertexIndexInPolygon(Point t, vector<Point> p) {
    for (unsigned int i = 0; i < p.size(); i++)
        if (p[i] == t)
            return i;
    return NaN;
}

double VHR::calculatePathLength(vector<Point> p) {
    double length = 0;
    for (unsigned int i = 0; i < p.size() - 1; i++) {
        length += G::distance(p[i], p[i+1]);
    }
    return length;
}

// sDist1 = distance(S, p[s1]), sDist2 = distance(S, p[s2]); dDist1 = distance(D, p[d1]) dDist2 = distance(D, p[d2])
vector<Point> VHR::bypassHole(double sDist1, double sDist2, double dDist1, double dDist2, int s1, int s2, int d1, int d2, vector<Point> p) {
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
vector<Point> VHR::determineCaveContainingNode(Point s) {
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

void VHR::dumpBroadcastRegion() {
    // FILE *fp = fopen("BroadcastRegion.tr", "a+");
    // fprintf(fp, "%d\t%f\t%f\n", &selfLocation->id(), &selfLocation->x_, &selfLocation->y_);
    // fclose(fp);
}

void VHR::dumpScale(Point nAScale) {
    // FILE *fp = fopen("Scale.tr", "a+");
    // fprintf(fp, "%f\t%f\n", nAScale.x_, nAScale.y_);
    // fclose(fp);
}

