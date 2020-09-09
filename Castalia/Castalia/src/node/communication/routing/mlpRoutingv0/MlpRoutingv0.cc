#include "MlpRoutingv0.h"
#include <crng.h>
#include "assert.h"

Define_Module(MlpRoutingv0);

int MlpRoutingv0::cnt = 0;
int MlpRoutingv0::nextId;
map<tuple<tuple<Point, Point>, Point, Point, int>, pair<vector<Point>, int> > MlpRoutingv0::outCavernCache;
map<tuple<Point, Point, double>, vector<Point> > MlpRoutingv0::aroundHoleCache;
map<tuple<Point, Point, int, double, int>, vector<Point> > MlpRoutingv0::findPathCache;

void MlpRoutingv0::startup(){
  nextId = 0; // static member
  setTimer(DISCOVER_HOLE_START, 1);
  receivedHole = false;
}

void MlpRoutingv0::timerFiredCallback(int index){
  switch(index){
    case DISCOVER_HOLE_START: {
      double minAngle = 3 * M_PI;
      Point chosenCenter;
      for (auto &n: neighborTable) {
        for (auto center: G::centers(selfLocation, n.location, RADIO_RANGE / 2)) {
          bool ok = true;
          for (auto on: neighborTable) {
            if (on.id == n.id) continue;
            if (G::distance(center, on.location) <= RADIO_RANGE / 2) {
              ok = false;
            }
          }

          if (ok) {
            if (G::norm(atan2(center.y() - selfLocation.y(), center.x() - selfLocation.x())) < minAngle) {
              minAngle = G::norm(atan2(center.y() - selfLocation.y(), center.x() - selfLocation.x()));
              chosenCenter = center;
            }
          }
        }
      }

      if (!chosenCenter.isUnspecified()) {
        Point nextCenter;
        int nextHop = G::findNextHopRollingBall(selfLocation, chosenCenter, RADIO_RANGE / 2, neighborTable, nextCenter);
        if (nextHop != -1) {
          Point nextHopLocation = GlobalLocationService::getLocation(nextHop);
          DiscoverHolePacket *discoverHolePacket = new DiscoverHolePacket("Discover hole packet", NETWORK_LAYER_PACKET);
          discoverHolePacket->setOriginatorId(self);
          discoverHolePacket->setPreviousId(self); // unspecified previous point
          discoverHolePacket->setPath(Util::intVectorToString({self}).c_str());
          discoverHolePacket->setBallCenter(nextCenter);
          // debugLine(selfLocation, GlobalLocationService::getLocation(nextHop), "black");
          toMacLayer(discoverHolePacket, nextHop);
        }
      }

      break;
    }
    default: break;
  }
}


void MlpRoutingv0::processBufferedPacket(){
  while (!TXBuffer.empty()) {
    toMacLayer(TXBuffer.front(), BROADCAST_MAC_ADDRESS);
    TXBuffer.pop();
  }
}

void MlpRoutingv0::fromApplicationLayer(cPacket * pkt, const char *destination){
  MlpPacket *dataPacket = new MlpPacket("MLP routing data packet", NETWORK_LAYER_PACKET);
  encapsulatePacket(dataPacket, pkt);
  dataPacket->setMlpPacketKind(MLP_DATA_PACKET);
  dataPacket->setSource(SELF_NETWORK_ADDRESS);
  dataPacket->setDestination(destination);
  dataPacket->setTTL(1000);


  if (string(destination).compare(BROADCAST_NETWORK_ADDRESS)==0) {
    toMacLayer(dataPacket, BROADCAST_MAC_ADDRESS);
    return;
  }


  dataPacket->setDestLocation(GlobalLocationService::getLocation(atoi(destination)));
  dataPacket->setPacketId(nextId++);


  processDataPacket(dataPacket);
}



void MlpRoutingv0::fromMacLayer(cPacket * pkt, int macAddress, double rssi, double lqi){
  DiscoverHolePacket *discoverHolePacket = dynamic_cast <DiscoverHolePacket*>(pkt);
  if (discoverHolePacket) {
    processDiscoverHolePacket(discoverHolePacket);
    return;
  }



  MlpPacket *netPacket = dynamic_cast <MlpPacket*>(pkt);
  if (!netPacket)
    return;

  trace() << "WSN_EVENT RECEIVE DATA packetId:" << netPacket->getPacketId() << " source:" << netPacket->getSource()
        << " destination:" << netPacket->getDestination() << " current:" << self;

  switch (netPacket->getMlpPacketKind()) {
    case MLP_DATA_PACKET:
      {
        string dst(netPacket->getDestination());
        string src(netPacket->getSource());
        // if the node is the destination
        if ((dst.compare(SELF_NETWORK_ADDRESS) == 0)) {
          toApplicationLayer(netPacket->decapsulate());
          return;
        }
        // if the node is the destination by broadcast, we do not forward it
        if ((dst.compare(BROADCAST_NETWORK_ADDRESS) == 0)) {
          toApplicationLayer(netPacket->decapsulate());
          return;
        }

        MlpPacket *pkt = netPacket->dup();
        processDataPacket(pkt);
        break;
      }

    default: return;
  }
}

void MlpRoutingv0::finishSpecific() {
  trace() << "WSN_EVENT FINAL" << " id:" << self << " x:" << selfLocation.x() << " y:" << selfLocation.y() << " deathTime:-1";
//  log() << "done man ";
}

void MlpRoutingv0::processHole(DiscoverHolePacket* pkt) {


  Point ballCenter = pkt->getBallCenter();
  string pathString(pkt->getPath());
  vector<int> path = Util::stringToIntVector(pathString);
  vector<Point> points;
  for (auto id: path) {
    points.push_back(GlobalLocationService::getLocation(id));
  }
  if (!G::pointInPolygon(points, ballCenter)) {
    return; // outside boundary
  }

  vector<Point> convexHull = G::convexHull(points);
  if (pkt->getHole().empty()){

    holeConvexHull = convexHull;
    // check if there is a really significant "cave"
    int pi = 0;
    for (int i = 0; i < points.size(); i++) {
      if (points[i] == convexHull[0]) {
        pi = i;
        break;
      }
    }

    for (int i = 0; i < convexHull.size(); i++) {
      // invariant: at the beginning of the loop, points[pi] == convexHull[gate1]
      int gate1 = i;
      int gate2 = (i + 1) % convexHull.size();
      double maxDistance = 0;
      int startId = pi;
      while (points[pi] != convexHull[gate2]) {
        double distance = G::distanceToLineSegment(convexHull[gate1], convexHull[gate2], points[pi]);
        if (distance > maxDistance) {
          maxDistance = distance;
        }
        pi = (pi + 1) % points.size();
      }
      int endId = pi;

      if (maxDistance > 3 * RADIO_RANGE) {
        // cave points is points within the cave (between two gate)
        vector<CavernNode> caveNodes;
        vector<Point> cavePoints;
        CavernNode cp_start;
        cp_start.x_ = points[startId].x_;
        cp_start.y_ = points[startId].y_;
        cp_start.id_ = startId;
        caveNodes.push_back(cp_start);
        int j = startId;
        while (j != endId) {
          j = (j + 1) % points.size();
          CavernNode cp;
          cp.x_ = points[j].x_;
          cp.y_ = points[j].y_;
          cp.id_ = j;
          caveNodes.push_back(cp);
          cavePoints.push_back(points[j]);
        }
        double diameter = G::diameter(cavePoints);
        for (int j=0; j<caveNodes.size(); j++){
          caveNodes[j].level_ = computeLevel(points, caveNodes[j].id_, diameter);
          trace() << "id_ " << caveNodes[j].id_;
        }
        caverns.push_back(caveNodes);

      }

    }
  }
  else {
    hole = pkt->getHole();
    caverns = pkt->getCaverns();
  }

  receivedHole = true;
  hole = points;
  holeDiameter = G::diameter(convexHull);
  double distanceToHole = G::distanceToPolygon(convexHull, selfLocation);

  pkt->setHole(hole);
  pkt->setCaverns(caverns);

  // if (distanceToHole < 20 * log2(holeDiameter) || G::pointInOrOnPolygon(convexHull, selfLocation)) {
    propagateHole(pkt);
  // }

  if (self == 731) {
    debugPolygon(hole, "#8d168f");
//    debugPolygon(convexHull, "#8d168f");

//    findPathOutCavern(GlobalLocationService::getLocation(1622),
//     GlobalLocationService::getLocation(2796), hole, caverns[0], 25);
  }

}


void MlpRoutingv0::processDiscoverHolePacket(DiscoverHolePacket* pkt){
  string dst(pkt->getDestination());
  string src(pkt->getSource());

  if ((dst.compare(BROADCAST_NETWORK_ADDRESS) == 0)) {
    if (!receivedHole) {
      processHole(pkt);
    }
    return;
  }

  int originatorId = pkt->getOriginatorId();
  int previousId = pkt->getPreviousId();
  Point ballCenter = pkt->getBallCenter();
  string pathString(pkt->getPath());
  vector<int> path = Util::stringToIntVector(pathString);

  if (smallestOriginatorId.find(previousId) != smallestOriginatorId.end()) {
    if (smallestOriginatorId[previousId] < originatorId) {
      // drop the packet
      return;
    } else {
      smallestOriginatorId[previousId] = originatorId;
    }
  } else {
    smallestOriginatorId[previousId] = originatorId;
  }


  Point nextCenter;
  int nextHop = G::findNextHopRollingBall(selfLocation, ballCenter, RADIO_RANGE / 2, neighborTable, nextCenter);

  if (nextHop != -1) {
    if (path.size() >= 2) {
      if (path[0] == self && path[1] == nextHop) {
        if (path.size() > 20) {
          // TODO
          processHole(pkt);
        }
        return;
      }
    }


    path.push_back(self);
    DiscoverHolePacket *newPkt = pkt->dup();
    newPkt->setPreviousId(self);
    newPkt->setBallCenter(nextCenter);
    newPkt->setPath(Util::intVectorToString(path).c_str());
    // debugLine(selfLocation, GlobalLocationService::getLocation(nextHop), "black");
    toMacLayer(newPkt, nextHop);
  }

}


void MlpRoutingv0::propagateHole(DiscoverHolePacket *pkt) {
  DiscoverHolePacket *dup = pkt->dup();
  dup->setDestination(BROADCAST_NETWORK_ADDRESS);
  toMacLayer(dup, BROADCAST_MAC_ADDRESS);
}

void MlpRoutingv0::processDataPacket(MlpPacket* pkt){
  std::clock_t start_time = std::clock();

  trace() << "PROCESS HOLE";
  Point destLocation = pkt->getDestLocation();
  if (pkt->getNextStoppingPlace().isUnspecified()) {
    trace() << "getNextStoppingPlace isUnspecified";
    if (receivedHole) {
      // first time in
      vector<Point> path = findPath(selfLocation, destLocation, hole, caverns, pkt->getPacketId());
      // findPathCache[make_tuple(selfLocation, destLocation, outDelta, aroundHoleRadius, inDelta)] = path;
      // debugPath(path, "red");
      pkt->setPath(path);
      pkt->setNextStoppingPlace(path[1]);
      pkt->setNextStoppingPlaceId(1);
      pkt->setRoutingMode(MLP_GREEDY_ROUTING);

      processDataPacket(pkt);
    }
    else {
      // still outside
      if (pkt->getRoutingMode() == MLP_GREEDY_ROUTING) {
        int nextHop = -1;
        double minDist = G::distance(selfLocation, destLocation);

        for (auto &neighbor: neighborTable) {
          double dist = G::distance(destLocation, neighbor.location);

          if (dist < minDist) {
            minDist = dist;
            nextHop = neighbor.id;
          }
        }
        if (nextHop != -1) {
          debugLine(selfLocation, GlobalLocationService::getLocation(nextHop), "black");
          trace() << "WSN_EVENT SEND packetId:" << pkt->getPacketId() << " source:" << pkt->getSource()
        << " destination:" << pkt->getDestination() << " current:" << self;
          toMacLayer(pkt, nextHop);
        }
      }
    }
  }
  else {
    trace() << "heading to the next stopping place";
    // heading to the next stopping place
    Point nextStoppingPlace = pkt->getNextStoppingPlace();
    if (reached(nextStoppingPlace)) {
      trace() << "reached nextStoppingPlace";
      vector<Point> path = pkt->getPath();
      int j = pkt->getNextStoppingPlaceId();
      pkt->setNextStoppingPlace(path[j + 1]);
      pkt->setNextStoppingPlaceId(j + 1);
      processDataPacket(pkt);
      
    } else {
      if (pkt->getRoutingMode() == MLP_GREEDY_ROUTING) {
        trace() << "MLP_GREEDY_ROUTING";
        int nextHop = -1;
        double minDist = G::distance(selfLocation, nextStoppingPlace);

        for (auto &neighbor: neighborTable) {
          double dist = G::distance(nextStoppingPlace, neighbor.location);

          if (dist < minDist) {
            minDist = dist;
            nextHop = neighbor.id;
          }
        }
        if (nextHop != -1) {
          debugLine(selfLocation, GlobalLocationService::getLocation(nextHop), "black");
          trace() << "WSN_EVENT SEND packetId:" << pkt->getPacketId() << " source:" << pkt->getSource()
        << " destination:" << pkt->getDestination() << " current:" << self;
          toMacLayer(pkt, nextHop);
        }
      }
    }
  }
  trace() << "computation time " << double(std::clock() - start_time);
}


// will handle interaction between the application layer and the GPRS module in order to pass parameters such as
// the node's position
void MlpRoutingv0::handleNetworkControlCommand(cMessage *msg) {

}

vector<Point> MlpRoutingv0::findPath(Point from, Point to, vector<Point> &hole, vector<vector<CavernNode>> &caverns, int id) {
  vector<Point> result;

  // strange case
  if (hole.empty()) {
    vector<Point> path = {from, to};
    return path;
  }

  // go straight
  if (G::outOrOnPolygon(hole, LineSegment(from, to))) {
    vector<Point> path = {from, to};
    return path;
  }

  // in the same cavern
  for (auto cavern: caverns) {
    if (G::pointInOrOnPolygon(cavern, from) && G::pointInOrOnPolygon(cavern, to)) {
      auto result = G::shortestPathInOrOnPolygon(cavern, from, to);
      return result;
    }
  }

  vector<Point> outCavern = {};
  vector<Point> inCavern = {};
  Point newFrom = from, newTo = to;

  int delta = getRandomNumber(0, RANGE);
  int side_out = 0;
  log () << rand();
  log () << "delta " << delta;

  for (auto cavern: caverns) {
    if (G::pointInOrOnPolygon(cavern, from)) {
      auto outCavernPair = findPathOutCavern2(from, to, hole, cavern, delta);
      outCavern = outCavernPair.first;
      side_out = outCavernPair.second;
      // debugPath(outCavern, "red");
      newFrom = outCavern[outCavern.size() - 1];
      break;
    }
  }

  int path = getRandomNumber(0, RANGE);
  if (side_out == 1) path = delta;
  else if (side_out == -1) path = RANGE-delta;

  for (auto cavern: caverns) {
    if (G::pointInOrOnPolygon(cavern, to)) {
      inCavern = findPathOutCavern2(to, from, hole, cavern, path).first;
      reverse(inCavern.begin(), inCavern.end());
      newTo = inCavern[0];
      break;
    }
  }

  from = newFrom;
  to = newTo;

  // double maxRadius = 5 * log2(holeDiameter);

  //for (int i=0; i<5; i++) trace() << "OK4";
  // trace() << "aroundHoleRadius path " << path;
  vector<Point> middlePath = findPathAroundHole(newFrom, newTo, hole, path);
  //for (int i=0; i<5; i++) trace() << "OK4";
  // debugPath(middlePath, "blue");

  // trace() << "MERGE";
  // trace() << "from " << GlobalLocationService::getId(from) << "to " << GlobalLocationService::getId(to);
  if (outCavern.empty()){
    result.insert(result.end(), middlePath.begin(), middlePath.end());
  }
  else {
    result.insert(result.end(), outCavern.begin(), outCavern.end()-1);
    if (middlePath.size()>1) result.insert(result.end(), middlePath.begin()+1, middlePath.end());
  }
  if (!inCavern.empty()){
    result.insert(result.end(), inCavern.begin()+1, inCavern.end());
  }
  debugPath(result, "red");

  auto flattenResult = G::flatten(result);

  return flattenResult;
}

pair<vector<Point>, int> MlpRoutingv0::findPathOutCavern2(Point from, Point to, vector<Point> &hole,
                                vector<CavernNode> &cavern, int delta) {
  // delta from -RANGE to RANGE

  vector<Point> cavernPoints;
  for (Point p : cavern) cavernPoints.push_back(p);
  tuple<Point, Point> cavernHash = G::hash(cavernPoints);
  if (outCavernCache.find(make_tuple(cavernHash, from, to, delta)) != outCavernCache.end()) {
    return outCavernCache[make_tuple(cavernHash, from, to, delta)];
  }

  vector<int> shortestPath;
  int nHole = hole.size();
  vector<Point> convexHull = G::convexHull(hole);

  int startCavern;
  for (int i=0; i<nHole; i++){
    if (cavern[0] == hole[i]) startCavern = i;
  }
  if (!G::isPointInsidePolygon(to, convexHull)) {
    int d1,d2;
    findViewLimitVertices(to, convexHull, d1, d2);

    DGraph *graph1 = new DGraph(cavernPoints);
    vector<int> sp1Index = graph1->shortestPath2(from, cavern[0]);
    DGraph *graph2 = new DGraph(cavernPoints);
    vector<int> sp2Index = graph2->shortestPath2(from, cavern[cavern.size() - 1]);
    delete(graph1);
    delete(graph2);

    for (int i=0; i<3; i++) trace() << "OK";

    vector<Point> sp1;
    for (int i=1; i<sp1Index.size(); i++) sp1.push_back(cavern[sp1Index[i]]);
    vector<Point> sp2;
    for (int i=1; i<sp2Index.size(); i++) sp2.push_back(cavern[sp2Index[i]]);
    // debugPath(sp1, "blue");
    // debugPath(sp2, "blue");
    for (int i=0; i<3; i++) trace() << "OK2";

    vector<Point> sp_ = bypassHole(calculatePathLength(sp1),
        calculatePathLength(sp2), G::distance(to, convexHull[d1]),
        G::distance(to, convexHull[d2]),
        G::getVertexIndexInPolygon(cavern[0], convexHull),
        G::getVertexIndexInPolygon(cavern[cavern.size() - 1], convexHull), d1,
        d2, convexHull);

    // debugPath(sp_, "green");

    if (sp1.back() == sp_.front()) {
        // for (int i=1; i<sp1Index.size(); i++) shortestPath.push_back((sp1Index[i]+startCavern)%nHole);
        for (int i=1; i<sp1Index.size(); i++) shortestPath.push_back(sp1Index[i]);
    } else if (sp2.back() == sp_.front()) {
        // for (int i=1; i<sp2Index.size(); i++) shortestPath.push_back((sp2Index[i]+startCavern)%nHole);
        for (int i=1; i<sp2Index.size(); i++) shortestPath.push_back(sp2Index[i]);
    }
  }
  else {
    vector<CavernNode> cavern2;
    for (int i=0; i<caverns.size(); i++) {
      if (G::pointInOrOnPolygon(caverns[i], to)) {
        cavern2 = caverns[i];
      }
    }

    vector<int> s_sp1Index = G::shortestPathInOrOnPolygon2(cavern, from, cavern[0]);
    vector<int> s_sp2Index = G::shortestPathInOrOnPolygon2(cavern, from, cavern[cavern.size() - 1]);
    vector<int> d_sp1Index = G::shortestPathInOrOnPolygon2(cavern2, cavern2[cavern2.size() - 1], to);
    vector<int> d_sp2Index = G::shortestPathInOrOnPolygon2(cavern2, cavern2[0], to);

    vector<Point> s_sp1;
    for (int i : s_sp1Index) s_sp1.push_back(cavern[i]);
    vector<Point> s_sp2;
    for (int i : s_sp2Index) s_sp2.push_back(cavern[i]);
    vector<Point> d_sp1;
    for (int i : d_sp1Index) d_sp1.push_back(cavern[i]);
    vector<Point> d_sp2;
    for (int i : d_sp2Index) d_sp2.push_back(cavern[i]);

    vector<Point> sp_ = bypassHole(calculatePathLength(s_sp1),
        calculatePathLength(s_sp2), calculatePathLength(d_sp1),
        calculatePathLength(d_sp2),
        G::getVertexIndexInPolygon(cavern[0], convexHull),
        G::getVertexIndexInPolygon(cavern[cavern.size() - 1], convexHull),
        G::getVertexIndexInPolygon(cavern2[cavern2.size() - 1], convexHull),
        G::getVertexIndexInPolygon(cavern2[0], convexHull), convexHull);

    if (s_sp1.back() == sp_.front()) {
        // for (int i : s_sp1Index) shortestPath.push_back((i+startCavern)%nHole);
        shortestPath = s_sp1Index;
    } else if (s_sp2.back() == sp_.front()) {
        // for (int i : s_sp2Index) shortestPath.push_back((i+startCavern)%nHole);
        shortestPath = s_sp2Index;
    }
  }

  // shortestPath = G::shortestPathOutOrOnPolygon2(hole, from, to);
  // vector<Point> shortestPathPoint = G::shortestPathOutOrOnPolygon(hole, from, to);
  // debugPath(shortestPathPoint, "purple");
  vector<int> L_out = shortestPath;
  vector<Point> shortestPathPoint;
  // for (int i : shortestPath) shortestPathPoint.push_back(hole[i]);
  for (int i : shortestPath) shortestPathPoint.push_back(cavern[i]);
  double l_out = calculatePathLength(shortestPathPoint);

  // // gate
  // Point M = cavern[0], N = cavern[cavern.size() - 1];
  // // debugPoint(M, "green");
  // // debugPoint(N, "green");
  // LineSegment MN(M, N);
  // Point I;
  // for (int i = 0; i < shortestPath.size() - 1; i++) {
  //   Point X = hole[shortestPath[i]], Y = hole[shortestPath[i + 1]];
  //   LineSegment XY(X, Y);
  //   // debugPoint(X, "blue");
  //   L_out.push_back(shortestPath[i]);
  //   l_out += G::distance(X,Y);
  //   if (G::doIntersect(XY, MN)) {
  //     I = G::intersection(XY, MN);
  //     L_out.push_back(shortestPath[i+1]);
  //     break;
  //   }
  // }

  // for (int k : L_out){
  //   debugPoint(hole[k], "purple");
  // }

  //for (int i=0; i<5; i++) trace() << "OK";

  Angle theta = 0;
  // for (int i = 0; i < L_out.size(); i++){
  //   Point X_i = hole[L_out[i]];
  //   Point X_i1 = hole[(L_out[i]-1+nHole)%nHole];
  //   Point X_i2 = hole[(L_out[i]+1)%nHole];
  //   theta += 2*M_PI - G::angle(X_i, X_i1, X_i, X_i2);
  // }
  double baseK = G::distance(cavern[0], cavern[cavern.size() - 1]) / 4;
  trace() << "baseK " << baseK;

  // double r = baseK / RANGE * delta;
  vector<Point> result;
  result.push_back(from);
  //for (int i=0; i<5; i++) trace() << "OK";

  // if stay inside deep caverns
  int startShift = 0;
  /*Point SP_0 = hole[L_out[0]];
  Point SP_1 = hole[L_out[1]];
  Vector v_0(SP_0, SP_1);
  int sign_0 = G::isMonotoneLine(L_out[0], L_out[1], hole);
  trace() << "sign " << sign_0;
  // trace() << G::isMonotoneLine(L_out[0], L_out[1], hole);
  // trace() << G::isMonotoneLine(L_out[1], L_out[0], hole);
  Point SP_01 = SP_0 + v_0.rotate(M_PI / 2) * (baseK * sign_0 / v_0.length());
  Point SP_02 = SP_0 + v_0.rotate(M_PI / 2) * (baseK * sign_0 / v_0.length() / 2);
  debugLine(SP_01, SP_02, "green");
  // debugPoint(SP_0, "blue");
  // debugPoint(SP_1, "blue");
  // debugPoint(SP_01, "green");
  // debugPoint(SP_02, "green");
  LineSegment XY(SP_01, SP_02);
  if (G::is_intersect(XY, hole)){
    // for (int ii=0; ii<5; ii++) trace() << "is_intersect";
    int i;
    for (i=1; i<L_out.size() - 1; i++){
      Point SP_i = hole[L_out[i]];
      Vector v_i1(SP_i, hole[L_out[i-1]]);
      int sign_i1 = G::isMonotoneLine(L_out[i], L_out[i-1], hole);
      // trace() << "sign " << sign_i1;
      Point SP_i1 = SP_i + v_i1.rotate(M_PI / 2) * (baseK * sign_i1 / v_i1.length());
      Vector v_i2(SP_i, hole[L_out[i+1]]);
      int sign_i2 = G::isMonotoneLine(L_out[i], L_out[i+1], hole);
      // trace() << "sign " << sign_i2;
      Point SP_i2 = SP_i + v_i2.rotate(M_PI / 2) * (baseK * sign_i2 / v_i2.length());
      DirectedArc arc_i(SP_i1, SP_i2, SP_i, r);
      if (!G::is_intersect(arc_i, hole)) break;
      debugPoint(SP_i, "black");
      debugPoint(SP_i1, "blue");
      debugPoint(SP_i2, "green");
      result.push_back(SP_i);
    }
    if (i<L_out.size() - 1) startShift = i;
    else {
      result.push_back(hole[L_out[L_out.size() - 1]]);
      return std::make_pair(G::flatten(result), 0);
    }
  }*/

  // trace() << "OK2";
  // trace() << startShift;

  // debugPath(result, "black");

  vector<int> monoIndex;
  int side = 0;
  for (int i = startShift; i < L_out.size(); i++){
    // trace() << "index " << i;
    // for (int k : monoIndex){
    //   trace() << k;
    // }
    if (monoIndex.size() == 0){
      monoIndex.push_back(L_out[i]);
    }
    else {
      int p_i1 = monoIndex[monoIndex.size()-1];
      int p_i2 = L_out[i];
      if (i == L_out.size()-1) {
        monoIndex.push_back(L_out[i]);
        if (monoIndex.size() == 2) side = G::isMonotoneLine(cavern[monoIndex[0]].id_, cavern[monoIndex[1]].id_, hole);
        // trace() << L_out[i];
      }
      if ((G::isMonotoneLine(cavern[p_i1].id_, cavern[p_i2].id_, hole) == 0) || ((i == L_out.size()-1) && (monoIndex.size() >= 2))){
        vector<Point> monoLine;
        double kmax = baseK;
        for (int j=0; j<monoIndex.size(); j++){
          monoLine.push_back(cavern[monoIndex[j]]);
          // if (cavern[monoIndex[j]].level_ < kmax) kmax = cavern[monoIndex[j]].level_;
          // if (side < 0) debugPoint(cavern[monoIndex[j]], "blue");
          // else debugPoint(cavern[monoIndex[j]], "red");
        }
        double r = kmax / RANGE * delta;
        double translateFactor = r;
        if (side < 0) translateFactor = -r;
        trace() << "kmax " << kmax << " translateFactor " << translateFactor;
        auto translatedLine = G::translate(monoLine, translateFactor);
        for (auto p: translatedLine){
          result.push_back(p);
        }
        monoIndex.clear();
      }
      if (i < L_out.size()-1) monoIndex.push_back(L_out[i]);
      if (monoIndex.size() == 2) side = G::isMonotoneLine(cavern[monoIndex[0]].id_, cavern[monoIndex[1]].id_, hole);
    }
  }

  // for (int i=0; i<hole.size(); i+=20) {
  //   debugPoint(hole[i], "gray");
  // }
  // debugPoint(hole[0], "black");
  //for (int i=0; i<5; i++) trace() << "OK3";

  if (!monoIndex.empty()){
    result.push_back(hole[cavern[monoIndex[0]].id_]);
  }

  auto flattenResult = G::flatten(result);

  flattenResult.push_back(cavern[L_out[L_out.size()-1]]);
  auto result_pair = std::make_pair(flattenResult, side);
  outCavernCache[make_tuple(cavernHash, from, to, delta)] = result_pair;

  return result_pair;
}


// vector<Point> MlpRoutingv0::findPathAroundHole(Point from, Point to, vector<Point> &hole, double r) {

//   vector<Point> convexHull = G::convexHull(hole);
//   //for (int i=0; i<5; i++) trace() << "OK4.0";
//   auto closestFromConvexHull = from, closestToConvexHull = to;
//   if (G::pointInOrOnPolygon(convexHull, from)) {
//     closestFromConvexHull = G::closestPointOnPolygon(convexHull, from);
//   }
//   if (G::pointInOrOnPolygon(convexHull, to)) {
//     closestToConvexHull = G::closestPointOnPolygon(convexHull, to);
//   }
//   //for (int i=0; i<5; i++) trace() << "OK4.1";

//   int n = convexHull.size();
//   vector<int> L_arround = G::shortestPathOutOrOnPolygon2(convexHull, closestFromConvexHull, closestToConvexHull);
//   //for (int i=0; i<5; i++) trace() << "OK4.2";
//   double theta = 0;
//   double l_arround = 0;
//   for (int i=0; i<L_arround.size(); i++){
//     trace() << i << " " << L_arround.size();
//     Point X_i = convexHull[L_arround[i]];
//     Point X_i1 = convexHull[(L_arround[i]-1+n)%n];
//     Point X_i2 = convexHull[(L_arround[i]+1)%n];    
//     theta += 2*M_PI - G::angle(X_i, X_i1, X_i, X_i2);
//     if (i < L_arround.size()-1){
//       Point X_n = convexHull[L_arround[i+1]];
//       l_arround += G::distance(X_i, X_n);
//     }
//   }
//   double maxRadius = eps*l_arround/2/theta;
//   double ballRadius = maxRadius / RANGE * r;
//   trace() << "ballRadius " << ballRadius;

//   if (aroundHoleCache.find(make_tuple(from, to, ballRadius)) != aroundHoleCache.end()) {
//     return aroundHoleCache[make_tuple(from, to, ballRadius)];
//   }

//   if (G::outOrOnPolygon(hole, LineSegment(from, to))) {
//     vector<Point> result = {from, to};
//     return result;
//   }

//   vector<Point> balledConvexHull = G::rollBallPolygon(convexHull, ballRadius);
//   vector<Point> res;
//   auto closestFrom = from, closestTo = to;
//   if (G::pointInOrOnPolygon(balledConvexHull, from)) {
//     closestFrom = G::closestPointOnPolygon(balledConvexHull, from);
//   }
//   if (G::pointInOrOnPolygon(balledConvexHull, to)) {
//     closestTo = G::closestPointOnPolygon(balledConvexHull, to);
//   }

//   if (closestFrom != from) {
//     for (auto p: G::shortestPathOutOrOnPolygon(hole, from, closestFrom)) {
//       res.push_back(p);
//     }
//   }

//   for (auto p: G::shortestPathOutOrOnPolygon(balledConvexHull, closestFrom, closestTo)) {
//     res.push_back(p);
//   }

//   if (closestTo != to) {
//     for (auto p: G::shortestPathOutOrOnPolygon(hole, closestTo, to)) {
//       res.push_back(p);
//     }
//   }

//   auto flattenResult = G::flatten(res);

//   aroundHoleCache[make_tuple(from, to, ballRadius)] = flattenResult;
//   return flattenResult;
// }

vector<Point> MlpRoutingv0::findPathAroundHole(Point from, Point to, vector<Point> &hole, double r) {

  int nHole = hole.size();
  vector<Point> convexHull = G::convexHull(hole);
  int startConvex;
  for (int i=0; i<nHole; i++){
    if (convexHull[0] == hole[i]) startConvex = i;
  }
  vector<int> mapping;
  mapping.push_back(startConvex);
  int next = 1;
  for (int i=1; i<nHole; i++){
    if (hole[(i+startConvex)%nHole] == convexHull[next]){
      mapping.push_back((i+startConvex)%nHole);
      next++;
    }
  }
  if (mapping.size() != convexHull.size()) trace() << "ERROR";

  int s1, s2, d1, d2;

  if (G::isPointInsidePolygon(from, convexHull)){
    for (int i=0; i<convexHull.size(); i++){
      if (convexHull[i] == from){
        s1 = i; s2 = i;
      }
    }
  }
  else {
    findViewLimitVertices(from, convexHull, s1, s2);
  }

  if (G::isPointInsidePolygon(to, convexHull)){
    for (int i=0; i<convexHull.size(); i++){
      if (convexHull[i] == to){
        d1 = i; d2 = i;
      }
    }
  }
  else {
    findViewLimitVertices(to, convexHull, d1, d2);
  }
  vector<int> sp = bypassHole2(G::distance(from, convexHull[s1]), G::distance(from, convexHull[s2]),
      G::distance(to, convexHull[d1]), G::distance(to, convexHull[d2]), s1, s2, d1, d2, convexHull);

  vector<int> L_arround;
  for (int i : sp) L_arround.push_back(mapping[i]);
  
  int n = hole.size();
  double theta = 0;
  double l_arround = 0;
  for (int i=0; i<L_arround.size(); i++){
    Point X_i = hole[L_arround[i]];
    Point X_i1 = hole[(L_arround[i]-1+n)%n];
    Point X_i2 = hole[(L_arround[i]+1)%n];    
    theta += 2*M_PI - G::angle(X_i, X_i1, X_i, X_i2);
    if (i < L_arround.size()-1){
      Point X_n = hole[L_arround[i+1]];
      l_arround += G::distance(X_i, X_n);
    }
  }
  l_arround += G::distance(from, hole[L_arround[0]]) + G::distance(hole[L_arround[L_arround.size()-1]], to);

  //for (int i=0; i<5; i++) trace() << "OK5";

  double baseK = 10 * log2(holeDiameter);
  // trace() << "baseK " << baseK;
  // trace() << "r " << r;
  vector<Point> result;
  if (L_arround.empty()){
    result.push_back(from);
    result.push_back(to);
    // debugPath(result, "green");
    return result;
  }
  else if (L_arround.size() == 1){
    result.push_back(from);
    Point p = hole[L_arround[0]];
    if ((p != from) && (p != to)) result.push_back(p);
    result.push_back(to);
    // debugPath(result, "green");
    return result;
  }

  if (from != hole[L_arround[0]]) result.push_back(from);

  double sign = G::isMonotoneLine(L_arround[0], L_arround[1], hole);
  // trace() << "sign " << sign;
  vector<Point> L_arroundPath;
  for (int j=0; j<L_arround.size(); j++){
    L_arroundPath.push_back(hole[L_arround[j]]);
  }
  auto translatedPath = G::translate(L_arroundPath, baseK / RANGE * r * sign);
  for (auto p: translatedPath){
    result.push_back(p);
    // debugPoint(p, "green");
  }

  if (to != hole[L_arround[L_arround.size()-1]]) result.push_back(from);

  //for (int i=0; i<5; i++) trace() << "OK6";

  auto flattenResult = G::flatten(result);
  // debugPath(flattenResult, "green");
  return flattenResult;
}

vector<Point> MlpRoutingv0::bypassHole(double sDist1, double sDist2, double dDist1, double dDist2, int s1, int s2, int d1, int d2, vector<Point> p) {
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

vector<int> MlpRoutingv0::bypassHole2(double sDist1, double sDist2, double dDist1, double dDist2, int s1, int s2, int d1, int d2, vector<Point> p) {
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
    vector<int> sp1, sp2, sp3, sp4, sp;
    //============================================= S S1 D1 D
    t1 = sDist1;
    if (d1 < s1) d1 += p.size();
    for (int i = s1; i < d1; i++) {
        t1 += G::distance(p[i % p.size()], p[(i+1) % p.size()]);
        sp1.push_back(i % p.size());
    }
    t1 += dDist1;
    sp1.push_back(d1 % p.size());
    min = t1; sp = sp1;

    //============================================= S S2 D2 D
    t2 = sDist2;
    if (d2 < s2) d2 += p.size();
    for (int i = s2; i < d2; i++) {
        t2 += G::distance(p[i % p.size()], p[(i+1) % p.size()]);
        sp2.push_back(i % p.size());
    }
    t2 += dDist2;
    sp2.push_back(d2 % p.size());
    if (t2 < min) { min = t2; sp = sp2;}

    //============================================= D D1 S1 S
    t3 = dDist1;
    if (s1 < d1) s1 += p.size();
    for (int i = d1; i < s1; i++) {
        t3 += G::distance(p[i % p.size()], p[(i+1) % p.size()]);
        sp3.push_back(i % p.size());
    }
    t3 += sDist1;
    sp3.push_back(s1 % p.size());
    reverse(sp3.begin(), sp3.end());
    if (t3 < min) { min = t3; sp = sp3;}

    //============================================= D D2 S2 S
    t4 = dDist2;
    if (s2 < d2) s2 += p.size();
    for (int i = d2; i < s2; i++) {
        t4 += G::distance(p[i % p.size()], p[(i+1) % p.size()]);
        sp4.push_back(i % p.size());
    }
    t4 += sDist2;
    sp4.push_back(s2 % p.size());
    reverse(sp4.begin(), sp4.end());
    if (t4 < min) {sp = sp4;}

    return sp;

}

void MlpRoutingv0::findViewLimitVertices(Point point, std::vector<Point> polygon, int &i1, int &i2) {
  vector<BoundaryNode> boundaryPoly;
  for (Point p : polygon){
    BoundaryNode bp;
    bp.x_ = p.x_;
    bp.y_ = p.y_;
    boundaryPoly.push_back(bp);
  }
  std::vector<BoundaryNode*> clone;
  for (unsigned int i = 0; i < boundaryPoly.size(); i++) {
    boundaryPoly[i].id_ = i;
    clone.push_back(&boundaryPoly[i]);
  }

  BoundaryNode pivot;
  pivot.x_ = point.x_;
  pivot.y_ = point.y_;
  std::sort(clone.begin(), clone.end(), POLAR_ORDER(pivot));
  i1 = clone[0]->id_;
  i2 = clone[clone.size()-1]->id_;
}

double MlpRoutingv0::calculatePathLength(vector<Point> p) {
    double length = 0;
    for (unsigned int i = 0; i < p.size() - 1; i++) {
        length += G::distance(p[i], p[i+1]);
    }
    return length;
}

double MlpRoutingv0::computeLevel(vector<Point> hole, int i, double max){
  int n = hole.size();
  Point p = hole[i];
  Point p1 = hole[(i-1+n)%n];
  Point p2 = hole[(i+1)%n];

  // for (int j=2; j<n; j++){
  //   Point p1s = hole[(i-j+n)%n];
  //   if (G::isMonotoneLine(p, p1s, hole) == 1) p1 = p1s;
  //   else break;
  // }
  // for (int j=2; j<n; j++){
  //   Point p2s = hole[(i+j)%n];
  //   if (G::isMonotoneLine(p, p2s, hole) == -1) p2 = p2s;
  //   else break;
  // }
  // if (p == GlobalLocationService::getLocation(4713)){
  //   debugPoint(p1, "green");
  //   debugPoint(p2, "green");
  // }

  Vector V1(p, p1);
  Vector V2(p, p2);
  double radius = G::distance(p1, p);
  double diff = 0;
  while (true){
    // Vector v_i1(SP_i, hole[L_out[i-1]]);
    //   int sign_i1 = G::isMonotoneLine(L_out[i], L_out[i-1], hole);
    //   // trace() << "sign " << sign_i1;
    // Point SP_i1 = SP_i + v_i1.rotate(M_PI / 2) * (baseK * sign_i1 / v_i1.length());
    //   Vector v_i2(SP_i, hole[L_out[i+1]]);
    //   int sign_i2 = G::isMonotoneLine(L_out[i], L_out[i+1], hole);
    //   // trace() << "sign " << sign_i2;
    //   Point SP_i2 = SP_i + v_i2.rotate(M_PI / 2) * (baseK * sign_i2 / v_i2.length());

    Point q1 = p1 + V1.rotate(M_PI / 2) * (-radius / V1.length());
    Point q2 = p2 + V2.rotate(M_PI / 2) * (radius / V2.length());
    DirectedArc arc(q1, q2, p, radius);
    // if (p == GlobalLocationService::getLocation(4770)) {
    //   debugPoint(q1, "green");
    //   debugPoint(q2, "green");
    //   debugArc(q1, q1, radius, "green");
    // }

    // int start = 0;
    // for (int j=1; j<n; j++){
    //   if (G::distance(p, hole[(i+j)%n]) > radius){
    //     start = j;
    //     break;
    //   }
    // }
    // int end = n-1;
    // for (int j=1; j<n; j++){
    //   if (G::distance(p, hole[(i-j+n)%n]) > radius){
    //     end = n-j;
    //     break;
    //   }
    // }
    // bool check = false;
    // for (int j=start; j<=end; j++){
    //   DirectedSegment seg(hole[(i+j)%n], hole[(i+j+1)%n]);
    //   Point intersection;
    //   if (G::is_intersect(seg, arc, intersection)) check = true;
    // }

    // if (!check){
    //   if (diff == 0){
    //     diff = radius/8;
    //     radius -= radius/4;
    //   }
    //   else {
    //     radius -= diff;
    //     diff = diff/2;
    //   }
    // }
    // else {
    //   if (diff == 0) radius += radius;
    //   else {
    //     radius += diff;
    //     diff = diff/2;
    //   }
    // }

    
    // if (diff < 0.01) break;

    if (!G::is_intersect(arc, hole)){
      radius += 20;
    } else {
      radius -= 20;
      break;
    }
    if (radius > max) break;
  }
  // if (p == GlobalLocationService::getLocation(4770)) debugCircle(p, radius, "green");
  return radius;
}
