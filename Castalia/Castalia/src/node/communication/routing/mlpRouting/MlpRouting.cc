#include "MlpRouting.h"
#include <crng.h>
#include "assert.h"

Define_Module(MlpRouting);

int MlpRouting::cnt = 0;
int MlpRouting::nextId;
map<tuple<tuple<Point, Point>, Point, Point, int>, vector<Point> > MlpRouting::outCavernCache;
map<tuple<Point, Point, double>, vector<Point> > MlpRouting::aroundHoleCache;
map<tuple<Point, Point, int, double, int>, vector<Point> > MlpRouting::findPathCache;

void MlpRouting::startup(){
  nextId = 0; // static member
  setTimer(DISCOVER_HOLE_START, 1);
  receivedHole = false;
}

void MlpRouting::timerFiredCallback(int index){
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
          toMacLayer(discoverHolePacket, nextHop);
        }
      }

      break;
    }
    default: break;
  }
}


void MlpRouting::processBufferedPacket(){
  while (!TXBuffer.empty()) {
    toMacLayer(TXBuffer.front(), BROADCAST_MAC_ADDRESS);
    TXBuffer.pop();
  }
}

void MlpRouting::fromApplicationLayer(cPacket * pkt, const char *destination){
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



void MlpRouting::fromMacLayer(cPacket * pkt, int macAddress, double rssi, double lqi){
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

void MlpRouting::finishSpecific() {
  trace() << "WSN_EVENT FINAL" << " id:" << self << " x:" << selfLocation.x() << " y:" << selfLocation.y() << " deathTime:-1";
//  log() << "done man ";
}

void MlpRouting::processHole(DiscoverHolePacket* pkt) {


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
      vector<Point> cavePoints;
      cavePoints.push_back(points[startId]);
      int j = startId;
      while (j != endId) {
        j = (j + 1) % points.size();
        cavePoints.push_back(points[j]);
      }
      caverns.push_back(cavePoints);

    }

  }

  receivedHole = true;
  hole = points;
  holeDiameter = G::diameter(convexHull);
  double distanceToHole = G::distanceToPolygon(convexHull, selfLocation);

  if (distanceToHole < 20 * log2(holeDiameter) || G::pointInOrOnPolygon(convexHull, selfLocation)) {
    propagateHole(pkt);
  }

  if (self == 731) {
    debugPolygon(hole, "#8d168f");
//    debugPolygon(convexHull, "#8d168f");

//    findPathOutCavern(GlobalLocationService::getLocation(1622),
//     GlobalLocationService::getLocation(2796), hole, caverns[0], 25);
  }

}


void MlpRouting::processDiscoverHolePacket(DiscoverHolePacket* pkt){
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
    toMacLayer(newPkt, nextHop);
  }

}


void MlpRouting::propagateHole(DiscoverHolePacket *pkt) {
  DiscoverHolePacket *dup = pkt->dup();
  dup->setDestination(BROADCAST_NETWORK_ADDRESS);
  toMacLayer(dup, BROADCAST_MAC_ADDRESS);
}

void MlpRouting::processDataPacket(MlpPacket* pkt){
  trace() << "PROCESS HOLE";
  Point destLocation = pkt->getDestLocation();
  if (pkt->getNextStoppingPlace().isUnspecified()) {
    trace() << "getNextStoppingPlace isUnspecified";
    if (receivedHole) {
      // first time in
      vector<Point> path;
      double aroundHoleRadius;
      int outDelta, inDelta;
      tie(path, outDelta, aroundHoleRadius, inDelta) = findPath(selfLocation,
        destLocation, hole, caverns);
      findPathCache[make_tuple(selfLocation, destLocation, outDelta, aroundHoleRadius, inDelta)] = path;
      debugPath(path, "red");
      pkt->setOutDelta(outDelta);
      pkt->setAroundHoleRadius(aroundHoleRadius);
      pkt->setInDelta(inDelta);
      pkt->setStartStableLocation(selfLocation);
      pkt->setNextStoppingPlace(path[1]);
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
        } else {

          pkt->setRoutingMode(MLP_ROLLINGBALL_ROUTING);
          pkt->setStuckLocation(selfLocation);

          // compute first ball with radius = RADIO_RANGE/2
          double x1 = selfLocation.x(), y1 = selfLocation.y();
          double x2 = destLocation.x(), y2 = destLocation.y();
          double d = G::distance(selfLocation, destLocation);
          double centerX = x1 + (x2 - x1) * RADIO_RANGE / 2 / d;
          double centerY = y1 + (y2 - y1) * RADIO_RANGE / 2 / d;

          // set ball radius
          pkt->setBallCenter(Point(centerX, centerY));

          processDataPacket(pkt);
        }
      }
      else {
        Point stuckLocation = pkt->getStuckLocation();
        Point destLocation = pkt->getDestLocation();
        if (abs(G::distance(selfLocation, destLocation) - G::distance(stuckLocation, destLocation)) < -EPSILON) {
          pkt->setRoutingMode(MLP_GREEDY_ROUTING);
          processDataPacket(pkt);
        } else {
          int nextHop = -1;
          Point nextCenter;
          Point ballCenter = pkt->getBallCenter();
          nextHop = G::findNextHopRollingBall(selfLocation, ballCenter, RADIO_RANGE / 2, neighborTable, nextCenter);
          if (nextHop != -1) {
            debugLine(selfLocation, GlobalLocationService::getLocation(nextHop), "black");
            trace() << "WSN_EVENT SEND packetId:" << pkt->getPacketId() << " source:" << pkt->getSource()
        << " destination:" << pkt->getDestination() << " current:" << self;
            toMacLayer(pkt, nextHop);
          }
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
//      debugPoint(selfLocation, "green");
      Point startStableLocation = pkt->getStartStableLocation();
      double outDelta = pkt->getOutDelta();
      double aroundHoleRadius = pkt->getAroundHoleRadius();
      double inDelta = pkt->getInDelta();

      if (findPathCache.find(make_tuple(startStableLocation, destLocation, outDelta, aroundHoleRadius, inDelta))
        != findPathCache.end()) {
        vector<Point> path = findPathCache[make_tuple(startStableLocation, destLocation, outDelta,
            aroundHoleRadius, inDelta)];
        int j = -1;
        for (int i = 0; i < path.size() - 1; i++) {
          if (path[i] == nextStoppingPlace) {
            j = i;
            // break;
          }
        }

        if (j != -1) {
          pkt->setNextStoppingPlace(path[j + 1]);
          processDataPacket(pkt);
        }

      } else {
        log() << "not in cache";
      }
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
        } else {
          trace() << "setRoutingMode MLP_ROLLINGBALL_ROUTING";
          pkt->setRoutingMode(MLP_ROLLINGBALL_ROUTING);
          pkt->setStuckLocation(selfLocation);

          // compute first ball with radius = RADIO_RANGE/2
          double x1 = selfLocation.x(), y1 = selfLocation.y();
          double x2 = nextStoppingPlace.x(), y2 = nextStoppingPlace.y();
          double d = G::distance(selfLocation, nextStoppingPlace);
          double centerX = x1 + (x2 - x1) * RADIO_RANGE / 2 / d;
          double centerY = y1 + (y2 - y1) * RADIO_RANGE / 2 / d;

          // set ball radius
          pkt->setBallCenter(Point(centerX, centerY));

          processDataPacket(pkt);
        }
      }
      else {
        Point stuckLocation = pkt->getStuckLocation();
        if (abs(G::distance(selfLocation, nextStoppingPlace) - G::distance(stuckLocation, nextStoppingPlace)) < -EPSILON) {
          trace() << "setRoutingMode MLP_GREEDY_ROUTING";
          pkt->setRoutingMode(MLP_GREEDY_ROUTING);
          processDataPacket(pkt);
        } else {
          trace() << "findNextHopRollingBall";
          int nextHop = -1;
          Point nextCenter;
          Point ballCenter = pkt->getBallCenter();
          nextHop = G::findNextHopRollingBall(selfLocation, ballCenter, RADIO_RANGE / 2, neighborTable, nextCenter);
          if (nextHop != -1) {
            debugLine(selfLocation, GlobalLocationService::getLocation(nextHop), "black");
            trace() << "WSN_EVENT SEND packetId:" << pkt->getPacketId() << " source:" << pkt->getSource()
        << " destination:" << pkt->getDestination() << " current:" << self;
            toMacLayer(pkt, nextHop);
          }
        }
      }
    }
  }
}


// will handle interaction between the application layer and the GPRS module in order to pass parameters such as
// the node's position
void MlpRouting::handleNetworkControlCommand(cMessage *msg) {

}

tuple<vector<Point>, int, double, int> MlpRouting::findPath(Point from, Point to, vector<Point> &hole, vector<vector<Point>> &caverns) {
  vector<Point> result;

  int inDelta = NULL_VAL;
  int outDelta = NULL_VAL;
  double aroundHoleRadius = NULL_VAL;

  // strange case
  if (hole.empty()) {
    vector<Point> path = {from, to};
    return make_tuple(path, outDelta, aroundHoleRadius, inDelta);
  }

  // go straight
  if (G::outOrOnPolygon(hole, LineSegment(from, to))) {
    vector<Point> path = {from, to};
    return make_tuple(path, outDelta, aroundHoleRadius, inDelta);
  }

  // in the same cavern
  for (auto cavern: caverns) {
    if (G::pointInOrOnPolygon(cavern, from) && G::pointInOrOnPolygon(cavern, to)) {
      auto result = G::shortestPathInOrOnPolygon(cavern, from, to);
      return make_tuple(result, outDelta, aroundHoleRadius, inDelta);
    }
  }

  vector<Point> outCavern = {};
  vector<Point> inCavern = {};
  Point newFrom = from, newTo = to;

  int delta = getRandomNumber(-RANGE, RANGE);
  log () << rand();
  log () << "delta " << delta;

  for (auto cavern: caverns) {
    if (G::pointInOrOnPolygon(cavern, from)) {
      outCavern = findPathOutCavern(from, to, hole, cavern, delta);
      newFrom = outCavern[outCavern.size() - 1];
      break;
    }
  }

  for (auto cavern: caverns) {
    if (G::pointInOrOnPolygon(cavern, to)) {
      inCavern = findPathOutCavern(to, from, hole, cavern, delta);
      reverse(inCavern.begin(), inCavern.end());
      newTo = inCavern[0];
      break;
    }
  }

  from = newFrom;
  to = newTo;

  double maxRadius = 5 * log2(holeDiameter);
  int path = getRandomNumber(1, 2 * RANGE);
  aroundHoleRadius = maxRadius / (2 * RANGE) * path;

  trace() << "aroundHoleRadius " << aroundHoleRadius;
  vector<Point> middlePath = findPathAroundHole(newFrom, newTo, hole, aroundHoleRadius);

  result.insert(result.end(), outCavern.begin(), outCavern.end());
  result.insert(result.end(), middlePath.begin(), middlePath.end());
  result.insert(result.end(), inCavern.begin(), inCavern.end());

  auto flattenResult = G::flatten(result);

  return make_tuple(flattenResult, outDelta, aroundHoleRadius, inDelta);
}

vector<Point> MlpRouting::findPathOutCavern2(Point from, Point to, vector<Point> &hole,
                                vector<Point> &cavern, int delta) {
  // delta from -RANGE to RANGE

  tuple<Point, Point> cavernHash = G::hash(cavern);
  if (outCavernCache.find(make_tuple(cavernHash, from, to, delta)) != outCavernCache.end()) {
    return outCavernCache[make_tuple(cavernHash, from, to, delta)];
  }

  vector<int> shortestPath = G::shortestPathOutOrOnPolygon2(hole, from, to);
  vector<Point> shortestPathPoint = G::shortestPathOutOrOnPolygon(hole, from, to);
  debugPath(shortestPathPoint, "purple");
  vector<int> L_out;

  // gate
  Point M = cavern[0], N = cavern[cavern.size() - 1];
  // debugPoint(M, "green");
  // debugPoint(N, "green");
  LineSegment MN(M, N);
  Point I;
  for (int i = 0; i < shortestPath.size() - 1; i++) {
    Point X = hole[shortestPath[i]], Y = hole[shortestPath[i + 1]];
    LineSegment XY(X, Y);
    // debugPoint(X, "blue");
    L_out.push_back(shortestPath[i]);
    if (G::doIntersect(XY, MN)) {
      I = G::intersection(XY, MN);
      L_out.push_back(shortestPath[i+1]);
      break;
    }
  }

  for (int k : L_out){
    debugPoint(hole[k], "purple");
  }
  if (I.isUnspecified()) return vector<Point>(); // not gonna happen

  for (int i=0; i<5; i++) trace() << "OK";
  double baseK = G::distance(cavern[0], cavern[cavern.size() - 1]) / 8;
  double r = (baseK / RANGE) * abs(delta);
  vector<Point> result;
  result.push_back(from);
  if (L_out.empty()){
    result.push_back(I);
    return result;
  }
  // vector<Point> L_outPoint;
  // for (int i=0; i<L_out.size(); i++){
  //   L_outPoint.push_back(hole[L_out[i]]);
  // }
  // debugPath(L_outPoint, "blue");
  for (int i=0; i<5; i++) trace() << "OK";

  // if stay inside deep caverns
  int startShift = 0;
  Point SP_0 = hole[L_out[0]];
  Point SP_1 = hole[L_out[1]];
  Vector v_0(SP_0, SP_1);
  int sign_0 = G::isMonotoneLine(L_out[0], L_out[1], hole);
  trace() << "sign " << sign_0;
  trace() << G::isMonotoneLine(L_out[0], L_out[1], hole);
  trace() << G::isMonotoneLine(L_out[1], L_out[0], hole);
  Point SP_01 = SP_0 + v_0.rotate(M_PI / 2) * (baseK * sign_0 / v_0.length());
  Point SP_02 = SP_0 + v_0.rotate(M_PI / 2) * (baseK * sign_0 / v_0.length() / 2);
  // debugPoint(SP_0, "blue");
  // debugPoint(SP_1, "blue");
  debugPoint(SP_01, "green");
  debugPoint(SP_02, "green");
  LineSegment XY(SP_01, SP_02);
  if (G::is_intersect(XY, hole)){
    for (int ii=0; ii<5; ii++) trace() << "is_intersect";
    int i;
    for (i=1; i<L_out.size() - 1; i++){
      Point SP_i = hole[L_out[i]];
      Vector v_i1(SP_i, hole[L_out[i-1]]);
      int sign_i1 = G::isMonotoneLine(L_out[i], L_out[i-1], hole);
      trace() << "sign " << sign_i1;
      Point SP_i1 = SP_i + v_i1.rotate(M_PI / 2) * (baseK * sign_i1 / v_i1.length());
      Vector v_i2(SP_i, hole[L_out[i+1]]);
      int sign_i2 = G::isMonotoneLine(L_out[i], L_out[i+1], hole);
      trace() << "sign " << sign_i2;
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
      result.push_back(I);
      return G::flatten(result);
    }
  }

  trace() << "OK2";
  trace() << startShift;

  // debugPath(result, "black");

  int nHole = hole.size();
  vector<int> monoIndex;
  int side = 0;
  for (int i = startShift; i < L_out.size(); i++){
    trace() << "index " << i;
    for (int k : monoIndex){
      trace() << k;
    }
    if (monoIndex.size() < 2){
      monoIndex.push_back(L_out[i]);
      if (monoIndex.size() == 2) side = G::isMonotoneLine(monoIndex[0], monoIndex[1], hole);
    }
    else {
      int p_i1 = monoIndex[monoIndex.size()-1];
      int p_i2 = L_out[i];
      if ((G::isMonotoneLine(p_i1, p_i2, hole) == 0) || ((i == L_out.size()-1) && (monoIndex.size() >= 2))){
        if (i == L_out.size()-1) monoIndex.push_back(L_out[i]);
        vector<Point> monoLine;
        for (int j=0; j<monoIndex.size(); j++){
          monoLine.push_back(hole[monoIndex[j]]);
          if (side < 0) debugPoint(hole[monoIndex[j]], "blue");
          else debugPoint(hole[monoIndex[j]], "red");
        }
        double translateFactor = r;
        if (side < 0) translateFactor = r - baseK;
        auto translatedLine = G::translate(monoLine, translateFactor);
        for (auto p: translatedLine){
          result.push_back(p);
        }
        monoIndex.clear();
      }
      if (i < L_out.size()-1) monoIndex.push_back(L_out[i]);
    }
  }

  for (int i=0; i<hole.size(); i+=20) {
    debugPoint(hole[i], "gray");
  }
  debugPoint(hole[0], "gray");
  for (int i=0; i<5; i++) trace() << "OK3";

  if (!monoIndex.empty()){
    result.push_back(hole[monoIndex[0]]);
  }
  auto flattenResult = G::flatten(result);

  outCavernCache[make_tuple(cavernHash, from, to, delta)] = flattenResult;
  flattenResult.push_back(hole[L_out[L_out.size()-1]]);

  return flattenResult;
}


vector<Point> MlpRouting::findPathOutCavern(Point from, Point to, vector<Point> &hole,
                                vector<Point> &cavern, int delta) {
  // delta from -RANGE to RANGE

  tuple<Point, Point> cavernHash = G::hash(cavern);
  if (outCavernCache.find(make_tuple(cavernHash, from, to, delta)) != outCavernCache.end()) {
    return outCavernCache[make_tuple(cavernHash, from, to, delta)];
  }

  double baseK = G::distance(cavern[0], cavern[cavern.size() - 1]) / 8;
  vector<Point> interiorCavern = G::rollBallCavern(cavern, baseK);

//  debugPolygon(interiorCavern, "red");
  vector<Point> shortestPath = G::shortestPathOutOrOnPolygon(hole, from, to);

  // gate
  Point M = cavern[0], N = cavern[cavern.size() - 1];
  Point I;
  LineSegment MN(M, N);
  for (int i = 0; i < shortestPath.size() - 1; i++) {
    Point X = shortestPath[i], Y = shortestPath[i + 1];
    LineSegment XY(X, Y);
    if (G::doIntersect(XY, MN)) {
      I = G::intersection(XY, MN);
      break;
    }
  }

  if (I.isUnspecified()) return vector<Point>(); // not gonna happen
  vector<Point> result;
  result.push_back(from);

  Point T = I + Vector(M, N).rotate(M_PI / 2) * (baseK / Vector(M, N).length());
  if (!G::pointInOrOnPolygon(interiorCavern, from)) {
    double minDistance = INT_MAX;
    Point closest = G::closestPointOnPolygon(interiorCavern, from);
    vector<Point> shortestPathToClosest = G::shortestPathOutOrOnPolygon(hole, from, closest);
    for (int i = 0; i < shortestPathToClosest.size() - 1; i++) {
      result.push_back(shortestPathToClosest[i]);
    }

//    debugLine(from, closest, "black");
    from = closest;
  }

//  debugPath(result, "black");
//  log() << result.size();

  // from to T
  vector<Point> spInterior = G::shortestPathInOrOnPolygon(interiorCavern, from, T);
//  debugPath(spInterior, "blue");
  auto translatedRoad = G::translate(spInterior, (baseK / RANGE) * delta);
  for (auto p: translatedRoad) result.push_back(p);

  auto flattenResult = G::flatten(result);
//  debugPath(flattenResult, "brown");

  outCavernCache[make_tuple(cavernHash, from, to, delta)] = flattenResult;

  return flattenResult;
}

vector<Point> MlpRouting::findPathAroundHole(Point from, Point to, vector<Point> &hole, double ballRadius) {

  if (aroundHoleCache.find(make_tuple(from, to, ballRadius)) != aroundHoleCache.end()) {
    return aroundHoleCache[make_tuple(from, to, ballRadius)];
  }

  if (G::outOrOnPolygon(hole, LineSegment(from, to))) {
    vector<Point> result = {from, to};
    return result;
  }


  vector<Point> convexHull = G::convexHull(hole);
  vector<Point> balledConvexHull = G::rollBallPolygon(convexHull, ballRadius);
  vector<Point> res;
  auto closestFrom = from, closestTo = to;
  if (G::pointInOrOnPolygon(balledConvexHull, from)) {
    closestFrom = G::closestPointOnPolygon(balledConvexHull, from);
  }
  if (G::pointInOrOnPolygon(balledConvexHull, to)) {
    closestTo = G::closestPointOnPolygon(balledConvexHull, to);
  }

  if (closestFrom != from) {
    for (auto p: G::shortestPathOutOrOnPolygon(hole, from, closestFrom)) {
      res.push_back(p);
    }
  }

  for (auto p: G::shortestPathOutOrOnPolygon(balledConvexHull, closestFrom, closestTo)) {
    res.push_back(p);
  }

  if (closestTo != to) {
    for (auto p: G::shortestPathOutOrOnPolygon(hole, closestTo, to)) {
      res.push_back(p);
    }
  }

  auto flattenResult = G::flatten(res);

  aroundHoleCache[make_tuple(from, to, ballRadius)] = flattenResult;
  return flattenResult;
}
