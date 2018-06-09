#include "StableRouting.h"
#include <crng.h>
#include "assert.h"

Define_Module(StableRouting);

int StableRouting::cnt = 0;
int StableRouting::nextId;
map<CavernPairRadius, vector<Point> > StableRouting::outCavernCache;
map<tuple<Point, Point, double>, vector<Point> > StableRouting::aroundHoleCache;
map<tuple<Point, Point, double, double, double>, vector<Point> > StableRouting::findPathCache;

void StableRouting::startup(){
  nextId = 0; // static member
  setTimer(DISCOVER_HOLE_START, 1);
  secondBallRadius = par("secondBallRadius");
  receivedHole = false;
}

void StableRouting::timerFiredCallback(int index){
  switch(index){
    case DISCOVER_HOLE_START: {
//      debugCircle(selfLocation, RADIO_RANGE,"black");

      // check whether this node is in a hole
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
//        debugPoint(selfLocation, "red");
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


void StableRouting::processBufferedPacket(){
  while (!TXBuffer.empty()) {
    toMacLayer(TXBuffer.front(), BROADCAST_MAC_ADDRESS);
    TXBuffer.pop();
  }
}

void StableRouting::fromApplicationLayer(cPacket * pkt, const char *destination){


  StablePacket *dataPacket = new StablePacket("STABLE routing data packet", NETWORK_LAYER_PACKET);
  encapsulatePacket(dataPacket, pkt);
  dataPacket->setStablePacketKind(STABLE_DATA_PACKET);
  dataPacket->setSource(SELF_NETWORK_ADDRESS);
  dataPacket->setRoutingMode(GREEDY_ROUTING);
  dataPacket->setDestination(destination);
  dataPacket->setTTL(1000);


  if (string(destination).compare(BROADCAST_NETWORK_ADDRESS)==0) {
    toMacLayer(dataPacket, BROADCAST_MAC_ADDRESS);
    return;
  }


  dataPacket->setDestLocation(GlobalLocationService::getLocation(atoi(destination)));
  dataPacket->setPacketId(nextId++);

//  return;

//  debugPoint(selfLocation, "red");
//  debugPoint(dataPacket->getDestLocation(), "green");
  processDataPacket(dataPacket);

}



void StableRouting::fromMacLayer(cPacket * pkt, int macAddress, double rssi, double lqi){
  DiscoverHolePacket *discoverHolePacket = dynamic_cast <DiscoverHolePacket*>(pkt);
  if (discoverHolePacket) {
    processDiscoverHolePacket(discoverHolePacket);
    return;
  }


  StablePacket *netPacket = dynamic_cast <StablePacket*>(pkt);
  if (!netPacket)
    return;

  switch (netPacket->getStablePacketKind()) {
    case STABLE_DATA_PACKET:
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

        StablePacket *pkt = netPacket->dup();
        processDataPacket(pkt);
        break;
      }

    default: return;
  }
}

void StableRouting::finishSpecific() {
  trace() << "WSN_EVENT FINAL" << " id:" << self << " x:" << selfLocation.x() << " y:" << selfLocation.y() << " deathTime:-1";
//  log() << "done man ";
}

void StableRouting::processHole(DiscoverHolePacket* pkt) {

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

//  debugPolygon(convexHull, "green");
//  debugPolygon(points, "blue");
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

//  if (self == 800) { debugPolygon(hole, "green");
//  }
//  if (self == 680) {
//    Point from = GlobalLocationService::getLocation(680);
//    Point to = GlobalLocationService::getLocation(923);
//    debugPolygon(convexHull, "purple");
//    auto balledPolygon = G::rollBallPolygon(convexHull, 70);
//    debugPolygon(balledPolygon, "blue");
//    debugPath(findPathAroundHole(from, to, hole, 70), "black");
//
//  }

}


void StableRouting::processDiscoverHolePacket(DiscoverHolePacket* pkt){
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
//          propagateHole(pkt);
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


void StableRouting::propagateHole(DiscoverHolePacket *pkt) {
//  debugPoint(selfLocation, "red");
  DiscoverHolePacket *dup = pkt->dup();
  dup->setDestination(BROADCAST_NETWORK_ADDRESS);
  toMacLayer(dup, BROADCAST_MAC_ADDRESS);
}

void StableRouting::processDataPacket(StablePacket* pkt){
//  debugPoint(selfLocation, "green");
  Point destLocation = pkt->getDestLocation();
  if (pkt->getNextStoppingPlace().isUnspecified()) {
    if (receivedHole) {
      // first time in
      vector<Point> path;
      double outCavernRadius, aroundHoleRadius, inCavernRadius;
      tie(path, outCavernRadius, aroundHoleRadius, inCavernRadius) = findPath(selfLocation,
        destLocation, hole, caverns);
      findPathCache[make_tuple(selfLocation, destLocation, outCavernRadius, aroundHoleRadius, inCavernRadius)] = path;
//      debugPath(path, "black");
//      for (auto p: path) {
////        debugPoint(p, "black");
//      }

//      for (auto p: path) debugPoint(p, "green");
      pkt->setOutCavernRadius(outCavernRadius);
      pkt->setAroundHoleRadius(aroundHoleRadius);
      pkt->setInCavernRadius(inCavernRadius);
      pkt->setStartStableLocation(selfLocation);
      pkt->setNextStoppingPlace(path[1]);
      pkt->setRoutingMode(GREEDY_ROUTING);

      processDataPacket(pkt);
    } else {
      // still outside
      if (pkt->getRoutingMode() == GREEDY_ROUTING) {
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
          debugLine(selfLocation, GlobalLocationService::getLocation(nextHop), "red");
          toMacLayer(pkt, nextHop);
        } else {

          pkt->setRoutingMode(ROLLINGBALL_ROUTING);
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
        if (G::distance(selfLocation, destLocation) < G::distance(stuckLocation, destLocation)) {
          pkt->setRoutingMode(GREEDY_ROUTING);
          processDataPacket(pkt);
        } else {
          int nextHop = -1;
          Point nextCenter;
          Point ballCenter = pkt->getBallCenter();
          nextHop = G::findNextHopRollingBall(selfLocation, ballCenter, RADIO_RANGE / 2, neighborTable, nextCenter);
          if (nextHop != -1) {
            debugLine(selfLocation, GlobalLocationService::getLocation(nextHop), "red");
            toMacLayer(pkt, nextHop);
          }
        }
      }
    }
  } else {
    // heading to the next stopping place
    Point nextStoppingPlace = pkt->getNextStoppingPlace();
    if (reached(nextStoppingPlace)) {
//      debugPoint(selfLocation, "green");
      Point startStableLocation = pkt->getStartStableLocation();
      double outCavernRadius = pkt->getOutCavernRadius();
      double aroundHoleRadius = pkt->getAroundHoleRadius();
      double inCavernRadius = pkt->getInCavernRadius();

      if (findPathCache.find(make_tuple(startStableLocation, destLocation, outCavernRadius, aroundHoleRadius, inCavernRadius))
        != findPathCache.end()) {
        vector<Point> path = findPathCache[make_tuple(startStableLocation, destLocation, outCavernRadius, aroundHoleRadius, inCavernRadius)];
        int j = -1;
        for (int i = 0; i < path.size() - 1; i++) {
          if (path[i] == nextStoppingPlace) {
            j = i;
            break;
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
      if (pkt->getRoutingMode() == GREEDY_ROUTING) {
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
          debugLine(selfLocation, GlobalLocationService::getLocation(nextHop), "red");
          toMacLayer(pkt, nextHop);
        } else {
          pkt->setRoutingMode(ROLLINGBALL_ROUTING);
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
        if (G::distance(selfLocation, nextStoppingPlace) < G::distance(stuckLocation, nextStoppingPlace)) {
          pkt->setRoutingMode(GREEDY_ROUTING);
          processDataPacket(pkt);
        } else {
          int nextHop = -1;
          Point nextCenter;
          Point ballCenter = pkt->getBallCenter();
          nextHop = G::findNextHopRollingBall(selfLocation, ballCenter, RADIO_RANGE / 2, neighborTable, nextCenter);
          if (nextHop != -1) {
            debugLine(selfLocation, GlobalLocationService::getLocation(nextHop), "red");
            toMacLayer(pkt, nextHop);
          }
        }
      }
    }
  }
}


Point StableRouting::getNeighborLocation(int id) {
  for (auto &n: neighborTable) {
    if (n.id == id) {
      return n.location;
    }
  }

  return Point(); // default
}
// will handle interaction between the application layer and the GPRS module in order to pass parameters such as
// the node's position
void StableRouting::handleNetworkControlCommand(cMessage *msg) {

}



vector<Point> StableRouting::findPathOutCavern(Point from, Point to, vector<Point> &hole,
                                vector<Point> &cavern, double ballRadius) {

  tuple<Point, Point> cavernHash = G::hash(cavern);
  if (outCavernCache.find(make_tuple(cavernHash, from, to, ballRadius)) != outCavernCache.end()) {
    return outCavernCache[make_tuple(cavernHash, from, to, ballRadius)];
  }

  vector<Point> interiorCavern = G::rollBallCavern(cavern, ballRadius);
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

  Point T = I + Vector(M, N).rotate(M_PI / 2) * (ballRadius / Vector(M, N).length());
  if (!G::pointInOrOnPolygon(interiorCavern, from)) {
    double minDistance = INT_MAX;
    Point closest = G::closestPointOnPolygon(interiorCavern, from);
    vector<Point> shortestPathToClosest = G::shortestPathOutOrOnPolygon(hole, from, closest);
    for (int i = 0; i < shortestPathToClosest.size() - 1; i++) {
      result.push_back(shortestPathToClosest[i]);
    }

    from = closest;
  }

  vector<Point> spInterior = G::shortestPathInOrOnPolygon(interiorCavern, from, T);
  for (auto p: spInterior) result.push_back(p);

  auto flattenResult = G::flatten(result);
  outCavernCache[make_tuple(cavernHash, from, to, ballRadius)] = flattenResult;

  return flattenResult;
}

vector<Point> StableRouting::findPathAroundHole(Point from, Point to, vector<Point> &hole, double ballRadius) {
  if (aroundHoleCache.find(make_tuple(from, to, ballRadius)) != aroundHoleCache.end()) {
    return aroundHoleCache[make_tuple(from, to, ballRadius)];
  }

  if (G::outOrOnPolygon(hole, LineSegment(from, to))) {
    vector<Point> result = {from, to};
    aroundHoleCache[make_tuple(from, to, ballRadius)] = result;
    return result;
  }


  vector<Point> convexHull = G::convexHull(hole);
//  debugPolygon(convexHull, "purple");
//  debugPolygon(hole, "red");
  // assume none of from and to is inside the convex hull
  vector<Point> balledConvexHull = G::rollBallPolygon(convexHull, ballRadius);
//  debugPolygon(balledConvexHull, "gray");
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

tuple<vector<Point>, double, double, double> StableRouting::findPath(Point from, Point to, vector<Point> &hole, vector<vector<Point>> &caverns) {
  vector<Point> result;
  double inCavernRadius = -1;
  double outCavernRadius = -1;
  double aroundHoleRadius = -1;

  // strange case
  if (hole.empty()) {
    vector<Point> path = {from, to};
    return make_tuple(path, outCavernRadius, aroundHoleRadius, inCavernRadius);
  }

  // go straight
  if (G::outOrOnPolygon(hole, LineSegment(from, to))) {
    vector<Point> path = {from, to};
    return make_tuple(path, outCavernRadius, aroundHoleRadius, inCavernRadius);
  }

  // in the same cavern
  for (auto cavern: caverns) {
    if (G::pointInOrOnPolygon(cavern, from) && G::pointInOrOnPolygon(cavern, to)) {
      auto result = G::shortestPathInOrOnPolygon(cavern, from, to);
      return {result, outCavernRadius, aroundHoleRadius, inCavernRadius};
    }
  }

  vector<Point> outCavern;
  vector<Point> inCavern;
  Point newFrom = from, newTo = to;

  int pathNum = (rand() % NUM_PATH + 1);

  for (auto cavern: caverns) {
    if (G::pointInOrOnPolygon(cavern, from)) {
      double maxRadius = G::distance(cavern[0], cavern[cavern.size() - 1]) / 4;
      outCavernRadius = pathNum * (maxRadius / NUM_PATH);
      outCavern = findPathOutCavern(from, to, hole, cavern, outCavernRadius);
//      debugPolygon(G::rollBallCavern(cavern, outCavernRadius), "green");
      newFrom = outCavern[outCavern.size() - 1];
      break;
    }
  }

  for (auto cavern: caverns) {
    if (G::pointInOrOnPolygon(cavern, to)) {
      double maxRadius = G::distance(cavern[0], cavern[cavern.size() - 1]) / 4;
      inCavernRadius = pathNum * (maxRadius / NUM_PATH);
      inCavern = findPathOutCavern(to, from, hole, cavern, inCavernRadius);
//      debugPolygon(G::rollBallCavern(cavern, inCavernRadius), "green");
      reverse(inCavern.begin(), inCavern.end());
      newTo = inCavern[0];
      break;
    }
  }

  from = newFrom;
  to = newTo;


  double maxRadius = 10 * log2(holeDiameter);
  aroundHoleRadius = pathNum * (maxRadius / NUM_PATH);
  vector<Point> middlePath = findPathAroundHole(newFrom, newTo, hole, aroundHoleRadius);
//  debugPolygon(G::rollBallPolygon(G::convexHull(hole), aroundHoleRadius), "purple");


  result.insert(result.end(), outCavern.begin(), outCavern.end());
  result.insert(result.end(), middlePath.begin(), middlePath.end());
  result.insert(result.end(), inCavern.begin(), inCavern.end());

  auto flattenResult = G::flatten(result);

  return {flattenResult, outCavernRadius, aroundHoleRadius, inCavernRadius};
}
