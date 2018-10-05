#include "MlpRouting.h"
#include <crng.h>
#include "assert.h"

Define_Module(MlpRouting);

int MlpRouting::cnt = 0;
int MlpRouting::nextId;

int getRandomNumber(int from, int to) {
    srand(time(0));
    int result = from + rand() % (to - from + 1);
    return result;
}

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

  if (distanceToHole < 200 * log2(holeDiameter) || G::pointInOrOnPolygon(convexHull, selfLocation)) {
    propagateHole(pkt);
  }

  if (self == 787) {
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
  Point destLocation = pkt->getDestLocation();
  if (receivedHole) {
    auto path = findPath(selfLocation, destLocation, hole, caverns);
    debugPath(path, "green");
  }
}


// will handle interaction between the application layer and the GPRS module in order to pass parameters such as
// the node's position
void MlpRouting::handleNetworkControlCommand(cMessage *msg) {

}

vector<Point> MlpRouting::findPath(Point from, Point to, vector<Point> &hole, vector<vector<Point>> &caverns) {
  vector<Point> result;
  double inCavernRadius = -1;
  double outCavernRadius = -1;
  double aroundHoleRadius = -1;

  // strange case
  if (hole.empty()) {
    return {from, to};
  }

  // go straight
  if (G::outOrOnPolygon(hole, LineSegment(from, to))) {
    return {from, to};
  }

  // in the same cavern
  for (auto cavern: caverns) {
    if (G::pointInOrOnPolygon(cavern, from) && G::pointInOrOnPolygon(cavern, to)) {
      return G::shortestPathInOrOnPolygon(cavern, from, to);
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

  vector<Point> middlePath = findPathAroundHole(newFrom, newTo, hole, aroundHoleRadius);

  result.insert(result.end(), outCavern.begin(), outCavern.end());
  result.insert(result.end(), middlePath.begin(), middlePath.end());
  result.insert(result.end(), inCavern.begin(), inCavern.end());

  return G::flatten(result);
}


vector<Point> MlpRouting::findPathOutCavern(Point from, Point to, vector<Point> &hole,
                                vector<Point> &cavern, int delta) {
  // delta from -RANGE to RANGE
  double baseK = G::distance(cavern[0], cavern[cavern.size() - 1]) / 8;
  vector<Point> interiorCavern = G::rollBallCavern(cavern, baseK);

  debugPolygon(interiorCavern, "red");
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

  Point T = I + Vector(M, N).rotate(M_PI / 2) * (baseK / Vector(M, N).length());
  if (!G::pointInOrOnPolygon(interiorCavern, from)) {
    double minDistance = INT_MAX;
    Point closest = G::closestPointOnPolygon(interiorCavern, from);
    vector<Point> shortestPathToClosest = G::shortestPathOutOrOnPolygon(hole, from, closest);
    for (int i = 0; i < shortestPathToClosest.size() - 1; i++) {
      result.push_back(shortestPathToClosest[i]);
    }

    from = closest;
  }

  // from to T
  vector<Point> spInterior = G::shortestPathInOrOnPolygon(interiorCavern, from, T);
  debugPath(spInterior, "blue");
  auto translatedRoad = G::translate(spInterior, (baseK / RANGE) * delta);
  for (auto p: translatedRoad) result.push_back(p);

  auto flattenResult = G::flatten(result);

  return flattenResult;
}

vector<Point> MlpRouting::findPathAroundHole(Point from, Point to, vector<Point> &hole, double ballRadius) {

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

  return flattenResult;
}
