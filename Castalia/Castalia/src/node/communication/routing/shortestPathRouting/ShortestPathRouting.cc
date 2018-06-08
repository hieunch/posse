#include "ShortestPathRouting.h"
#include <crng.h>
#include "assert.h"

Define_Module(ShortestPathRouting);

int ShortestPathRouting::cnt = 0;
int ShortestPathRouting::nextId;
map<tuple<Point, Point>, vector<Point> > ShortestPathRouting::shortestPathCache;

void ShortestPathRouting::startup(){
  nextId = 0; // static member
  setTimer(DISCOVER_HOLE_START, 1);
  secondBallRadius = par("secondBallRadius");
  receivedHole = false;
}

void ShortestPathRouting::timerFiredCallback(int index){
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
        Point nextCenter;
        int nextHop = G::findNextHopRollingBall(selfLocation, chosenCenter, RADIO_RANGE / 2, neighborTable, nextCenter);
        if (nextHop != -1) {
          Point nextHopLocation = GlobalLocationService::getLocation(nextHop);
          ShortestPathDiscoverHolePacket *discoverHolePacket = new ShortestPathDiscoverHolePacket("Discover hole packet", NETWORK_LAYER_PACKET);
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


void ShortestPathRouting::processBufferedPacket(){
  while (!TXBuffer.empty()) {
    toMacLayer(TXBuffer.front(), BROADCAST_MAC_ADDRESS);
    TXBuffer.pop();
  }
}

void ShortestPathRouting::fromApplicationLayer(cPacket * pkt, const char *destination){


  ShortestPathRoutingPacket *dataPacket = new ShortestPathRoutingPacket("STABLE routing data packet", NETWORK_LAYER_PACKET);
  encapsulatePacket(dataPacket, pkt);
  dataPacket->setShortestPathPacketKind(SHORTEST_PATH_DATA_PACKET);
  dataPacket->setSource(SELF_NETWORK_ADDRESS);
  dataPacket->setRoutingMode(GREEDY_ROUTING);
  dataPacket->setDestination(destination);


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



void ShortestPathRouting::fromMacLayer(cPacket * pkt, int macAddress, double rssi, double lqi){
  ShortestPathDiscoverHolePacket *discoverHolePacket = dynamic_cast <ShortestPathDiscoverHolePacket*>(pkt);
  if (discoverHolePacket) {
    processDiscoverHolePacket(discoverHolePacket);
    return;
  }


  ShortestPathRoutingPacket *netPacket = dynamic_cast <ShortestPathRoutingPacket*>(pkt);
  if (!netPacket)
    return;

  switch (netPacket->getShortestPathPacketKind()) {
    case SHORTEST_PATH_DATA_PACKET:
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

        ShortestPathRoutingPacket *pkt = netPacket->dup();
        processDataPacket(pkt);
        break;
      }

    default: return;
  }
}

void ShortestPathRouting::finishSpecific() {
  trace() << "WSN_EVENT FINAL" << " id:" << self << " x:" << selfLocation.x() << " y:" << selfLocation.y() << " deathTime:-1";
}

void ShortestPathRouting::processHole(ShortestPathDiscoverHolePacket* pkt) {

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

}


void ShortestPathRouting::processDiscoverHolePacket(ShortestPathDiscoverHolePacket* pkt){
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
    ShortestPathDiscoverHolePacket *newPkt = pkt->dup();
    newPkt->setPreviousId(self);
    newPkt->setBallCenter(nextCenter);
    newPkt->setPath(Util::intVectorToString(path).c_str());
    toMacLayer(newPkt, nextHop);
  }

}


void ShortestPathRouting::propagateHole(ShortestPathDiscoverHolePacket *pkt) {
  ShortestPathDiscoverHolePacket *dup = pkt->dup();
  dup->setDestination(BROADCAST_NETWORK_ADDRESS);
  toMacLayer(dup, BROADCAST_MAC_ADDRESS);
}

void ShortestPathRouting::processDataPacket(ShortestPathRoutingPacket* pkt){
//  debugPoint(selfLocation, "green");
  Point destLocation = pkt->getDestLocation();
  if (pkt->getNextStoppingPlace().isUnspecified()) {
    if (receivedHole) {
      // first time in
      vector<Point> path;
      path = findPath(selfLocation, destLocation, hole, caverns);
//      debugPath(path, "black");

      pkt->setStartShortestPathLocation(selfLocation);
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
//          debugLine(selfLocation, GlobalLocationService::getLocation(nextHop), "blue");
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
//            debugLine(selfLocation, GlobalLocationService::getLocation(nextHop), "blue");
            toMacLayer(pkt, nextHop);
          }
        }
      }
    }
  } else {
    // heading to the next stopping place
    Point nextStoppingPlace = pkt->getNextStoppingPlace();
    if (reached(nextStoppingPlace)) {
      Point startShortestPathLocation = pkt->getStartShortestPathLocation();

      vector<Point> path = findPath(startShortestPathLocation, destLocation, hole, caverns);
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
//          debugLine(selfLocation, GlobalLocationService::getLocation(nextHop), "blue");
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
//            debugLine(selfLocation, GlobalLocationService::getLocation(nextHop), "blue");
            toMacLayer(pkt, nextHop);
          }
        }
      }
    }
  }
}


Point ShortestPathRouting::getNeighborLocation(int id) {
  for (auto &n: neighborTable) {
    if (n.id == id) {
      return n.location;
    }
  }

  return Point(); // default
}
// will handle interaction between the application layer and the GPRS module in order to pass parameters such as
// the node's position
void ShortestPathRouting::handleNetworkControlCommand(cMessage *msg) {

}


vector<Point> ShortestPathRouting::findPath(Point from, Point to, vector<Point> &hole, vector<vector<Point>> &caverns) {
  if (shortestPathCache.find({from, to}) != shortestPathCache.end()) {
    return shortestPathCache[{from, to}];
  }
  vector<Point> result = G::shortestPathOutOrOnPolygon(hole, from, to);
  shortestPathCache[{from, to}] = result;

  return result;
}
