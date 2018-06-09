
#include "GlobalLocationService.h"

cModule *GlobalLocationService::networkModule;
bool GlobalLocationService::initialized = false;
std::vector<Point> GlobalLocationService::locations;
std::vector<std::vector<NeighborRecord>> GlobalLocationService::neighborTables;
map<tuple<int, int>, int> GlobalLocationService::spCache;
const double range = 40;

void GlobalLocationService::initialize(cModule *module) {
  if (!initialized) {
    networkModule = module;
    initialized = true;

    int numNodes = (int) module->par("numNodes").longValue();
    locations.assign(numNodes, Point());
    neighborTables.assign(numNodes, std::vector<NeighborRecord>());
    for (int i = 0; i < numNodes; i++) {
      double xCoor = networkModule->getSubmodule("node", i)->par("xCoor").doubleValue();
      double yCoor = networkModule->getSubmodule("node", i)->par("yCoor").doubleValue();
      locations[i] = Point(xCoor, yCoor);
    }

    for (int i = 0; i < numNodes; i++) {
      for (int j = i + 1; j < numNodes; j++) {
        if (G::distance(locations[i], locations[j]) < range) {
          neighborTables[i].push_back(NeighborRecord(j, locations[j]));
          neighborTables[j].push_back(NeighborRecord(i, locations[i]));
        }
      }
    }
  }
}
std::vector<NeighborRecord> GlobalLocationService::getNeighborTable(int id) {
  return neighborTables[id];
}

Point GlobalLocationService::getLocation(int id) {
  return locations[id];
}
int GlobalLocationService::numHopShortestPath(int source, int destination) {
  if (spCache.find(make_tuple(source, destination)) != spCache.end()) {
    return spCache[make_tuple(source, destination)];
  }

  vector<bool> marked;
  marked.assign(locations.size(), false);
  vector<int> d;
  d.assign(locations.size(), 0);

  marked[source] = true;
  queue<int> q;
  q.push(source);
  while (!q.empty()) {
    int u = q.front(); q.pop();
    if (u == destination) break;
    for (int v = 0; v < locations.size(); v++) if (!marked[v]) {
      if (G::distance(locations[u], locations[v]) < range) {
        marked[v] = true;
        d[v] = d[u] + 1;
        q.push(v);
      }
    }
  }


  spCache[{source, destination}] = d[destination];


  return d[destination];
}
