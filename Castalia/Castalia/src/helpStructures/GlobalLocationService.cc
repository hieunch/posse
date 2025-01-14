
#include "GlobalLocationService.h"

cModule *GlobalLocationService::networkModule;
bool GlobalLocationService::initialized = false;
int GlobalLocationService::numNodes = false;
double GlobalLocationService::cellWidth = 1;
double GlobalLocationService::cellHeight = 1;
std::vector<int> GlobalLocationService::numReceiveds;
std::vector<double> GlobalLocationService::sumRatios;
int GlobalLocationService::numEnds;
std::vector<std::map<int, int>> GlobalLocationService::colors;
std::vector<Point> GlobalLocationService::locations;
std::vector<std::vector<NeighborRecord>> GlobalLocationService::neighborTables;
map<tuple<int, int>, int> GlobalLocationService::spCache;
const double range = 40;

void GlobalLocationService::initialize(cModule *module) {
  if (!initialized) {
    networkModule = module;
    initialized = true;

    numNodes = (int) module->par("numNodes").longValue();
    locations.assign(numNodes, Point());
    neighborTables.assign(numNodes, std::vector<NeighborRecord>());
    numReceiveds.assign(numNodes, 0);
    sumRatios.assign(numNodes, 0);
    numEnds = 0;
    colors.assign(numNodes, std::map<int, int>());
    for (int i = 0; i < numNodes; i++) {
      double xCoor = networkModule->getSubmodule("node", i)->par("xCoor").doubleValue();
      double yCoor = networkModule->getSubmodule("node", i)->par("yCoor").doubleValue();
      locations[i] = Point(xCoor, yCoor);
    }
    cellWidth = module->par("cellWidth").doubleValue();
    cellHeight = module->par("cellHeight").doubleValue();

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

void GlobalLocationService::removeNode(int nodeId) {
  for (auto record : neighborTables[nodeId]) {
    int neighborId = record.id;
    for(vector<NeighborRecord>::iterator iter = neighborTables[neighborId].begin(); iter != neighborTables[neighborId].end(); ++iter) {
      if((*iter).id == nodeId) {
        neighborTables[neighborId].erase(iter);
        break;
      }
    }
  }
  neighborTables[nodeId].clear();
}

std::vector<NeighborRecord> GlobalLocationService::getNeighborTable(int id) {
  return neighborTables[id];
}

Point GlobalLocationService::getLocation(int id) {
  if (id == -1) return Point();
  return locations[id];
}

int GlobalLocationService::getId(Point p) {
  for (int v = 0; v < locations.size(); v++) {
    if (locations[v] == p) return v;
  }
  return -1;
}

int GlobalLocationService::getNumReceived(int id) {
  if (id == -1) return 0;
  return numReceiveds[id];
}

void GlobalLocationService::increaseNumReceived(int id){
  numReceiveds[id]++;
}

void GlobalLocationService::decreaseNumReceived(int id){
  numReceiveds[id]--;
}

int GlobalLocationService::getTotalNumReceived() {
  int total = 0;
  for (int num : numReceiveds) total += num;
  return total;
}

double GlobalLocationService::getSumRatio(int id) {
  if (id == -1) return 0;
  return sumRatios[id];
}

void GlobalLocationService::increaseSumRatio(int id, double ratio){
  sumRatios[id] += ratio;
  numEnds++;
}

double GlobalLocationService::getTotalSumRatio() {
  double total = 0;
  for (double ratio : sumRatios) total += ratio;
  return total;
}

double GlobalLocationService::getStretch() {
  double totalSumRatio = getTotalSumRatio();
  return totalSumRatio/numEnds;
}

double GlobalLocationService::getBalancingIndex() {
  int total2 = 0;
  for (int num : numReceiveds) total2 += num*num;
  int total = getTotalNumReceived();
  return 1.*total*total/numNodes/total2;
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


  spCache[make_tuple(source, destination)] = d[destination];


  return d[destination];
}

bool GlobalLocationService::isInSameCell(Point A, Point B) {
  // return ((int(A.x_/cellWidth) == int(B.x_/cellWidth)) &&
  //         (int(A.y_/cellHeight) == int(B.y_/cellHeight)));
  return G::distance(A, B) < 40;
}

vector<Point> GlobalLocationService::getCell(Point p) {
  vector<Point> cell;
  double x1 = floor(p.x_/cellWidth) * cellWidth;
  double y1 = floor(p.y_/cellHeight) * cellHeight;
  double x2 = x1 + cellWidth;
  double y2 = y1 + cellHeight;
  Point A(x1, y1); cell.push_back(A);
  Point B(x2, y1); cell.push_back(B);
  Point C(x2, y2); cell.push_back(C);
  Point D(x1, y2); cell.push_back(D);
  return cell;
}

int GlobalLocationService::getColor(int current, int destination) {
  if (colors[current].find(destination) == colors[current].end()) {
    colors[current][destination] = 0;
  }
  return colors[current][destination];
}

void GlobalLocationService::increaseColor(int current, int destination) {
  if (colors[current].find(destination) != colors[current].end()) {
    colors[current][destination]++;
  }
}

int GlobalLocationService::getColor(int node) {
  int maxColor = 0;
  std::map<int, int>::iterator it;
  for(it = colors[node].begin(); it != colors[node].end(); it++) {
    int color = it->second;
    if (color > maxColor) maxColor = color;
  }
  return maxColor;
}