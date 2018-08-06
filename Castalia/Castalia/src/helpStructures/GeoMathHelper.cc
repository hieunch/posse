#include "GeoMathHelper.h"
#include "DebugInfoWriter.h"

std::ostream & G::log() {
  return trace() << " WSN_LOG ";
}

void G::debugLine(double x1, double y1, double x2, double y2, string color) {
  trace() << "WSN_EVENT DRAW LINE " << x1 << " " << y1 << " " << x2 << " " << y2 << " " << color;
}
void G::debugLine(Point p1, Point p2, string color) {
  debugLine(p1.x(), p1.y(), p2.x(), p2.y(), color);
}
void G::debugCircle(double centerX, double centerY, double radius, string color) {
  trace() << "WSN_EVENT DRAW CIRCLE " << centerX << " " << centerY << " " << radius << " " << color;
}
void G::debugCircle(Point center, double radius, string color) {
  debugCircle(center.x(), center.y(), radius, color);
}
void G::debugPoint(double x1, double x2, string color) {
  trace() << "WSN_EVENT DRAW POINT " << x1 << " " << x2 << " " << color;
}
void G::debugPoint(Point p, string color) {
  debugPoint(p.x(), p.y(), color);
}
void G::debugArc(Point from, Point to, double radius, string color) {
  trace() << "WSN_EVENT DRAW ARC " << from.x() << " " << from.y() << " " << to.x() << " "
    << to.y() << " " << radius << " " << color;
}

std::ostream & G::trace() {
  return (ostream &) DebugInfoWriter::getStream() << endl << " ";
}
void G::debugPolygon(vector<Point> ps, string color) {
  for (int i = 0; i < ps.size(); i++) {
    Point from = ps[i];
    Point to = ps[(i + 1) % ps.size()];
    debugLine(from, to, color);
  }
}

void G::debugPath(vector<Point> ps, string color) {
  for (int i = 0; i < ps.size() - 1; i++) {
    Point from = ps[i];
    Point to = ps[(i + 1) % ps.size()];
    debugLine(from, to, color);
  }
}

std::ostream &operator<<(std::ostream &os, const Point &m) {
    return os << "Point(" << m.x() << ", " << m.y() << ")";
}

std::ostream &operator<<(std::ostream &os, const Vector &v) {
    return os << "Vector(" << v.x << ", " << v.y << ")";
}

Line G::line(Point p1, Point p2) {
    Line re;
    line(p1.x_, p1.y_, p2.x_, p2.y_, re.a_, re.b_, re.c_);
    return re;
}

void G::line(double x1, double y1, double x2, double y2, double &a, double &b,
        double &c) {
    a = y1 - y2;
    b = x2 - x1;
    c = -y1 * x2 + y2 * x1;
}

double G::distance(Point p1, Point p2) {
    return distance(p1.x_, p1.y_, p2.x_, p2.y_);
}
double G::distance(double x1, double y1, double x2, double y2) {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

bool G::is_intersect(Point* p1, Point* p2, Point* p3, Point* p4) {
  Line l1 = line(p1, p2);
  Line l2 = line(p3, p4);
  Point in;
  return (intersection(l1, l2, in)
          && ((in.x_ - p1->x_) * (in.x_ - p2->x_) <= 0)
          && ((in.y_ - p1->y_) * (in.y_ - p2->y_) <= 0)
          && ((in.x_ - p3->x_) * (in.x_ - p4->x_) <= 0)
          && ((in.y_ - p3->y_) * (in.y_ - p4->y_) <= 0));
}
bool G::intersection(Point* p1, Point* p2, Point* p3, Point* p4, Point& p) {
    Line l1 = line(p1, p2);
    Line l2 = line(p3, p4);
    return (intersection(l1, l2, p) && ((p.x_ - p1->x_) * (p.x_ - p2->x_) <= 0)
            && ((p.y_ - p1->y_) * (p.y_ - p2->y_) <= 0)
            && ((p.x_ - p3->x_) * (p.x_ - p4->x_) <= 0)
            && ((p.y_ - p3->y_) * (p.y_ - p4->y_) <= 0));
}

bool G::intersection(Line l1, Line l2, Point& p) {
  return intersection(l1.a_, l1.b_, l1.c_, l2.a_, l2.b_, l2.c_, p.x_, p.y_);
}


bool G::intersection(double a1, double b1, double c1, double a2, double b2,
        double c2, double &x, double &y) {
    if (a1 == 0 && b1 == 0)
        return false;
    if (a2 == 0 && b2 == 0)
        return false;

    if (a1 == 0 && b2 == 0) {
        x = -c2 / a2;
        y = -c1 / b1;
    } else if (a2 == 0 && b1 == 0) {
        x = -c1 / a1;
        y = -c2 / b2;
    } else if (a1 * b2 != a2 * b1) {
        x = (b1 * c2 - b2 * c1) / (a1 * b2 - a2 * b1);
        y = (c1 * a2 - c2 * a1) / (a1 * b2 - a2 * b1);
    } else
        return a1 * c2 == a2 * c1;

    return true;
}

// find center of the circle with radius on which two points are located
void G::centers(Point p1, Point p2, double r, Point &center1, Point &center2) {
  double x1 = p1.x(), y1 = p1.y();
  double x2 = p2.x(), y2 = p2.y();
  double x3 = (x1 + x2) / 2;
  double y3 = (y1 + y2) / 2;
  double q = distance(p1, p2);

  center1 = Point(
    x3 + sqrt(r * r - (q / 2) * (q / 2)) * (y1 - y2) / q,
    y3 + sqrt(r * r - (q / 2) * (q / 2)) * (x2 - x1) / q
  );

  center2 = Point(
    x3 - sqrt(r * r - (q / 2) * (q / 2)) * (y1 - y2) / q,
    y3 - sqrt(r * r - (q / 2) * (q / 2)) * (x2 - x1) / q
  );
}

LineSegment G::translate(LineSegment ls, Vector v) {
  return LineSegment(ls.u + v, ls.v + v);
}

vector<Point> G::centers(Point p1, Point p2, double radius) {
  Point center1, center2;
  centers(p1, p2, radius, center1, center2);

  return {center1, center2};
}

Point Point::operator+(Vector v) {
  return Point(x() + v.x, y() + v.y);
}

double G::dot(Vector u, Vector v) {
  return u.x * v.x + u.y * v.y;
}

double G::intersection(LineSegment ls, Ray r) {
  Vector v1 = Vector(ls.u, r.p);
  Vector v2 = Vector(ls.u, ls.v);
  Vector v3 = Vector(-r.v.y, r.v.x);


  double dt = dot(v2, v3);
  if (abs(dt) < 0.0000001) {
    return -1;
  }
  double t1 = v2.crossProduct(v1) / dt;
  double t2 = dot(v1, v3) / dt;

  if (t1 >= 0.0 && (t2 >= 0.0 && t2 <= 1.0))
    return t1;
  return -1;
}
bool G::pointInPolygon(vector<Point> vs, Point point) {
  for (int i = 0; i < vs.size(); i++) {
    Point a = vs[i];
    Point b = vs[(i + 1) % vs.size()];
    if (onSegment(LineSegment(a, b), point)) {
      return true;
    }
  }

  double x = point.x(), y = point.y();
  bool inside = false;

  for (int i = 0, j = vs.size() - 1; i < vs.size(); j = i++) {
    double xi = vs[i].x(), yi = vs[i].y();
    double xj = vs[j].x(), yj = vs[j].y();

    double intersect = ((yi > y) != (yj > y))
                && (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
    if (intersect) inside = !inside;
  }

  return inside;
}
bool G::onSegment(LineSegment ls, Point p) {
  Vector uv(ls.u, ls.v);
  Vector up(ls.u, p);
  Vector pv(p, ls.v);

  return abs(uv.length() - up.length() - pv.length()) < EPSILON;
}
int G::orientation(Point p, Point q, Point r) {
  // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
  // for details of below formula.
  int val = (q.y() - p.y()) * (r.x() - q.x()) -
            (q.x() - p.x()) * (r.y() - q.y());

  if (val == 0) return 0;  // colinear

  return (val > 0) ? 1 : 2; // clock or counterclock wise
}
bool G::doIntersect(LineSegment ls1, LineSegment ls2) {
  Point p1 = ls1.u, q1 = ls1.v;
  Point p2 = ls2.u, q2 = ls2.v;

  // Find the four orientations needed for general and
  // special cases
  int o1 = orientation(p1, q1, p2);
  int o2 = orientation(p1, q1, q2);
  int o3 = orientation(p2, q2, p1);
  int o4 = orientation(p2, q2, q1);

  // General case
  if (o1 != o2 && o3 != o4)
      return true;

  // Special Cases
  // p1, q1 and p2 are colinear and p2 lies on segment p1q1
  if (o1 == 0 && onSegment(LineSegment(p1, q1), p2)) return true;

  // p1, q1 and q2 are colinear and q2 lies on segment p1q1
  if (o2 == 0 && onSegment(LineSegment(p1, q1), q2)) return true;

  // p2, q2 and p1 are colinear and p1 lies on segment p2q2
  if (o3 == 0 && onSegment(LineSegment(p2, q2), p1)) return true;

   // p2, q2 and q1 are colinear and q1 lies on segment p2q2
  if (o4 == 0 && onSegment(LineSegment(p2, q2), q1)) return true;

  return false; // Doesn't fall in any of the above cases
}

bool G::pointOnPolygon(vector<Point> points, Point p) {
  for (int i = 0; i < points.size(); i++) {
    Point a = points[i];
    Point b = points[(i + 1) % points.size()];
    LineSegment ab(a, b);
    if (onSegment(ab, p)) return true;
  }

  return false;
}

bool G::pointInOrOnPolygon(vector<Point> vs, Point p) {
  return pointInPolygon(vs, p);
}
bool G::pointOutOrOnPolygon(vector<Point> vs, Point p) {
  if (pointOnPolygon(vs, p)) return true;
  return (!pointInPolygon(vs, p));
}

vector<Point> G::shortestPathInOrOnPolygon(vector<Point> vs, Point source, Point destination) {
  vector<Point> points;
  points.push_back(source);
  for (auto p: vs) points.push_back(p);
  points.push_back(destination);
  int V = vs.size() + 2;

  // find shortest path from 0 to V - 1;
  int trace[V + 2];
  trace[0] = -1;

  double d[V + 2];
  d[0] = 0;
  for (int i = 1; i < V; i++) {
    d[i] = 1e9;
  }

  std::set<pair<double, int> > st;
  for (int i = 0; i < V; i++) st.insert({d[i], i});

  while (!st.empty()) {
    auto pr = st.begin();
    int id; double dis;
    tie(dis, id) = *pr;
    st.erase(pr);

    if (id == V - 1) break;

    for (int i = 0; i < V; i++) {
      if (i != id && inOrOnPolygon(vs, LineSegment(points[id], points[i]))) {
        if (d[id] + distance(points[id], points[i]) < d[i]) {
          auto it = st.find({d[i], i});
          if (it != st.end()) st.erase(it);

          d[i] = d[id] + distance(points[id], points[i]);
          st.insert({d[i], i});
          trace[i] = id;
        }
      }
    }
  }

  vector<Point> result;
  int current = V - 1;
  while (current != -1) {
    result.push_back(points[current]);
    current = trace[current];
  }

  reverse(result.begin(), result.end());

  return result;
}

vector<Point> G::shortestPathOutOrOnPolygon(vector<Point> vs, Point source, Point destination) {
  vector<Point> points;
  points.push_back(source);
  for (auto p: vs) points.push_back(p);
  points.push_back(destination);
  int V = vs.size() + 2;

  // find shortest path from 0 to V - 1;
  int trace[V + 2];
  trace[0] = -1;

  double d[V + 2];
  d[0] = 0;
  for (int i = 1; i < V; i++) {
    d[i] = 1e9;
  }

  std::set<pair<double, int> > st;
  for (int i = 0; i < V; i++) st.insert({d[i], i});

  while (!st.empty()) {
    auto pr = st.begin();
    int id; double dis;
    tie(dis, id) = *pr;
    st.erase(pr);

    if (id == V - 1) break;

    for (int i = 0; i < V; i++) {
      if (i != id && outOrOnPolygon(vs, LineSegment(points[id], points[i]))) {
        if (d[id] + distance(points[id], points[i]) < d[i]) {
          auto it = st.find({d[i], i});
          if (it != st.end()) st.erase(it);

          d[i] = d[id] + distance(points[id], points[i]);
          st.insert({d[i], i});
          trace[i] = id;
        }
      }
    }
  }

  vector<Point> result;
  int current = V - 1;
  while (current != -1) {
    result.push_back(points[current]);
    current = trace[current];
  }

  reverse(result.begin(), result.end());

  return result;
}
bool G::inOrOnPolygon(vector<Point> vs, LineSegment ls) {
  Point u = ls.u, v = ls.v;
  bool intersected = false;

  for (int i = 0; i < vs.size(); i++) {
    Point a = vs[i];
    Point b = vs[(i + 1) % vs.size()];
    LineSegment ab(a, b);
    if (a == u && b == v) {
      return true;
    }
    if (b == u && a == v) {
      return true;
    }
    if (doIntersect(ab, ls) && (orientation(a, b, u) != 0 && orientation(a, b, v) != 0)) {
      Point X = intersection(ab, ls);
      if (u != X && v != X && a != X && b != X) return false;
    }
    if (onSegment(ab, u) && onSegment(ab, v)) {
      return true;
    }
    if (doIntersect(ab, ls)) intersected = true;
  }

  if (!intersected) return true;

  vector<pair<double, Point> > marks;
  marks.push_back({1, v});
  marks.push_back({0, u});
  for (int i = 0; i < vs.size(); i++) {
    Point a = vs[i];
    Point b = vs[(i + 1) % vs.size()];
    LineSegment ab(a, b);
    if (doIntersect(ab, ls)) {
      for (auto p: intersectionMarks(ab, ls)) {
        double k = Vector(u, p).length() / Vector(u, v).length();
        marks.push_back({k, v});
      }
    }
  }

  std::sort(marks.begin(), marks.end());
  for (int i = 0; i < marks.size() - 1; i++) {
    Point X = marks[i].second;
    Point Y = marks[i + 1].second;
    Point M = Point((X.x() + Y.x()) / 2, (X.y() + Y.y()) / 2);

    if (!pointInOrOnPolygon(vs, M)) return false;
  }

  return true;
}
bool G::outOrOnPolygon(vector<Point> vs, LineSegment ls) {
  Point u = ls.u, v = ls.v;
  bool intersected = false;

  for (int i = 0; i < vs.size(); i++) {
    Point a = vs[i];
    Point b = vs[(i + 1) % vs.size()];
    LineSegment ab(a, b);
    if (a == u && b == v) {
      return true;
    }
    if (b == u && a == v) {
      return true;
    }
    if (onSegment(ab, u) && onSegment(ab, v)) {
      return true;
    }

    if (doIntersect(ab, ls) && (orientation(a, b, u) != 0 && orientation(a, b, v) != 0)) {
      Point X = intersection(ab, ls);
      if (u != X && v != X && a != X && b != X) {
        return false;
      }
    }
    if (doIntersect(ab, ls)) intersected = true;
  }

  if (!intersected) return true;

  vector<pair<double, Point> > marks;
  marks.push_back({1, v});
  marks.push_back({0, u});
  for (int i = 0; i < vs.size(); i++) {
    Point a = vs[i];
    Point b = vs[(i + 1) % vs.size()];
    LineSegment ab(a, b);
    if (doIntersect(ab, ls)) {
      for (auto p: intersectionMarks(ab, ls)) {
        double k = Vector(u, p).length() / Vector(u, v).length();
        marks.push_back({k, v});
      }
    }
  }

  std::sort(marks.begin(), marks.end());
  for (int i = 0; i < marks.size() - 1; i++) {
    Point X = marks[i].second;
    Point Y = marks[i + 1].second;
    Point M = Point((X.x() + Y.x()) / 2, (X.y() + Y.y()) / 2);

    if (!pointOutOrOnPolygon(vs, M)) return false;
  }

  return true;
}


Point G::intersection(LineSegment ls1, LineSegment ls2) {
  Point result;
  G::intersection(ls1.u, ls1.v, ls2.u, ls2.v, result);

  return result;
}

vector<Point> G::intersectionMarks(LineSegment ab, LineSegment uv) {
  // condition: ls1 and ls2 do intersect
  Point a = ab.u, b = ab.v;
  Point u = uv.u, v = uv.v;

  if (onSegment(ab, u) && onSegment(ab, v)) return {u, v};
  if (onSegment(uv, a) && onSegment(uv, b)) return {a, b};

  if (orientation(a, b, u) == 0 && orientation(a, b, v) == 0) {
    if (onSegment(ab, u)) {
      if (onSegment(uv, b)) {
        return {u, b};
      } else {
        return {a, u};
      }
    } else {
      // on segment (ab, v)
      if (onSegment(uv, b)) {
        return {v, b};
      } else {
        return {a, v};
      }
    }
  }


  return {intersection(ab, uv)};
}
bool G::is_intersect(DirectedSegment seg, DirectedArc arc, Point &intersection) {
  if (seg.to == arc.from)  {
    intersection = seg.to;
    return true;
  }

  if (arc.to == seg.from) {
    intersection = seg.from;
    return true;
  }

  if (abs(distance(arc.center, seg.from) - arc.radius) < EPSILON) {
    intersection = seg.from;
    if (Vector(arc.from, intersection).crossProduct(Vector(intersection, arc.to)) <= 0) {
      return true;
    }
  }

  if (abs(distance(arc.center, seg.to) - arc.radius) < EPSILON) {
    intersection = seg.to;
    if (Vector(arc.from, intersection).crossProduct(Vector(intersection, arc.to)) <= 0) {
      return true;
    }
  }


  double dx, dy, A, B, C, det, t;
  Point point1 = seg.from;
  Point point2 = seg.to;
  double radius = arc.radius;
  double cx = arc.center.x();
  double cy = arc.center.y();

  dx = point2.x() - point1.x();
  dy = point2.y() - point1.y();

  A = dx * dx + dy * dy;
  B = 2 * (dx * (point1.x() - cx) + dy * (point1.y() - cy));
  C = (point1.x() - cx) * (point1.x() - cx) +
      (point1.y() - cy) * (point1.y() - cy) -
      radius * radius;

  det = B * B - 4 * A * C;
  if ((A <= 0.0000001) || (det < 0)) {
    return false;
  }
  else if (det == 0) {
    // One solution.
    t = -B / (2 * A);
    if (t >= 0 && t <= 1) {
      intersection = Point(point1.x() + t * dx, point1.y() + t * dy);
      if (Vector(arc.from, intersection).crossProduct(Vector(intersection, arc.to)) <= 0) {
        return true;
      }
    }

    return false;
  }
  else
  {
    // Two solutions.
    t = (double)((-B + sqrt(det)) / (2 * A));
    if (t >= 0 && t <= 1) {
      intersection = Point(point1.x() + t * dx, point1.y() + t * dy);
      if (Vector(arc.from, intersection).crossProduct(Vector(intersection, arc.to)) <= 0) {
        return true;
      }
    }

    t = (double)((-B - sqrt(det)) / (2 * A));
    if (t >= 0 && t <= 1) {
      intersection = Point(point1.x() + t * dx, point1.y() + t * dy);
      if (Vector(arc.from, intersection).crossProduct(Vector(intersection, arc.to)) <= 0) {
        return true;
      }
    }

    return false;
  }

}
bool G::is_intersect(DirectedArc c1, DirectedArc c2, Point &intersection) {
  double r, R, d, dx, dy, cx, cy, Cx, Cy;

  if (c1.radius < c2.radius) {
    r  = c1.radius;  R = c2.radius;
    cx = c1.center.x(); cy = c1.center.y();
    Cx = c2.center.x(); Cy = c2.center.y();
  } else {
    r  = c2.radius; R  = c1.radius;
    Cx = c1.center.x(); Cy = c1.center.y();
    cx = c2.center.x(); cy = c2.center.y();
  }

  // Compute the vector <dx, dy>
  dx = cx - Cx;
  dy = cy - Cy;

  // Find the distance between two points.
  d = sqrt( dx*dx + dy*dy );

  // There are an infinite number of solutions
  // Seems appropriate to also return null
  if (d < EPSILON && abs(R-r) < EPSILON) return false;

  // No intersection (circles centered at the
  // same place with different size)
  else if (d < EPSILON) return false;

  double x = (dx / d) * R + Cx;
  double y = (dy / d) * R + Cy;
  Point P = Point(x, y);

  // Single intersection (kissing circles)
  if (abs((R+r)-d) < EPSILON || abs(R-(r+d)) < EPSILON) {
    // deal with P
    intersection = P;
    if (Vector(c1.from, intersection).crossProduct(Vector(intersection, c1.to)) <= 0) {
      if (Vector(c2.from, intersection).crossProduct(Vector(intersection, c2.to)) <= 0) {
        return true;
      }
    }
  }

  // No intersection. Either the small circle contained within
  // big circle or circles are simply disjoint.
  if ( (d+r) < R || (R+r < d) ) return false;

  Point C = Point(Cx, Cy);
  double angle = acossafe((r*r-d*d-R*R)/(-2.0*d*R));

  intersection = rotatePoint(C, P, +angle);
  if (Vector(c1.from, intersection).crossProduct(Vector(intersection, c1.to)) <= 0) {
    if (Vector(c2.from, intersection).crossProduct(Vector(intersection, c2.to)) <= 0) {
      return true;
    }
  }

  intersection = rotatePoint(C, P, -angle);
  if (Vector(c1.from, intersection).crossProduct(Vector(intersection, c1.to)) <= 0) {
    if (Vector(c2.from, intersection).crossProduct(Vector(intersection, c2.to)) <= 0) {
      return true;
    }
  }

  return false;
}

double G::acossafe(double x) {
  if (x >= +1.0) return 0;
  if (x <= -1.0) return M_PI;
  return acos(x);
}

Point G::rotatePoint(Point fp, Point pt, double a) {
  double x = pt.x() - fp.x();
  double y = pt.y() - fp.y();
  double xRot = x * cos(a) + y * sin(a);
  double yRot = y * cos(a) - x * sin(a);
  return Point(fp.x() + xRot,fp.y() + yRot);
}

bool G::distanceEqual(Point A, Point B, double dis) {
  return abs(distance(A, B) - dis) < EPSILON;
}

vector<Point> G::rollBallCavern(vector<Point> points, double r) {
//  debugPath(points, "red");
  vector<Either> parts;

  // MN is the gate
  Vector MN(points[0], points[points.size() - 1]);
  Point X1 = points[0] + MN.rotate(M_PI / 2) * (r / MN.length());

//  debugCircle(X1, r, "black");
//  debugPoint(X1, "black");
  Point X2 = points[0] + MN.rotate(- M_PI / 2) * (r / MN.length());
  parts.push_back(Either(DirectedArc(X1, X2, points[0], r)));

  for (int i = 0; i < points.size() - 1; i++) {
    Point A = points[i];
    Point B = points[i + 1];
    Vector AB(A, B);
    Point A1 = A + AB.rotate(M_PI / 2) * (r / AB.length());
    Point B1 = B + AB.rotate(M_PI / 2) * (r / AB.length());
    DirectedSegment A1B1(A1, B1);
    parts.push_back(Either(A1B1));
//    debugLine(A1, B1, "black");

    Point B2 = B + AB.rotate(-M_PI / 2) * (r / AB.length());
    DirectedArc B1B2(B1, B2, B, r);
    parts.push_back(Either(B1B2));
//    debugCircle(B, r, "blue");
  }

  DirectedSegment currentSegment = parts[2].directedSegment;
  DirectedArc otherArc = parts[1].directedArc;

  vector<Point> trajectory;
  trajectory.push_back(parts[0].from());
  Either current = parts[0];
  int currentId = 0;
  int nextId = 0;
  while (currentId < parts.size()) {
    Either next;
    bool foundNext = false;
    double minDistance = 1e9;

    if (current.isSegment) {
      DirectedSegment currentSegment = current.directedSegment;
      for (int i = currentId + 1; i < parts.size(); i++) {
        Either other = parts[i];
        if (other.isSegment) {
          DirectedSegment otherSegment = other.directedSegment;
          LineSegment ab(currentSegment.from, currentSegment.to),
                      xy(otherSegment.from, otherSegment.to);
          if (G::doIntersect(ab, xy)) {
            foundNext = true;
            Point M;
            if (currentSegment.to == otherSegment.from) {
              M = currentSegment.to;
            } else {
              M = G::intersection(ab, xy);
            }

            if (G::distance(currentSegment.from, M) < minDistance) {
              minDistance = G::distance(currentSegment.from, M);
              next = Either(DirectedSegment(M, otherSegment.to));
              nextId = i;
            }
          }
        } else if (other.isArc) {
          DirectedArc otherArc = other.directedArc;
          Point intersection;
          if (G::is_intersect(currentSegment, otherArc, intersection)) {
            foundNext = true;
            if (G::distance(currentSegment.from, intersection) < minDistance) {
              minDistance = G::distance(currentSegment.from, intersection);
              next = Either(DirectedArc(intersection, otherArc.to, otherArc.center, otherArc.radius));
              nextId = i;
            }
          }
        }
      }

//      if (foundNext)
//        debugLine(current.from(), next.from(), "green");
    } else {
      DirectedArc currentArc = current.directedArc;
      for (int i = currentId + 1; i < parts.size(); i++) {
        Either other = parts[i];
        if (other.isSegment) {
          DirectedSegment otherSegment = other.directedSegment;
          Point intersection;
          if (G::is_intersect(otherSegment, currentArc, intersection)) {
            foundNext = true;
            if (G::distance(currentArc.from, intersection) < minDistance) {
              minDistance = G::distance(currentArc.from, intersection);
              next = Either(DirectedSegment(intersection, otherSegment.to));
              nextId = i;
            }
          }
        } else if (other.isArc) {
          DirectedArc otherArc = other.directedArc;
          Point intersection;
          if (G::is_intersect(currentArc, otherArc, intersection)) {
            foundNext = true;
            if (G::distance(currentArc.from, intersection) < minDistance) {
              minDistance = G::distance(currentArc.from, intersection);
              next = Either(DirectedArc(intersection, otherArc.to, otherArc.center, otherArc.radius));
              nextId = i;
            }
          }

        }
      }
//      if (foundNext)
//        debugArc(current.from(), next.from(), r, "green");
    }
    if (!foundNext) break;

    trajectory.push_back(next.from());
    current = next;
    currentId = nextId;
  }
  Point J = points[points.size() - 1] + MN.rotate(M_PI / 2) * (r / MN.length());
//  debugPoint(J, "black");
//  debugCircle(J, r, "black");
//  debugArc(trajectory[trajectory.size() - 1], J, r, "green");
  trajectory.push_back(J);
//  debugPolygon(trajectory, "red");

//  for (auto p: trajectory) {
//    int random = rand() % 15;
//    if (random <= 1) {
//      debugPoint(p, "black");
//      debugCircle(p, r, "black");
//    }
////    debugPoint(p, "green");
//  }

//  debugCircle(trajectory[0], r, "green");
  return trajectory;
}
vector<Point> G::rollBallPolygon(vector<Point> points, double r) {

  vector<Either> parts;
  for (int i = 0; i < points.size() - 1; i++) {
    Point A = points[i];
    Point B = points[(i + 1) % points.size()];
    Vector AB(A, B);
    Point A1 = A + AB.rotate(M_PI / 2) * (r / AB.length());
    Point B1 = B + AB.rotate(M_PI / 2) * (r / AB.length());
    DirectedSegment A1B1(A1, B1);
    parts.push_back(Either(A1B1));

    Point B2 = B + AB.rotate(-M_PI / 2) * (r / AB.length());
    DirectedArc B1B2(B1, B2, B, r);
    parts.push_back(Either(B1B2));
  }

  DirectedSegment currentSegment = parts[2].directedSegment;
  DirectedArc otherArc = parts[1].directedArc;

  vector<Point> trajectory;
  trajectory.push_back(parts[0].from());
  Either current = parts[0];
  int currentId = 0;
  int nextId = 0;
  while (currentId < parts.size()) {
    Either next;
    bool foundNext = false;
    double minDistance = 1e9;

    if (current.isSegment) {
      DirectedSegment currentSegment = current.directedSegment;
      for (int i = currentId + 1; i < parts.size(); i++) {
        Either other = parts[i];
        if (other.isSegment) {
          DirectedSegment otherSegment = other.directedSegment;
          LineSegment ab(currentSegment.from, currentSegment.to),
                      xy(otherSegment.from, otherSegment.to);
          if (G::doIntersect(ab, xy)) {
            foundNext = true;
            Point M;
            if (currentSegment.to == otherSegment.from) {
              M = currentSegment.to;
            } else {
              M = G::intersection(ab, xy);
            }

            if (G::distance(currentSegment.from, M) < minDistance) {
              minDistance = G::distance(currentSegment.from, M);
              next = Either(DirectedSegment(M, otherSegment.to));
              nextId = i;
            }
          }
        } else if (other.isArc) {
          DirectedArc otherArc = other.directedArc;
          Point intersection;
          if (G::is_intersect(currentSegment, otherArc, intersection)) {
            foundNext = true;
            if (G::distance(currentSegment.from, intersection) < minDistance) {
              minDistance = G::distance(currentSegment.from, intersection);
              next = Either(DirectedArc(intersection, otherArc.to, otherArc.center, otherArc.radius));
              nextId = i;
            }
          }
        }
      }
    } else {
      DirectedArc currentArc = current.directedArc;
      for (int i = currentId + 1; i < parts.size(); i++) {
        Either other = parts[i];
        if (other.isSegment) {
          DirectedSegment otherSegment = other.directedSegment;
          Point intersection;
          if (G::is_intersect(otherSegment, currentArc, intersection)) {
            foundNext = true;
            if (G::distance(currentArc.from, intersection) < minDistance) {
              minDistance = G::distance(currentArc.from, intersection);
              next = Either(DirectedSegment(intersection, otherSegment.to));
              nextId = i;
            }
          }
        } else if (other.isArc) {
          DirectedArc otherArc = other.directedArc;
          Point intersection;
          if (G::is_intersect(currentArc, otherArc, intersection)) {
            foundNext = true;
            if (G::distance(currentArc.from, intersection) < minDistance) {
              minDistance = G::distance(currentArc.from, intersection);
              next = Either(DirectedArc(intersection, otherArc.to, otherArc.center, otherArc.radius));
              nextId = i;
            }
          }

        }
      }
    }
    if (!foundNext) break;

    trajectory.push_back(next.from());
    current = next;
    currentId = nextId;
  }

//  for (auto p: trajectory) {
//    int random = rand() % 15;
//    if (random <= 0) {
//      debugPoint(p, "black");
//      debugCircle(p, r, "black");
//    }
////    debugPoint(p, "green");
//  }

//  debugPolygon(trajectory, "green");

//  debugCircle(trajectory[0], r, "purple");

  return trajectory;
}

Point G::closestPointOnSegment(LineSegment ls, Point p) {
  Point lp1 = ls.u, lp2 = ls.v;
  double lx1 = lp1.x(), ly1 = lp1.y();
  double lx2 = lp2.x(), ly2 = lp2.y();
  double px = p.x(), py = p.y();

  double ldx = lx2 - lx1,
         ldy = ly2 - ly1,
         lineLengthSquared = ldx*ldx + ldy*ldy;
  double t;
  if (lineLengthSquared == 0) {
    t = 0;
  } else {
    t = ((px - lx1) * ldx + (py - ly1) * ldy) / lineLengthSquared;
    if (t < 0) t = 0;
    else if (t > 1) t = 1;
  }

   double lx = lx1 + t * ldx,
          ly = ly1 + t * ldy;
//          dx = px - lx,
//          dy = py - ly;

  return Point(lx, ly);
}
Point G::closestPointOnPolygon(vector<Point> vs, Point p) {
  double minDis = 1e9;
  Point res;
  for (int i = 0; i < vs.size(); i++) {
    Point A = vs[i];
    Point B = vs[(i + 1) % vs.size()];
    LineSegment AB(A, B);
    Point X = closestPointOnSegment(AB, p);
    if (G::distance(p, X) < minDis) {
      minDis = G::distance(p, X);
      res = X;
    }
  }

  return res;
}
double G::distanceToPolygon(vector<Point> vs, Point p) {
  double minDis = 1e9;
  for (int i = 0; i < vs.size(); i++) {
    Point A = vs[i];
    Point B = vs[(i + 1) % vs.size()];
    LineSegment AB(A, B);
    Point X = closestPointOnSegment(AB, p);
    if (G::distance(p, X) < minDis) {
      minDis = G::distance(p, X);
    }
  }

  return minDis;
}
double G::distanceToLineSegment(LineSegment ls, Point p) {
  Point lp1 = ls.u, lp2 = ls.v;
  double lx1 = lp1.x(), ly1 = lp1.y();
  double lx2 = lp2.x(), ly2 = lp2.y();
  double px = p.x(), py = p.y();

  double ldx = lx2 - lx1,
         ldy = ly2 - ly1,
         lineLengthSquared = ldx*ldx + ldy*ldy;
  double t;
  if (lineLengthSquared == 0) {
    t = 0;
  } else {
    t = ((px - lx1) * ldx + (py - ly1) * ldy) / lineLengthSquared;
    if (t < 0) t = 0;
    else if (t > 1) t = 1;
  }

   double lx = lx1 + t * ldx,
          ly = ly1 + t * ldy,
          dx = px - lx,
          dy = py - ly;

  return sqrt(dx * dx + dy * dy);
}

double G::deepness(vector<Point> cavern) {
  // gate at 0 and size - 1
  LineSegment gate(cavern[0], cavern[cavern.size() - 1]);
  double maxDis = 0;
  for (int i = 0; i < cavern.size(); i++) {
    maxDis = max(maxDis, G::distanceToLineSegment(gate, cavern[i]));
  }


  return maxDis;
}
vector<Point> G::flatten(vector<Point> path) {
  vector<Point> res;
  res.push_back(path[0]);
  for (int i = 0; i < path.size(); i++) {
    if (G::distance(res[res.size() - 1], path[i]) > 5) {
      res.push_back(path[i]);
    }
  }


  return res;
}

double G::diameter(vector<Point> polygon) {
  double maxDis = 0;
  for (int i = 0; i < polygon.size() - 1; i++) {
    for (int j = i + 1; j < polygon.size(); j++) {
      maxDis = max(maxDis, G::distance(polygon[i], polygon[j]));
    }
  }

  return maxDis;
}


tuple<Point, Point> G::hash(vector<Point> vs) {
  double xmin = 1e9, ymin = 1e9, xmax = -1, ymax = -1;
  for (auto p: vs) {
    xmin = min(xmin, p.x());
    ymin = min(ymin, p.y());
    xmax = max(xmax, p.x());
    ymax = max(ymax, p.y());
  }

  return make_tuple(Point(xmin, ymin), Point(xmax, ymax));
}
