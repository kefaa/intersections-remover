#include <iostream>
#include <vector>
#include <set>
#include <map>

struct Point {
  double x;
  double y;
  double z;
  bool operator < (const Point& rhs) const;

  Point Product(const Point& other) const;

  bool IsZero() const;
};

std::ostream& operator << (std::ostream& out, const Point& point);

struct Edge {
  size_t from;
  size_t to;
  Edge& Normalize();
  bool operator < (const Edge& rhs) const;
};

struct Plane;

struct Triangle {
  Point a;
  Point b;
  Point c;
  Point norm;
  Plane BuildPlane() const;
};

std::ostream& operator << (std::ostream& out, const Triangle& triangle);

struct Line {
  Point p0;
  Point dir;

  Point Get(const double t) const;
  Point IntersectWith(const Line& other) const;
};

Point SolveEquation(double a1, double b1, double c1, double a2, double b2, double c2);

struct Plane {
  double a;
  double b;
  double c;
  double d;

  int GetSide(const Point& p) const;

  bool OnLeftSide(const Triangle& t) const;
  bool OnRightSide(const Triangle& t) const;

  static Plane BuildPlane(const Point& norm, const Point& p);

  Point GetNorm() const;

  Point GetIntersectingVector(const Plane& other) const;

  bool IntersectsWith(const Plane& other) const;

  Point FindAnyCommonPoint(const Point& direction, const Plane& other) const;

  Line IntersectWith(const Plane& other) const;
};

struct BoundingBox {
  Point a;
  Point b;
  BoundingBox();

  void Update(const Point& p);

  Plane GetDividingPlane() const ;
};


class Object {
 public:
  void Append(Triangle triangle);
  void Print() const;

  void ProcessData();

  size_t Bucketize();

  void ValidRegionGrowing(const size_t seed_triangle);

 private:
  bool TriangleHasIntersections(const size_t triangle_index) const;

  size_t GetPointIndex(const Point& p);

  BoundingBox CalculateBB(const std::vector<size_t>& triangle_indices);

  size_t Bucketize(const std::vector<size_t>& triangle_indices);

  std::vector<Triangle> triangles_;

  std::vector<Point> points_;
  std::map<Point, size_t> points_index_;

  std::map<Edge, std::vector<size_t>> edges_to_triangles_;
  std::map<size_t, std::set<size_t>> adjacent_triangles_;

  std::vector<std::vector<size_t>> triangle_buckets_;
  std::vector<std::vector<size_t>> buckets_;
};