#include <iostream>
#include <vector>

struct Point {
  double x;
  double y;
  double z;
  friend std::ostream& operator << (std::ostream& out, const Point& point) {
    out << "(" << point.x << ' ' << point.y << ' ' << point.z << ")";
    return out;
  }
};

struct Triangle {
  Point a;
  Point b;
  Point c;
  Point norm;
  friend std::ostream& operator << (std::ostream& out, const Triangle& triangle) {
    out << triangle.a << ", " << triangle.b << ", " << triangle.c << ", " << triangle.norm;
    return out;
  }
};

class Object {
 public:
  void Append(Triangle triangle) {
    triangles_.push_back(triangle);
  }
  void Print() const {
    for (const auto& triangle : triangles_) {
      std::cout << triangle << std::endl;
    }
  }
 private:
  std::vector<Triangle> triangles_;
};