#include <iostream>
#include <vector>

static constexpr double kInf = 1e9;
static constexpr double kEps = 1e-7;
static constexpr size_t kBucketsizeThreshold = 1;

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

struct Plane {
  double a;
  double b;
  double c;
  double d;

  int GetSide(const Point& p) const {
    const double value = p.x * a + p.y * b + p.z * c + d;
    if (value > kEps) return 1;
    if (value < -kEps) return -1;
    return 0;
  }

  bool OnLeftSide(const Triangle& t) const {
    return GetSide(t.a) == -1 && GetSide(t.b) == -1 && GetSide(t.c) == -1;
  }
  bool OnRightSide(const Triangle& t) const {
    return GetSide(t.a) == 1 && GetSide(t.b) == 1 && GetSide(t.c) == 1;
  }

  static Plane BuildPlane(const Point& norm, const Point& p) {
    return {norm.x, norm.y, norm.z, -norm.x * p.x - norm.y * p.y - norm.z * p.z};
  }

};

struct BoundingBox {
  Point a;
  Point b;
  BoundingBox() {
    a.x = a.y = a.z = kInf;
    b.x = b.y = b.z = -kInf;
  }
  Plane GetDividingPlane() const {
    const double x = b.x - a.x;
    const double y = b.y - a.y;
    const double z = b.z - a.z;
    if (x >= y && x >= z) {
      return Plane::BuildPlane({1.0, 0.0, 0.0}, {(a.x + b.x) / 2.0, a.y, a.z});
    }
    if (y >= x && y >= z) {
      return Plane::BuildPlane({0.0, 1.0, 0.0}, {a.x, (a.y + b.y) / 2.0, a.z});
    }
    return Plane::BuildPlane({0.0, 0.0, 1.0}, {a.x, a.y, (a.z + b.z) / 2.0});
  }
};

class Object {
 public:
  void Append(Triangle triangle) {
    triangles_.push_back(triangle);
  }
  void Print() const {
    for (const auto &triangle : triangles_) {
      std::cout << triangle << std::endl;
    }
  }

  void Bucketize() {
    const size_t cnt_triangles = triangles_.size();
    triangle_buckets_.resize(cnt_triangles);

    std::vector<size_t> all_triangles(cnt_triangles);
    for (size_t i = 0; i < cnt_triangles; ++i) {
      all_triangles[i] = i;
    }
    Bucketize(all_triangles);

    PrintBuckets();
  }
 private:

  void PrintBuckets() {
    std::cerr << "buckets: " << buckets_.size() << std::endl;
    for (const auto& bucket : buckets_) {
      for (const auto& item : bucket) std::cerr << item << ' ';
      std::cerr << std::endl;
    }
  }

  BoundingBox CalculateBB(const std::vector<size_t>& triangle_indices) {
    BoundingBox bb;

    auto update = [&](const Point& p) {
      bb.a.x = std::min(bb.a.x, p.x);
      bb.b.x = std::max(bb.b.x, p.x);
      bb.a.y = std::min(bb.a.y, p.y);
      bb.b.y = std::max(bb.b.y, p.y);
      bb.a.z = std::min(bb.a.z, p.z);
      bb.b.z = std::max(bb.b.z, p.z);
    };

    for (const size_t i : triangle_indices) {
      update(triangles_.at(i).a);
      update(triangles_.at(i).b);
      update(triangles_.at(i).c);
    }
    return bb;
  }

  void Bucketize(const std::vector<size_t>& triangle_indices) {
    const auto bb = CalculateBB(triangle_indices);
    const auto dividing_plane = bb.GetDividingPlane();
    std::vector<size_t> left_triangles;
    std::vector<size_t> right_triangles;
    for (const size_t i : triangle_indices) {
      const auto triangle = triangles_.at(i);
      if (!dividing_plane.OnLeftSide(triangle)) {
        right_triangles.push_back(i);
      }
      if (!dividing_plane.OnRightSide(triangle)) {
        left_triangles.push_back(i);
      }
    }
    if (triangle_indices.size() <= kBucketsizeThreshold ||
        left_triangles.empty() ||
        right_triangles.empty() ||
        left_triangles.size() == triangle_indices.size() ||
        right_triangles.size() == triangle_indices.size()) {
      const size_t bucket_index = buckets_.size();
      buckets_.emplace_back(triangle_indices);
      for (const size_t i : triangle_indices) {
        triangle_buckets_.at(i).emplace_back(bucket_index);
      }
      return;
    }
    Bucketize(left_triangles);
    Bucketize(right_triangles);
  }

  std::vector<Triangle> triangles_;

  std::vector<std::vector<size_t>> triangle_buckets_;
  std::vector<std::vector<size_t>> buckets_;
};