#include <iostream>
#include <vector>
#include <limits>
#include <cassert>
#include <set>
#include <map>
#include <tuple>
#include <cmath>

#include "entities.h"

static constexpr double kInf = 1e9;
static constexpr double kEps = 1e-7;
static constexpr size_t kBucketSizeThreshold = 1;

std::ostream& operator << (std::ostream& out, const Point& point) {
  out << "(" << point.x << ' ' << point.y << ' ' << point.z << ")";
  return out;
}

bool Point::operator < (const Point& rhs) const {
  return std::tie(x, y, z) < std::tie(rhs.x, rhs.y, rhs.z);
}

Point Point::Product(const Point& other) const {
  return {
      y * other.z - z * other.y,
      other.x * z - other.z * x,
      x * other.y - y * other.x
  };
}

bool Point::IsZero() const {
  return fabs(x) < kEps && fabs(y) < kEps && fabs(z) < kEps;
}

Edge& Edge::Normalize() {
  if (from > to) {
    std::swap(from, to);
  }
  return *this;
}
bool Edge::operator < (const Edge& rhs) const {
  return std::tie(from, to) < std::tie(rhs.from, rhs.to);
}

std::ostream& operator << (std::ostream& out, const Triangle& triangle) {
  out << triangle.a << ", " << triangle.b << ", " << triangle.c << ", " << triangle.norm;
  return out;
}

Plane Triangle::BuildPlane() const {
  return Plane::BuildPlane(norm, a);
}

Point SolveEquation(double a1, double b1, double c1, double a2, double b2, double c2) {
  return {
      (b2 * c1 - b1 * c2) / (a2 * b1 - a1 * b2),
      (a1 * c2 - a2 * c1) / (a2 * b1 - a1 * b2),
      0.0
  };
}

int Plane::GetSide(const Point& p) const {
  const double value = p.x * a + p.y * b + p.z * c + d;
  if (value > kEps) return 1;
  if (value < -kEps) return -1;
  return 0;
}

bool Plane::OnLeftSide(const Triangle& t) const {
  return GetSide(t.a) == -1 && GetSide(t.b) == -1 && GetSide(t.c) == -1;
}
bool Plane::OnRightSide(const Triangle& t) const {
  return GetSide(t.a) == 1 && GetSide(t.b) == 1 && GetSide(t.c) == 1;
}

Plane Plane::BuildPlane(const Point& norm, const Point& p) {
  return {norm.x, norm.y, norm.z, -norm.x * p.x - norm.y * p.y - norm.z * p.z};
}

Point Plane::GetNorm() const {
  return {a, b, c};
}

Point Plane::GetIntersectingVector(const Plane& other) const {
  return GetNorm().Product(other.GetNorm());
}

bool Plane::IntersectsWith(const Plane& other) const {
  return !GetIntersectingVector(other).IsZero();
}

Point Plane::FindAnyCommonPoint(const Point& direction, const Plane& other) const {
  if (fabs(direction.x) > kEps) {
    const auto pt = SolveEquation(b, c, d, other.b, other.c, other.d);
    return {0.0, pt.x, pt.y};
  }
  if (fabs(direction.y) > kEps) {
    const auto pt = SolveEquation(a, c, d, other.a, other.c, other.d);
    return {pt.x, 0.0, pt.y};
  }
  const auto pt = SolveEquation(a, b, d, other.a, other.b, other.d);
  return {pt.x, pt.y, 0.0};
}

Line Plane::IntersectWith(const Plane& other) const {
  const auto direction = GetIntersectingVector(other);
  const auto p0 = FindAnyCommonPoint(direction, other);
  return {p0, direction};
}

BoundingBox::BoundingBox() {
  a.x = a.y = a.z = kInf;
  b.x = b.y = b.z = -kInf;
}

void BoundingBox::Update(const Point& p) {
  a.x = std::min(a.x, p.x);
  b.x = std::max(b.x, p.x);
  a.y = std::min(a.y, p.y);
  b.y = std::max(b.y, p.y);
  a.z = std::min(a.z, p.z);
  b.z = std::max(b.z, p.z);
};

Plane BoundingBox::GetDividingPlane() const {
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

void Object::Append(Triangle triangle) {
  triangles_.push_back(triangle);
}
void Object::Print() const {
  for (const auto &triangle : triangles_) {
    std::cout << triangle << std::endl;
  }
}

void Object::ProcessData() {
  for (size_t triangle_index = 0; triangle_index < triangles_.size(); ++triangle_index) {
    const auto triangle = triangles_.at(triangle_index);
    const size_t a = GetPointIndex(triangle.a);
    const size_t b = GetPointIndex(triangle.b);
    const size_t c = GetPointIndex(triangle.c);

    auto process_edge = [&](const size_t from, const size_t to) {
      const auto edge = Edge{from, to}.Normalize();
      for (const auto& adjacent_triangle_index : edges_to_triangles_[edge]) {
        adjacent_triangles_[adjacent_triangle_index].emplace(triangle_index);
        adjacent_triangles_[triangle_index].emplace(adjacent_triangle_index);
      }
      edges_to_triangles_[edge].emplace_back(triangle_index);
    };

    process_edge(a, b);
    process_edge(b, c);
    process_edge(c, a);
  }
}

size_t Object::Bucketize() {
  const size_t cnt_triangles = triangles_.size();
  triangle_buckets_.resize(cnt_triangles);

  std::vector<size_t> all_triangles(cnt_triangles);
  for (size_t i = 0; i < cnt_triangles; ++i) {
    all_triangles[i] = i;
  }
  size_t seed_triangle = Bucketize(all_triangles);

  return seed_triangle;
}

void Object::ValidRegionGrowing(const size_t seed_triangle) {
  std::set<size_t> visited;
  std::set<size_t> S;
  std::set<size_t> P;
  S.emplace(seed_triangle);
  visited.emplace(seed_triangle);

  while (!S.empty() || !P.empty()) {
    if (!S.empty()) {
      const auto triangle_index = *S.begin();
      S.erase(S.begin());
      for (const auto adjacent_triangle_index : adjacent_triangles_.at(triangle_index)) {
        if (visited.count(adjacent_triangle_index)) {
          continue;
        }
        visited.emplace(adjacent_triangle_index);
        if (!TriangleHasIntersections(adjacent_triangle_index)) {
          S.emplace(adjacent_triangle_index);
        } else {
          // TODO: store info about common edge also
          P.emplace(adjacent_triangle_index);
        }
      }
      continue;
    }
  }
}

bool Object::TriangleHasIntersections(const size_t triangle_index) const {
  std::vector<Line> intersection_lines;
  std::set<size_t> processed_triangles;

  const Plane& triangle_plane = triangles_.at(triangle_index).BuildPlane();
  for (const auto& bucket_index : triangle_buckets_.at(triangle_index)) {
    for (const auto other_triangle : buckets_.at(bucket_index)) {
      if (triangle_index == other_triangle) {
        continue;
      }
      if (adjacent_triangles_.at(triangle_index).count(other_triangle)) {
        continue;
      }
      if (processed_triangles.count(other_triangle)) {
        continue;
      }
      processed_triangles.emplace(other_triangle);

      const Plane& other_triangle_plane = triangles_.at(other_triangle).BuildPlane();
      if (!triangle_plane.IntersectsWith(other_triangle_plane)) {
        continue;
      }
      const Line line = triangle_plane.IntersectWith(other_triangle_plane);
      intersection_lines.push_back(line);
    }
  }
  return !intersection_lines.empty();
}

size_t Object::GetPointIndex(const Point& p) {
  if (!points_index_.count(p)) {
    points_.emplace_back(p);
    const auto index = points_index_.size();
    points_index_[p] = index;
  }
  return points_index_[p];
}

BoundingBox Object::CalculateBB(const std::vector<size_t>& triangle_indices) {
  BoundingBox bb;
  for (const size_t i : triangle_indices) {
    bb.Update(triangles_.at(i).a);
    bb.Update(triangles_.at(i).b);
    bb.Update(triangles_.at(i).c);
  }
  return bb;
}

size_t Object::Bucketize(const std::vector<size_t>& triangle_indices) {
  const auto bb = CalculateBB(triangle_indices);
  const auto dividing_plane = bb.GetDividingPlane();
  std::vector<size_t> left_triangles;
  std::vector<size_t> right_triangles;

  size_t seed_triangle = std::numeric_limits<size_t>::max();
  for (const size_t i : triangle_indices) {
    const auto& triangle = triangles_.at(i);
    const bool is_on_left_side = dividing_plane.OnLeftSide(triangle);
    const bool is_on_right_side = dividing_plane.OnRightSide(triangle);
    if (!is_on_left_side) {
      right_triangles.push_back(i);
    }
    if (!is_on_right_side) {
      left_triangles.push_back(i);
    }
    if (!is_on_left_side && !is_on_right_side) {
      seed_triangle = i;
    }
  }
  assert(seed_triangle != std::numeric_limits<size_t>::max());

  if (triangle_indices.size() <= kBucketSizeThreshold ||
      left_triangles.empty() ||
      right_triangles.empty() ||
      left_triangles.size() == triangle_indices.size() ||
      right_triangles.size() == triangle_indices.size()) {
    const size_t bucket_index = buckets_.size();
    buckets_.emplace_back(triangle_indices);
    for (const size_t i : triangle_indices) {
      triangle_buckets_.at(i).emplace_back(bucket_index);
    }
    return seed_triangle;
  }
  Bucketize(left_triangles);
  Bucketize(right_triangles);
  return seed_triangle;
}
