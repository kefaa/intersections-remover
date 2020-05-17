#include "entities.h"
#include <fstream>
#include <cassert>

namespace parser {

template<typename T>
T ParseSingle(std::ifstream &in) {
  T result;
  in.read((char *) &result, 4);
  return result;
}

Point ParsePoint(std::ifstream &in) {
  const auto x = ParseSingle<float>(in);
  const auto y = ParseSingle<float>(in);
  const auto z = ParseSingle<float>(in);
  return {x, y, z};
}

Triangle ParseTriangle(std::ifstream &in) {
  const auto norm = ParsePoint(in);
  const auto a = ParsePoint(in);
  const auto b = ParsePoint(in);
  const auto c = ParsePoint(in);
  return {a, b, c, norm};
}

Object Parse(const std::string &filename) {
  std::ifstream in = std::ifstream(filename.c_str(), std::ios::in | std::ios::binary);
  assert(in);

  static constexpr size_t kHeaderLength = 80;
  char header[kHeaderLength];
  in.read(header, kHeaderLength);

  const int size = ParseSingle<int>(in);
  Object result;
  for (int i = 0; i < size; ++i) {
    result.Append(ParseTriangle(in));
    in.ignore(2);
  }
  return result;
}

} // namespace parser