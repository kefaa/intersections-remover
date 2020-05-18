#include "parser.h"

int main() {

  auto object = parser::Parse("cube.stl");
  object.Print();
  object.ProcessData();
  size_t seed_triangle = object.Bucketize();
  object.ValidRegionGrowing(seed_triangle);
  return 0;
}
