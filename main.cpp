#include "parser.h"

int main() {
  auto object = parser::Parse("cube.stl");
  object.Print();
  object.Bucketize();
  return 0;
}
