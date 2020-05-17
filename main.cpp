#include "parser.h"

int main() {
  const auto object = parser::Parse("cube.stl");
  object.Print();
  return 0;
}
