#include "entities.h"
#include "parser.h"

#include <iostream>


int main() {
  const auto object = Parse("cube.stl");
  object.Print();
  return 0;
}
