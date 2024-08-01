#include <fmt/core.h>

#include "genshin.h"

void genshin_start() {
  std::string red = "\033[31m";
  std::string reset = "\033[0m";
  fmt::print("{}GenShin Start!{}\n", red, reset);
}