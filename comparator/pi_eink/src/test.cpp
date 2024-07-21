#include <iostream>
#include <memory>
#include "eink_proto/display.pb.h"
#include "pi_eink/server.hpp"

int main()
{
  DisplayState display_state = SHUTDOWN;
  std::cout << "Hello, World!" << std::endl;
  std::cout << "Display State: " << display_state << std::endl;
  return 0;
}
