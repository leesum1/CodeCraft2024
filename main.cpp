#include <cstdio>

#include "config.h"
#include "log.h"
#include "mananger.hpp"

int main() {

#ifdef LOG_ENABLE
  log_init("log.txt", 6);
#endif

  static int goods_count = 0;

  try {

    auto m = Manager();
    m.init_game();
    // m.io_layer.test_berths_come_from();
    m.test_berths1();

  } catch (const std::exception &e) {

    log_fatal("exception:%s", e.what());
  }

  return 0;
}
