#include <cstdio>

#include "config.h"
#include "log.h"
#include "mananger.hpp"

int main() {

#ifdef LOG_ENABLE
  log_init("log.txt", 6);
#endif

  auto m = new Manager();

  try {

    m->init_game();
    // m.io_layer.test_berths_come_from();
    m->run_game();

  } catch (const std::exception &e) {

    log_fatal("exception:%s", e.what());
  }
  delete m;

  return 0;
}
