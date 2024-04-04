#include <vector>

#include "config.h"
#include "log.h"
#include "mananger_new.hpp"





int main() {

#ifdef LOG_ENABLE
  log_init("log.txt", 6);
#endif

  auto m = new ManagerNew();
  try {
    m->init_game();
    m->run_game();
  } catch (const std::exception &e) {

    log_fatal("exception:%s", e.what());
  }
  delete m;

  return 0;
}
