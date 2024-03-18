#include <bits/stdc++.h>
#include <cstdio>

#include "log.h"
#include "mananger.hpp"

int main() {
  log_init("log.txt", 6);

  static int goods_count = 0;

  try {

    auto m = Manager();
    m.init_game();
    // m.io_layer.test_berths_come_from();
    m.test_berths1();

  } catch (const std::exception &e) {

    log_fatal("exception:%s", e.what());
  }
  // try {
  //   for (int zhen = 1; zhen <= 15000; zhen++) {
  //     io_layer.input_cycle();
  //     io_layer.output_cycle();
  //   }
  // } catch (const std::exception &e) {

  //   log_fatal("exception:%s", e.what());
  // }

  // log_info("goods_count:%d\n", goods_count);

  // #include "test.h"
  //   a_star_test2();
  // io_layer_test();

  return 0;
}
