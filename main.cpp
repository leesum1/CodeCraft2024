#include <algorithm>
#include <cstdio>
#include <vector>

#include "config.h"
#include "log.h"
#include "mananger.hpp"
#include "point.hpp"

void backtrace_test() {
  Point cur_post = Point(3, 3);
  std::vector<Point> orig_path = {Point(1, 1), Point(1, 2), Point(2, 2),
                                  Point(3, 2)};

  // std::reverse(orig_path.begin(), orig_path.end());
  
  std::vector<Point> backtrace_path = {Point(3, 4), Point(3, 5), Point(3, 6),
                                       Point(3, 7), Point(3, 8)};

  std::reverse(backtrace_path.begin(), backtrace_path.end());


  log_info("cur_post:(%d,%d)", cur_post.x, cur_post.y);
  log_info("orig_path.size:%d", orig_path.size());
  for (const auto &p : orig_path) {
    log_info("p:(%d,%d)", p.x, p.y);
  }


  log_info("backtrace_path.size:%d", backtrace_path.size());
  for (const auto &p : backtrace_path) {
    log_info("p:(%d,%d)", p.x, p.y);
  }

  log_info("before add_backtrace_path");
  for (const auto &p : orig_path) {
    log_info("p:(%d,%d)", p.x, p.y);
  }
  PATHHelper::add_backtrace_path(cur_post, orig_path, backtrace_path);

  log_info("after add_backtrace_path");
  for (const auto &p : orig_path) {
    log_info("p:(%d,%d)", p.x, p.y);
  }
}

int main() {

#ifdef LOG_ENABLE
  log_init("log.txt", 6);
#endif
// backtrace_test();
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
