//
// Created by leesum on 24-3-10.
//

#include "io_layer.h"

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <locale>
#include <queue>

#include "log.h"
#include "point.h"
#include <chrono>
#include <utility>

IoLayer::IoLayer() { this->game_map_original = std::make_shared<GameMap>(); }

void IoLayer::init() {
  auto start = std::chrono::high_resolution_clock::now();
  init_game_map();
  init_berths();
  init_ships();
  berths_come_from_init();
  auto end = std::chrono::high_resolution_clock::now();
  log_info("init time:%d ms",
           std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
               .count());
  char okk[100];
  scanf("%s", okk);
  printf("OK\n");
  fflush(stdout);
}

bool IoLayer::init_game_map() {

  game_map_original->height = 200;
  game_map_original->width = 200;
  game_map_original->set_map(new char[200 * 200]);

  char c_buf[210] = {0};

  for (int i = 0; i < 200; i++) {
    memset(c_buf, 0, sizeof(c_buf));
    scanf("%s", c_buf);
    log_raw("%s,len:%d\n", c_buf, strlen(c_buf));
    for (int j = 0; j < 200; j++) {
      game_map_original->write_pos(j, i, c_buf[j]);
    }
  }

  log_info("Game map initialized");
  return true;
}

std::optional<std::vector<Point>> IoLayer::get_berth_path(int berth_id,
                                                          Point from) {
  if (berth_id < 0 || berth_id >= BERTH_NUM) {
    log_fatal("berth id out of range, expect 0~%d, actual:%d", BERTH_NUM,
              berth_id);
    assert(false);
  }

  // find the path
  Point berth_pos =
      Point(berths[berth_id].pos.x + 1, berths[berth_id].pos.y + 1);
  Point current_point = from;
  std::vector<Point> cur_path;
  while (current_point != berth_pos) {
    cur_path.push_back(current_point);
    try {
      current_point = berths_come_from[berth_id].at(current_point).pos;
    } catch (const std::out_of_range &e) {
      log_fatal("berth[%d] come from map not found, from point:(%d,%d)",
                berth_id, from.x, from.y);
      return std::nullopt;
    }
  }

  std::reverse(cur_path.begin(), cur_path.end());
  return cur_path;
}

void IoLayer::berths_come_from_init() {
  for (int i = 0; i < BERTH_NUM; i++) {
    Point start = Point(berths[i].pos.x + 1, berths[i].pos.y + 1);
    std::queue<Point> q;
    q.push(start);
    int search_lvl = 0;
    berths_come_from[i][start] = PointCost(start, search_lvl);
    while (!q.empty()) {
      int level_size = q.size();
      for (int j = 0; j < level_size; j++) {
        Point cur = q.front();
        q.pop();
        for (const auto &next : game_map_original->neighbors(cur)) {
          if (berths_come_from[i].find(next) == berths_come_from[i].end()) {
            berths_come_from[i].emplace(next, PointCost(cur, search_lvl));
            q.push(next);
          }
        }
      }
      search_lvl++;
    }
  }

  log_info("berths_come_from initialized");

  for (int i = 0; i < BERTH_NUM; i++) {
    log_info("berth[%d] come from map size: %d", i, berths_come_from[i].size());
  }
}

void IoLayer::init_berths() {
  for (int i = 0; i < BERTH_NUM; i++) {
    scanf("%d%d%d%d%d", &berths[i].id, &berths[i].pos.x, &berths[i].pos.y,
          &berths[i].transport_time, &berths[i].loading_speed);

    log_info("berth id:%d,x:%d,y:%d,transport_time:%d,loading_speed:%d",
             berths[i].id, berths[i].pos.x, berths[i].pos.y,
             berths[i].transport_time, berths[i].loading_speed);
  }
  log_info("Berths initialized");
}

void IoLayer::init_ships() {
  for (int i = 0; i < SHIP_NUM; i++) {
    scanf("%d", &ships[i].capacity);
    ships[i].id = i;
    log_info("ship[%d] capacity:%d", i, ships[i].capacity);
  }
  log_info("Ships initialized");
}

void IoLayer::input_cycle() {

  scanf("%d%d", &cur_cycle, &cur_money);

  //   scanf("%d%d", &id, &money);
  // int num;
  // scanf("%d", &num);
  // for (int i = 1; i <= num; i++) {
  //   int x, y, val;
  //   scanf("%d%d%d", &x, &y, &val);
  // }

  // 新增的货物
  scanf("%d", &new_goods_num);
  for (int i = 0; i < new_goods_num; i++) {
    scanf("%d%d%d", &new_goods_list[i].pos.x, &new_goods_list[i].pos.y,
          &new_goods_list[i].money);
    new_goods_list[i].is_dead = false;
    new_goods_list[i].be_selected = false;
    new_goods_list[i].start_cycle = cur_cycle;
    new_goods_list[i].end_cycle = cur_cycle + 999;

    // log_trace("new goods x:%d,y:%d,money:%d", new_goods_list[i].pos.x,
    //           new_goods_list[i].pos.y, new_goods_list[i].money);
    total_goods_money += new_goods_list[i].money;
  }
  total_goods_num += new_goods_num;

  // 机器人状态
  for (int i = 0; i < ROBOT_NUM; i++) {
    scanf("%d%d%d%d", &robots[i].goods, &robots[i].x, &robots[i].y,
          &robots[i].status);

    log_trace("robot goods:%d,x:%d,y:%d,status:%d", robots[i].goods,
              robots[i].x, robots[i].y, robots[i].status);
  }

  // 船只状态
  for (int i = 0; i < SHIP_NUM; i++) {
    scanf("%d%d", &ships[i].status, &ships[i].berch_id);

    // log_trace("ship [%d] status:%d,berth_id:%d", i, ships[i].status,
    //           ships[i].berch_id);
    if (ships[i].id != i) {
      log_fatal("ship id not match, expect:%d, actual:%d", i, ships[i].id);
    }
  }

  char okk[10];
  scanf("%s", okk);
  if (strcmp(okk, "OK") != 0) {
    log_fatal("input cycle error, expect OK, actual:%s", okk);
  }
}

void IoLayer::output_cycle() {
  log_trace("cycle[%d],inst_num:%d", cur_cycle, commands.size());
  for (const auto &command : commands) {
    char fmt_buf[50] = {};
    switch (command.inst) {
    case ROBOT_MOVE:
      std::sprintf(fmt_buf, "move %d %d\n", command.arg1, command.arg2);
      break;
    case ROBOT_GET:
      std::sprintf(fmt_buf, "get %d\n", command.arg1);
      break;
    case ROBOT_PULL:
      std::sprintf(fmt_buf, "pull %d\n", command.arg1);
      break;
    case SHIP:
      std::sprintf(fmt_buf, "ship %d %d\n", command.arg1, command.arg2);
      break;
    case GO:
      std::sprintf(fmt_buf, "go %d\n", command.arg1);
      break;
    default:
      log_fatal("unknown command inst:%d", command.inst);
      break;
    }
    printf("%s", fmt_buf);
    log_trace("command:%s", fmt_buf);
  }
  printf("OK\n");
  fflush(stdout);
  commands.clear();
}

void IoLayer::robot_move(int robot_id, RobotDrirection direction) {
  if (robot_id < 0 || robot_id >= ROBOT_NUM) {
    log_fatal("robot id out of range, expect 0~%d, actual:%d", ROBOT_NUM,
              robot_id);
    return;
  }

  commands.push_back({
      ROBOT_MOVE,
      robot_id,
      static_cast<int>(direction),
  });
}

void IoLayer::robot_get(int robot_id) {
  if (robot_id < 0 || robot_id >= ROBOT_NUM) {
    log_fatal("robot id out of range, expect 0~%d, actual:%d", ROBOT_NUM,
              robot_id);
    return;
  }

  commands.push_back({
      ROBOT_GET,
      robot_id,
      0,
  });
}

void IoLayer::robot_pull(int robot_id) {
  if (robot_id < 0 || robot_id >= ROBOT_NUM) {
    log_fatal("robot id out of range, expect 0~%d, actual:%d", ROBOT_NUM,
              robot_id);
    return;
  }

  commands.push_back({
      ROBOT_PULL,
      robot_id,
      0,
  });
}

void IoLayer::ship(int ship_id, int berth_id) {
  if (ship_id < 0 || ship_id >= SHIP_NUM) {
    log_fatal("ship id out of range, expect 0~%d, actual:%d", SHIP_NUM,
              ship_id);
    return;
  }
  if (berth_id < 0 || berth_id >= BERTH_NUM) {
    log_fatal("berth id out of range, expect 0~%d, actual:%d", BERTH_NUM,
              berth_id);
    return;
  }

  commands.push_back({
      SHIP,
      ship_id,
      berth_id,
  });
}

void IoLayer::go(int ship_id) {
  if (ship_id < 0 || ship_id >= SHIP_NUM) {
    log_fatal("ship id out of range, expect 0~%d, actual:%d", SHIP_NUM,
              ship_id);
    return;
  }
  commands.push_back({
      GO,
      ship_id,
      0,
  });
}