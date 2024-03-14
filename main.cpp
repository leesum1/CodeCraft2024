#include <bits/stdc++.h>
#include <cstdio>

#include "io_layer.hpp"
#include "log.h"
#include "mananger.hpp"

const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int N = 210;

struct Robot1 {
  int x, y, goods;
  int status;
  int mbx, mby;
  Robot1() {}
  Robot1(int startX, int startY) {
    x = startX;
    y = startY;
  }
} robot[robot_num + 10];

struct Berth1 {
  int x;
  int y;
  int transport_time;
  int loading_speed;
  Berth1() {}
  Berth1(int x, int y, int transport_time, int loading_speed) {
    this->x = x;
    this->y = y;
    this->transport_time = transport_time;
    this->loading_speed = loading_speed;
  }
} berth[berth_num + 10];

struct Boat {
  int num, pos, status;
} boat[10];

int money, boat_capacity, id;
char ch[N][N];
int gds[N][N];
void Init() {
  for (int i = 1; i <= n; i++)
    scanf("%s", ch[i] + 1);
  for (int i = 0; i < berth_num; i++) {
    int id;
    scanf("%d", &id);
    scanf("%d%d%d%d", &berth[id].x, &berth[id].y, &berth[id].transport_time,
          &berth[id].loading_speed);
  }
  scanf("%d", &boat_capacity);
  char okk[100];
  scanf("%s", okk);
  printf("OK\n");
  fflush(stdout);
}

int Input() {
  scanf("%d%d", &id, &money);
  int num;
  scanf("%d", &num);
  for (int i = 1; i <= num; i++) {
    int x, y, val;
    scanf("%d%d%d", &x, &y, &val);
  }
  for (int i = 0; i < robot_num; i++) {
    int sts;
    scanf("%d%d%d%d", &robot[i].goods, &robot[i].x, &robot[i].y, &sts);
  }
  for (int i = 0; i < 5; i++)
    scanf("%d%d\n", &boat[i].status, &boat[i].pos);
  char okk[100];
  scanf("%s", okk);
  return id;
}

char scanf_buf[2048];

int main() {
  log_init("log.txt", 6);
  log_info("test");

  static int goods_count = 0;

  try {

    auto m = Manager();
    m.init_game();
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
