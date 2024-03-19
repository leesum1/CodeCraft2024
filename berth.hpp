#pragma once

#include "goods.hpp"
#include "log.h"
#include "point.hpp"
#include <algorithm>
#include <list>
#include <utility>
struct Berth {
  Point pos;
  int transport_time;
  int loading_speed;
  int id;

  int avg_berth_transport_time = 0;  // 平均港口运输时间
  float avg_berth_loading_speed = 0; // 平均港口装货速度

  // 一些信息, 港口周围货物的信息，机器人会优先去货物多的港口
  int near_goods_num = 0;
  int near_goods_value = 0;
  int near_goods_distance = 0;

  std::list<Goods> goods_list;
  // 1000周期内的货物信息
  // std::pair<放入的时间，货物价值>
  std::list<std::pair<int, int>> goods_list_in_1000cycle;

  void tick(const int cur_cycle) {
    // 清除1000周期外的货物信息
    while (!goods_list_in_1000cycle.empty() &&
           goods_list_in_1000cycle.front().first < (cur_cycle - 1000)) {
      goods_list_in_1000cycle.pop_front();
    }
    clear_goods_info();
  }

  int money_in_1000cycle() {
    int sum = 0;
    std::for_each(goods_list_in_1000cycle.begin(),
                  goods_list_in_1000cycle.end(),
                  [&sum](const std::pair<int, int> &p) { sum += p.second; });
    return sum;
  }

  void clear_goods_info() {
    near_goods_num = 0;
    near_goods_value = 0;
    near_goods_distance = 0;
  }

  void printf_near_goods_info() {
    log_info("Berth id: %d, near_goods_num: %d, near_goods_value: %d, "
             "near_goods_distance: %d",
             id, near_goods_num, near_goods_value, near_goods_distance);
  }

  float calc_berth_wight(const Point &p) {
    float trans_w =
        avg_berth_transport_time / (float)transport_time; // 越大越好
    float load_w = (float)loading_speed / avg_berth_loading_speed; // 越大越好
    return trans_w * load_w;
  }

  int goods_num() { return goods_list.size(); }
  int goods_value() {
    int sum = 0;
    std::for_each(goods_list.begin(), goods_list.end(),
                  [&sum](const Goods &g) { sum += g.money; });
    return sum;
  }

  Berth() {}
  bool in_berth_area(const Point &p) {
    const auto &left_top = pos;
    const auto right_bottom = Point{pos.x + 3, pos.y + 3};

    bool x_in = p.x >= left_top.x && p.x <= right_bottom.x;
    bool y_in = p.y >= left_top.y && p.y <= right_bottom.y;

    return x_in && y_in;
  }

  bool in_berth_search_area(const Point &p) {
    const auto &left_top = Point{pos.x - 2, pos.y - 2};
    const auto right_bottom = Point{pos.x + 5, pos.y + 5};

    bool x_in = p.x >= left_top.x && p.x <= right_bottom.x;
    bool y_in = p.y >= left_top.y && p.y <= right_bottom.y;

    return x_in && y_in;
  }

  bool is_empty() { return goods_list.empty(); }
  void add_goods(const Goods &g, const int cur_cycle) {
    goods_list.push_back(g);
    goods_list_in_1000cycle.push_back(std::make_pair(cur_cycle, g.money));
  }
  Goods get_goods() {
    Goods g = goods_list.front();
    goods_list.pop_front();
    return g;
  }

  void print() {
    log_info(
        "Berth id: %d, pos: (%d, %d), transport_time: %d, loading_speed: %d",
        id, pos.x, pos.y, transport_time, loading_speed);

    log_info("Goods list size: %d, money:%d", goods_list.size(), goods_value());
    std::for_each(goods_list.begin(), goods_list.end(), [](const Goods &g) {
      log_info("%s", g.to_string().c_str());
    });
  }
};