#pragma once

#include "goods.hpp"
#include "log.h"
#include "point.hpp"
#include <algorithm>
#include <list>
struct Berth {
  Point pos;
  int transport_time;
  int loading_speed;
  int id;

  // 一些信息, 港口周围货物的信息，机器人会优先去货物多的港口
  int near_goods_num = 0;
  int near_goods_value = 0;
  int near_goods_distance = 0;

  std::list<Goods> goods_list;

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
    // float w1 = (near_goods_value + 1) / (near_goods_distance * 2);
    float w1 = (near_goods_value) / ((near_goods_distance + 1) * 2);
    return w1;
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
  void add_goods(const Goods &g) { goods_list.push_back(g); }
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