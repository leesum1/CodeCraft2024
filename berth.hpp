#pragma once

#include "goods.hpp"
#include "log.h"
#include "point.hpp"
#include <algorithm>
#include <list>
#include <optional>
#include <utility>

struct Berth {
  Area berth_area{};
  Area dock_area{};

  Point pos;
  int transport_time;
  int loading_speed;
  int id;


  std::optional<int> occupied_ship_id = std::nullopt;

  int avg_berth_transport_time = 0; // 平均港口运输时间
  float avg_berth_loading_speed = 0; // 平均港口装货速度

  // 一些信息, 港口周围货物的信息，机器人会优先去货物多的港口
  bool is_baned = false;
  bool tmp_baned = false;
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
                  [&sum](const std::pair<int, int>& p) { sum += p.second; });
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

  float calc_berth_wight(const Point& p) {
    float trans_w =
      avg_berth_transport_time / (float)transport_time; // 越大越好
    float load_w = (float)loading_speed / avg_berth_loading_speed; // 越大越好

    return avg_berth_transport_time;
  }

  int goods_num() { return goods_list.size(); }

  int goods_value() {
    return goods_first_n(goods_list.size()).second;
  }

  /**
   * @brief 获取前n个货物的价值
   *
   * @param n
   * @return std::pair<int,int> first:实际获取的货物数量，second:货物价值
   */
  std::pair<int, int> goods_first_n(int n) {
    int sum = 0;
    auto it = goods_list.begin();
    const int max_n = std::min(n, (int)goods_list.size());

    for (int i = 0; i < max_n; i++) {
      sum += it->money;
      it++;
    }
    return {max_n, sum};
  }

  /**
   * @brief 获取装货的时间
   * @param goods_num
   * @return
   */
  int get_load_cost(const int goods_num) const {
    // 需要向上取整
    return (goods_num + loading_speed - 1) / loading_speed;
  }


  Berth() {}

  Berth(const int id, const Point& pos, const int transport_time,
        const int loading_speed)
    : id(id), pos(pos), transport_time(transport_time),
      loading_speed(loading_speed) {}

  void set_berth_area(const Area& area) { berth_area = area; }
  void set_dock_area(const Area& area) { dock_area = area; }
  bool in_dock_area(const Point& p) { return dock_area.contain(p); }
  bool in_berth_area(const Point& p) { return berth_area.contain(p); }

  bool in_berth_search_area(const Point& p) { return in_berth_area(p); }

  bool is_empty() { return goods_list.empty(); }

  void add_goods(const Goods& g, const int cur_cycle) {
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
    std::for_each(goods_list.begin(), goods_list.end(), [](const Goods& g) {
      log_info("%s", g.to_string().c_str());
    });
  }
};
