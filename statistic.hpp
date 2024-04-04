#pragma once

#include "goods.hpp"
#include "log.h"
#include <vector>

class Statistic {

public:
  Statistic() = default;
  ~Statistic() = default;

  std::vector<Goods> totol_goods_list{};  // 货物价值列表
  std::vector<Goods> goted_goods_list{};  // 已经获取的货物列表
  std::vector<Goods> selled_goods_list{}; // 已经卖出的货物列表

  int total_goods_value() { return goods_value_sum(totol_goods_list); }
  int goted_goods_value() { return goods_value_sum(goted_goods_list); }
  int selled_goods_value() { return goods_value_sum(selled_goods_list); }

  void print_total_goods_value() {
    log_raw("total_goods_value:");
    print_goods_value(totol_goods_list);
  }
  void print_goted_goods_value() {
    log_raw("goted_goods_value:");
    print_goods_value(goted_goods_list);
  }
  void print_selled_goods_value() {
    log_raw("selled_goods_value:");
    print_goods_value(selled_goods_list);
  }

  void print_goods_value(std::vector<Goods> &goods_list) {
    int total_value = 0;
    for (auto &goods : goods_list) {
      total_value += goods.money;
      log_raw("%d\n", goods.money);
    }
    log_info("total_value:%d", total_value);
  }
  int goods_value_sum(std::vector<Goods> &goods_list) {
    int total_value = 0;
    for (auto &goods : goods_list) {
      total_value += goods.money;
    }
    return total_value;
  }
};
