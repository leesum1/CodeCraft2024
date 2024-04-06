#pragma once

#include "goods.hpp"
#include "log.h"
#include <vector>

class Statistic {
public:
    Statistic() = default;

    ~Statistic() = default;

    std::vector<Goods> totol_goods_list{}; // 货物价值列表
    std::vector<Goods> goted_goods_list{}; // 已经获取的货物列表
    std::vector<Goods> selled_goods_list{}; // 已经卖出的货物列表

    int total_goods_value() { return goods_value_sum(totol_goods_list); }

    int goted_goods_value() { return goods_value_sum(goted_goods_list); }

    int selled_goods_value() { return goods_value_sum(selled_goods_list); }

    void goods_statistic() {
        // 以 20 为单位,统计货物价值
        std::array<int, 10> totol_goods_value_step_by_20{};
        std::array<int, 10> goted_goods_value_step_by_20{};
        std::array<int, 10> selled_goods_value_step_by_20{};

        for (auto& goods : totol_goods_list) {
            int step = goods.money / 20;
            if (step >= 10) {
                step = 9;
            }
            totol_goods_value_step_by_20[step]++;
        }
        for (auto& goods : goted_goods_list) {
            int step = goods.money / 20;
            if (step >= 10) {
                step = 9;
            }
            goted_goods_value_step_by_20[step]++;
        }
        for (auto& goods : selled_goods_list) {
            int step = goods.money / 20;
            if (step >= 10) {
                step = 9;
            }
            selled_goods_value_step_by_20[step]++;
        }

        for (int i = 0; i < 10; i++) {
            log_info("[%d-%d]:totol:%d,goted:%d,selled:%d", i * 20, (i + 1) * 20 - 1,
                     totol_goods_value_step_by_20[i], goted_goods_value_step_by_20[i],
                     selled_goods_value_step_by_20[i]);
        }
    }

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

    void print_goods_value(std::vector<Goods>& goods_list) {
        int total_value = 0;
        for (auto& goods : goods_list) {
            total_value += goods.money;
        }
        log_info("total_value:%d", total_value);
    }

    int goods_value_sum(std::vector<Goods>& goods_list) {
        int total_value = 0;
        for (auto& goods : goods_list) {
            total_value += goods.money;
        }
        return total_value;
    }
};
