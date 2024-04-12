#pragma once

#include "goods.hpp"
#include "log.h"
#include <utility>
#include <vector>

class Statistic {
    std::vector<Goods> totol_goods_list{}; // 货物价值列表
    std::vector<Goods> goted_goods_list{}; // 已经获取的货物列表
    std::vector<Goods> selled_goods_list{}; // 已经卖出的货物列表
    int total_goods_value_ = 0;
    int goted_goods_value_ = 0;
    int selled_goods_value_ = 0;
public:
    Statistic() = default;

    ~Statistic() = default;

    struct GoodsStatistic {
        int total_value;
        int total_count;
        int total_to_berth_cost;
        std::string name;
        std::array<int, 10> goods_value_step_by_20; // 以 20 为单位,统计货物价值
        std::array<int, 10> goods_cost_step_by_20;  // 以 20 为单位,统计货物到停靠点的距离
        GoodsStatistic() {
            total_value = 0;
            total_count = 0;
            total_to_berth_cost = 0;
            goods_value_step_by_20.fill(0);
            goods_cost_step_by_20.fill(0);
        }
    };




    void add_total_goods(const Goods& goods) { 
        totol_goods_list.push_back(goods); 
        total_goods_value_ += goods.money;
    }
    void add_goted_goods(const Goods& goods) { 
        goted_goods_list.push_back(goods); 
        goted_goods_value_ += goods.money;
    }
    void add_selled_goods(const Goods& goods) { 
        selled_goods_list.push_back(goods);
        selled_goods_value_ += goods.money;
    }

    int total_goods_value() { return total_goods_value_; }
    int total_goods_count() const { return totol_goods_list.size(); }
    int avg_total_goods_value() { return total_goods_value_ / (total_goods_count() + 1); }
    int goted_goods_value() { return goted_goods_value_; }
    int avg_goted_goods_value() { return goted_goods_value_ / (goted_goods_count() + 1); }
    int goted_goods_count() const { return goted_goods_list.size(); }
    int selled_goods_value() { return selled_goods_value_; }
    int selled_goods_count() const { return selled_goods_list.size(); }
    int avg_selled_goods_value() { return selled_goods_value_ / (selled_goods_count() + 1); }


    GoodsStatistic goods_statistic(const std::string& name, const std::vector<Goods>& goods_list) {
        // 以 20 为单位,统计货物价值
        std::array<int, 10> goods_value_step_by_20{};
        GoodsStatistic goods_statistic{};
        goods_statistic.name = name;
        for (const auto& goods : goods_list) {
            int step = goods.money / 20;
            if (step >= 10) {
                step = 9;
            }
            goods_statistic.goods_value_step_by_20[step]++;
            goods_statistic.goods_cost_step_by_20[step] += goods.to_near_berth_cost;
            goods_statistic.total_to_berth_cost += goods.to_near_berth_cost;
            goods_statistic.total_value += goods.money;
            goods_statistic.total_count++;
        }

        log_info("name:%s,value:%d,count:%d,to_berth_cost:%d", goods_statistic.name.c_str(),
                 goods_statistic.total_value, goods_statistic.total_count, goods_statistic.total_to_berth_cost);
        for (int i = 0; i < 10; i++) {
            log_info("[%d-%d]:count: %d, avg_berth_cost: %d, total berth cost: %d", i * 20, (i + 1) * 20 - 1, goods_statistic.goods_value_step_by_20[i],
                     goods_statistic.goods_cost_step_by_20[i] / (goods_statistic.goods_value_step_by_20[i] + 1), goods_statistic.goods_cost_step_by_20[i]);
        }
        return goods_statistic;
    }

    void printf_goods_statistic() {
        const auto total_info =   goods_statistic("total_goods", totol_goods_list);
        const auto goted_info =  goods_statistic("goted_goods", goted_goods_list);
        const auto selled_info =  goods_statistic("selled_goods", selled_goods_list);

        for (int i = 0; i < 10; i++) {
            log_info("[%d-%d]:total:%d,goted:%d,selled:%d", i * 20, (i + 1) * 20 - 1,
                     total_info.goods_value_step_by_20[i], goted_info.goods_value_step_by_20[i],
                     selled_info.goods_value_step_by_20[i]);
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
