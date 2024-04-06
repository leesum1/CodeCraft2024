
#pragma once

#include <complex>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <bits/random.h>

namespace Tools {
  inline int random(const int min, const int max) {
    static std::mt19937 gen((unsigned int)time(nullptr));
    std::uniform_int_distribution dis(min, max);
    return dis(gen);
  }

  double scaled_sigmoid(const double min, const double max, const double center, const double x) {
    // Calculate the sigmoid value
    const double sigmoid_value = 1 / (1 + std::exp(-0.1 * (x - center)));

    // Scale the sigmoid value linearly to the range [min, max]
    const double range = max - min;
    const double scaled_sigmoid = min + sigmoid_value * range;

    return scaled_sigmoid;
  }


  template <typename T>
  std::vector<T> last_n(const std::vector<T>& v, std::size_t n) {
    auto first = std::max(v.size(), n) - n;
    return std::vector<T>(v.begin() + first, v.end());
  }


  template <typename T>
  std::vector<T> first_n(const std::vector<T>& v, std::size_t n) {
    auto last = std::min(v.size(), n);
    return std::vector<T>(v.begin(), v.begin() + last);
  }


  template <typename Key, typename Value>
  std::unordered_set<Key> map_to_set(const std::unordered_map<Key, Value>& m) {
    std::unordered_set<Key> s(m.size());
    for (const auto& p : m) {
      s.emplace(p.first);
    }
    return s;
  }

  template <typename T>
  std::unordered_set<T> vector_to_set(const std::vector<T>& v) {
    std::unordered_set<T> s(v.begin(), v.end());
    return s;
  }
} // namespace Tools
