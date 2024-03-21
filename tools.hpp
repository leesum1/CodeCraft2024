
#pragma once

#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace Tools {
template <typename T>
std::vector<T> last_n(const std::vector<T> &v, std::size_t n) {
  auto first = std::max(v.size(), n) - n;
  return std::vector<T>(v.begin() + first, v.end());
}

template <typename Key, typename Value>
std::unordered_set<Key> map_to_set(const std::unordered_map<Key, Value> &m) {
  std::unordered_set<Key> s(m.size());
  for (const auto &p : m) {
    s.emplace(p.first);
  }
  return s;
}
} // namespace Tools