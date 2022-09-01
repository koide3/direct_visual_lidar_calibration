#include <memory>
#include <random>
#include <vector>

namespace vlcal {

template <typename T>
class StatisticalMedianFilter {
public:
  using Ptr = std::shared_ptr<StatisticalMedianFilter>;

  StatisticalMedianFilter(const int queue_size, const std::uint32_t seed) : queue_size(queue_size), mt(seed), total_num_data(0) { values.reserve(queue_size); }
  ~StatisticalMedianFilter() {}

  void push(const T value) {
    total_num_data++;
    if (values.size() < queue_size) {
      values.emplace_back(value);
      return;
    }

    const bool do_insert = std::uniform_int_distribution<>(0, total_num_data)(mt) < queue_size;
    if (!do_insert) {
      return;
    }

    values[std::uniform_int_distribution<>(0, queue_size - 1)(mt)] = value;
  }

  T median() {
    std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
    return *(values.begin() + values.size() / 2);
  }

private:
  const int queue_size;
  std::mt19937 mt;

  int total_num_data;
  std::vector<T> values;
};
}