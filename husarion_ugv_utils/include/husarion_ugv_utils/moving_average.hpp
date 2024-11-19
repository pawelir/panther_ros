// Copyright 2024 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HUSARION_UGV_UTILS_MOVING_AVERAGE_HPP_
#define HUSARION_UGV_UTILS_MOVING_AVERAGE_HPP_

#include <deque>

namespace husarion_ugv_utils
{

template <typename T>
class MovingAverage
{
public:
  MovingAverage(const std::size_t window_size = 5, const T initial_value = T(0))
  : window_size_(window_size), initial_value_(initial_value)
  {
  }

  void Roll(const T value)
  {
    values_.push_back(value);

    if (values_.size() > window_size_) {
      values_.pop_front();
    }
  }

  void Reset() { values_.erase(values_.begin(), values_.end()); }

  T GetAverage() const
  {
    if (values_.size() == 0) {
      return initial_value_;
    }

    T sum = T(0);

    for (const auto & value : values_) {
      sum += value / static_cast<T>(values_.size());
    }

    return sum;
  }

private:
  const std::size_t window_size_;
  std::deque<T> values_;
  const T initial_value_;
};

}  // namespace husarion_ugv_utils

#endif  // HUSARION_UGV_UTILS_MOVING_AVERAGE_HPP_
