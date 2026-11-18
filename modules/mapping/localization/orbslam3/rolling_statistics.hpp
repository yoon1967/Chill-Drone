#pragma once
#include <algorithm>
#include <deque>
#include <vector>

// Recent latency stays bounded even during hours of live input.
class RollingStatistics {
    std::deque<double> values;
public:
    static constexpr size_t capacity = 300;
    void push(double value) {
        values.push_back(value);
        if (values.size() > capacity) values.pop_front();
    }
    size_t size() const { return values.size(); }
    double percentile(double q) const {
        if (values.empty()) return 0;
        std::vector<double> copy(values.begin(), values.end());
        const size_t index = std::min(copy.size()-1, size_t(q*(copy.size()-1)));
        std::nth_element(copy.begin(), copy.begin()+index, copy.end());
        return copy[index];
    }
};
