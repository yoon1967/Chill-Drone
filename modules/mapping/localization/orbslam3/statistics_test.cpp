#include "rolling_statistics.hpp"
#include <iostream>
#include <stdexcept>

int main() {
    RollingStatistics stats;
    if(stats.percentile(.95)!=0) throw std::runtime_error("empty statistics");
    for(int i=1;i<=10000;++i) stats.push(i);
    if(stats.size()!=300 || stats.percentile(.95)!=9985)
        throw std::runtime_error("bounded window or percentile incorrect");
    for(int i=0;i<300;++i) stats.push(2);
    if(stats.percentile(.95)!=2) throw std::runtime_error("expired latency retained");
    std::cout << "Rolling statistics retain only the last 300 frames and expire stale spikes\n";
}
